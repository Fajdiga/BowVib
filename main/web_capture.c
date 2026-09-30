#include "web_capture.h"

#include "capture.h"
#include "lsm6dsv320x.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"

#include <stdio.h>
#include <string.h>

static httpd_handle_t s_server;
/* Only the HTTP server task writes this descriptor. TX reads it while the
   capture remains busy, so another browser cannot take over its stream. */
static volatile int s_browser_socket = -1;

extern const char s_index_start[] asm("_binary_index_html_start");
extern const char s_index_end[] asm("_binary_index_html_end");
extern const char s_format_start[] asm("_binary_format_js_start");
extern const char s_format_end[] asm("_binary_format_js_end");
extern const char s_capture_start[] asm("_binary_capture_js_start");
extern const char s_capture_end[] asm("_binary_capture_js_end");

bool web_capture_client_connected(void)
{
    return s_browser_socket >= 0;
}

bool web_capture_send(const void *data, size_t len)
{
    const int socket = s_browser_socket;
    if (!s_server || socket < 0 || !data || len == 0) return false;
    httpd_ws_frame_t frame = {
        .type = HTTPD_WS_TYPE_BINARY,
        .payload = (uint8_t *)data,
        .len = len,
    };
    /* Queue the send on the HTTP task and wait for completion. This keeps
       all WebSocket headers, control frames, and payloads serialized. */
    const esp_err_t err = httpd_ws_send_data(s_server, socket, &frame);
    if (err == ESP_OK) (void)httpd_sess_update_lru_counter(s_server, socket);
    if (err != ESP_OK) (void)httpd_sess_trigger_close(s_server, socket);
    return err == ESP_OK;
}

static void close_session(httpd_handle_t server, int socket)
{
    (void)server;
    if (socket == s_browser_socket) {
        s_browser_socket = -1;
        if (capture_browser_active()) capture_stop();
    }
    close(socket);
}

static esp_err_t page_handler(httpd_req_t *request)
{
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    httpd_resp_set_hdr(request, "Connection", "close");
    if (!strcmp(request->uri, "/format.js")) {
        httpd_resp_set_type(request, "application/javascript");
        return httpd_resp_send(request, s_format_start, s_format_end - s_format_start - 1);
    }
    if (!strcmp(request->uri, "/capture.js")) {
        httpd_resp_set_type(request, "application/javascript");
        return httpd_resp_send(request, s_capture_start, s_capture_end - s_capture_start - 1);
    }
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    return httpd_resp_send(request, s_index_start, s_index_end - s_index_start - 1);
}

static esp_err_t status_handler(httpd_req_t *request)
{
    char body[256];
    snprintf(body, sizeof(body),
             "{\"running\":%s,\"present_mask\":%u,\"odr_hz\":%u,\"fs_g\":%u,"
             "\"freq_fine\":[%d,%d,%d,%d]}",
             capture_running() ? "true" : "false", lsm6dsv320x_present_mask(),
             LSM6DSV320X_ODR_HZ, LSM6DSV320X_FS_G,
             lsm6dsv320x_internal_freq_fine(0), lsm6dsv320x_internal_freq_fine(1),
             lsm6dsv320x_internal_freq_fine(2), lsm6dsv320x_internal_freq_fine(3));
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    httpd_resp_set_hdr(request, "Connection", "close");
    return httpd_resp_sendstr(request, body);
}

static esp_err_t reply(httpd_req_t *request, const char *text)
{
    httpd_ws_frame_t frame = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)text,
        .len = strlen(text),
    };
    return httpd_ws_send_frame(request, &frame);
}

static esp_err_t websocket_handler(httpd_req_t *request)
{
    const int socket = httpd_req_to_sockfd(request);
    /* ESP-IDF 6 completes the handshake without calling this handler.
       Reserve the browser session on its first START command instead. */
    uint8_t command[128] = {0};
    httpd_ws_frame_t frame = {0};
    esp_err_t err = httpd_ws_recv_frame(request, &frame, 0);
    if (err != ESP_OK) return err;
    if (frame.len >= sizeof(command)) return ESP_FAIL;
    frame.payload = command;
    err = httpd_ws_recv_frame(request, &frame, sizeof(command) - 1);
    if (err != ESP_OK) return err;
    if (frame.type == HTTPD_WS_TYPE_PONG) return ESP_OK;
    if (frame.type != HTTPD_WS_TYPE_TEXT || frame.len == 0 || frame.len >= 16) return ESP_FAIL;
    if (!capture_control_lock()) return reply(request, "ERR capture_unavailable");
    if (s_browser_socket >= 0 && socket != s_browser_socket) {
        err = reply(request, "ERR browser_busy");
    } else if (!strcmp((char *)command, "STOP")) {
        if (capture_browser_active()) capture_stop();
    } else if (!strcmp((char *)command, "START")) {
        if (capture_running()) err = reply(request, "ERR busy");
        else if (lsm6dsv320x_present_mask() == 0U) err = reply(request, "ERR no_sensors");
        else {
            s_browser_socket = socket;
            /* ACK precedes every binary frame on the same server task. */
            err = reply(request, "CAPTURE START");
            if (err == ESP_OK && !capture_start_browser()) {
                err = reply(request, "ERR capture_start_failed");
            }
        }
    } else if (!capture_browser_active()) {
        err = reply(request, "ERR unknown_command");
    }
    capture_control_unlock();
    return err;
}

esp_err_t web_capture_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 6144;
    config.max_open_sockets = 6;
    config.lru_purge_enable = true;
    config.send_wait_timeout = 1;
    config.recv_wait_timeout = 2;
    config.close_fn = close_session;
    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) return err;
    const httpd_uri_t routes[] = {
        {.uri = "/", .method = HTTP_GET, .handler = page_handler},
        {.uri = "/format.js", .method = HTTP_GET, .handler = page_handler},
        {.uri = "/capture.js", .method = HTTP_GET, .handler = page_handler},
        {.uri = "/status", .method = HTTP_GET, .handler = status_handler},
        {.uri = "/ws", .method = HTTP_GET, .handler = websocket_handler, .is_websocket = true},
    };
    for (unsigned i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
        err = httpd_register_uri_handler(s_server, &routes[i]);
        if (err != ESP_OK) {
            httpd_stop(s_server);
            s_server = NULL;
            return err;
        }
    }
    return ESP_OK;
}
