#include "wifi_stream.h"

#include "capture.h"
#include "lsm6dsv320x.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

#define WIFI_RX_LINE_BYTES 64U

static int s_client_socket = -1;
static SemaphoreHandle_t s_socket_lock;
static bool s_wifi_ready;

bool wifi_stream_ready(void)
{
    return s_wifi_ready;
}

bool wifi_stream_client_connected(void)
{
    if (!s_socket_lock) return false;
    xSemaphoreTake(s_socket_lock, portMAX_DELAY);
    const bool connected = s_client_socket >= 0;
    xSemaphoreGive(s_socket_lock);
    return connected;
}

bool wifi_stream_send(const void *data, size_t len)
{
    if (!data || len == 0 || !s_socket_lock) return false;
    const uint8_t *bytes = (const uint8_t *)data;
    size_t sent = 0;
    bool ok = true;

    xSemaphoreTake(s_socket_lock, portMAX_DELAY);
    if (s_client_socket < 0) {
        ok = false;
    } else {
        while (sent < len) {
            const int count = send(s_client_socket, bytes + sent, len - sent, 0);
            if (count <= 0) {
                ok = false;
                shutdown(s_client_socket, SHUT_RDWR);
                break;
            }
            sent += (size_t)count;
        }
    }
    xSemaphoreGive(s_socket_lock);
    return ok;
}

bool wifi_stream_send_text(const char *text)
{
    return text && wifi_stream_send(text, strlen(text));
}

static void wifi_reply_status(void)
{
    char line[192];
    const uint8_t mask = lsm6dsv320x_present_mask();
    char freq_fine[48];
    size_t pos = 0;
    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i) {
        const int8_t fine = lsm6dsv320x_internal_freq_fine(i);
        const int n = snprintf(freq_fine + pos, sizeof(freq_fine) - pos,
                               "%s%c:%d", i ? "," : "",
                               (char)('A' + i), fine == INT8_MIN ? 999 : fine);
        if (n <= 0 || (size_t)n >= sizeof(freq_fine) - pos) break;
        pos += (size_t)n;
    }
    (void)snprintf(line, sizeof(line),
                   "STATUS running=%d sensors=4 present_mask=%02X odr_hz=%u fs_g=%u freq_fine=%s\n",
                   capture_running() ? 1 : 0, mask,
                   LSM6DSV320X_ODR_HZ, LSM6DSV320X_FS_G, freq_fine);
    (void)wifi_stream_send_text(line);
}

static void handle_wifi_command_unlocked(char *line)
{
    if (capture_running()) {
        if (capture_wifi_active()) {
            if (!strcmp(line, "STOP")) capture_stop();
        } else if (!strcmp(line, "STATUS")) {
            wifi_reply_status();
        } else {
            (void)wifi_stream_send_text("ERR busy\n");
        }
        return;
    }
    if (!strcmp(line, "STATUS")) {
        wifi_reply_status();
    } else if (!strcmp(line, "SCAN")) {
        if (capture_running()) {
            (void)wifi_stream_send_text("ERR busy\n");
        } else {
            const uint8_t mask = lsm6dsv320x_rescan();
            char reply[48];
            (void)snprintf(reply, sizeof(reply), "SCAN present_mask=%02X\n", mask);
            (void)wifi_stream_send_text(reply);
            wifi_reply_status();
        }
    } else if (!strcmp(line, "START")) {
        if (capture_running()) {
            (void)wifi_stream_send_text("ERR busy\n");
        } else if (lsm6dsv320x_present_mask() == 0U) {
            (void)wifi_stream_send_text("ERR no_sensors\n");
        } else {
            char reply[96];
            (void)snprintf(reply, sizeof(reply),
                           "CAPTURE START present_mask=%02X odr_hz=%u fs_g=%u order=1234\n",
                           lsm6dsv320x_present_mask(), LSM6DSV320X_ODR_HZ,
                           LSM6DSV320X_FS_G);
            /* Send the line before starting the high-priority data task so
               that it cannot overtake the text header on the TCP connection. */
            if (!wifi_stream_send_text(reply)) return;
            if (!capture_start_wifi()) {
                (void)wifi_stream_send_text("ERR capture_start_failed\n");
            }
        }
    } else if (!strcmp(line, "STOP")) {
        if (capture_running()) capture_stop();
    } else if (!strcmp(line, "HELP") || !strcmp(line, "?")) {
        (void)wifi_stream_send_text("Commands: STATUS SCAN START STOP HELP\n");
    } else {
        (void)wifi_stream_send_text("ERR unknown command\n");
    }
}

static void handle_wifi_command(char *line)
{
    if (!capture_control_lock()) {
        (void)wifi_stream_send_text("ERR capture_unavailable\n");
        return;
    }
    handle_wifi_command_unlocked(line);
    capture_control_unlock();
}

static void close_client(int client)
{
    const bool was_wifi_capture = capture_wifi_active();
    if (was_wifi_capture) capture_stop();
    xSemaphoreTake(s_socket_lock, portMAX_DELAY);
    if (s_client_socket == client) {
        s_client_socket = -1;
        shutdown(client, SHUT_RDWR);
        close(client);
    }
    xSemaphoreGive(s_socket_lock);
    /* Do not attach another client while the old stream is still draining. */
    while (was_wifi_capture && capture_running()) vTaskDelay(1);
}

static void tcp_server_task(void *arg)
{
    (void)arg;
    const int server = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (server < 0) vTaskDelete(NULL);

    int reuse = 1;
    (void)setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(BOWVIB_WIFI_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(server, (struct sockaddr *)&address, sizeof(address)) != 0 ||
        listen(server, 1) != 0) {
        close(server);
        vTaskDelete(NULL);
    }

    for (;;) {
        struct sockaddr_in peer;
        socklen_t peer_len = sizeof(peer);
        const int client = accept(server, (struct sockaddr *)&peer, &peer_len);
        if (client < 0) continue;
        const struct timeval send_timeout = {.tv_sec = 1, .tv_usec = 0};
        (void)setsockopt(client, SOL_SOCKET, SO_SNDTIMEO,
                         &send_timeout, sizeof(send_timeout));

        xSemaphoreTake(s_socket_lock, portMAX_DELAY);
        s_client_socket = client;
        xSemaphoreGive(s_socket_lock);
        (void)wifi_stream_send_text("BowVib WiFi ready. Send STATUS, SCAN, START, STOP, or HELP.\n");

        char line[WIFI_RX_LINE_BYTES];
        size_t pos = 0;
        uint8_t bytes[32];
        for (;;) {
            const int count = recv(client, bytes, sizeof(bytes), 0);
            if (count <= 0) break;
            for (int i = 0; i < count; ++i) {
                const uint8_t byte = bytes[i];
                if (byte == '\r' || byte == '\n') {
                    if (pos) {
                        line[pos] = '\0';
                        handle_wifi_command(line);
                        pos = 0;
                    }
                } else if (pos < sizeof(line) - 1U) {
                    line[pos++] = (char)byte;
                } else {
                    pos = 0;
                    (void)wifi_stream_send_text("ERR command_too_long\n");
                }
            }
        }
        close_client(client);
    }
}

esp_err_t wifi_stream_start(void)
{
    s_socket_lock = xSemaphoreCreateMutex();
    if (!s_socket_lock) return ESP_ERR_NO_MEM;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) err = nvs_flash_init();
    }
    if (err != ESP_OK) return err;
    err = esp_netif_init();
    if (err != ESP_OK) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK) return err;
    if (!esp_netif_create_default_wifi_ap()) return ESP_FAIL;

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_cfg);
    if (err != ESP_OK) return err;
    wifi_config_t ap_cfg = {0};
    memcpy(ap_cfg.ap.ssid, BOWVIB_WIFI_SSID, sizeof(BOWVIB_WIFI_SSID) - 1U);
    memcpy(ap_cfg.ap.password, BOWVIB_WIFI_PASSWORD,
           sizeof(BOWVIB_WIFI_PASSWORD) - 1U);
    ap_cfg.ap.ssid_len = sizeof(BOWVIB_WIFI_SSID) - 1U;
    ap_cfg.ap.channel = 6;
    /* One station streams IMU data to the host; another may view the host's
       capture page from a phone over the same access point. */
    ap_cfg.ap.max_connection = 2;
    ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap_cfg.ap.pmf_cfg.required = false;

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) return err;
    err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (err != ESP_OK) return err;
    err = esp_wifi_start();
    if (err != ESP_OK) return err;
    if (xTaskCreate(tcp_server_task, "wifi_tcp", 6144, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_wifi_ready = true;
    return ESP_OK;
}
