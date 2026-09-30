#include "esp_err.h"
#include "esp_log.h"
#include "lsm6dsv320x.h"
#include "led_status.h"
#include "capture.h"
#include "usb.h"
#include "wifi_stream.h"
#include "web_capture.h"

static const char *TAG = "bowvib";

void app_main(void)
{
    esp_err_t err = led_status_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LED status init failed: %s", esp_err_to_name(err));
    }
    err = lsm6dsv320x_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LSM6DSV320X init failed (%s), present mask=0x%X",
                 esp_err_to_name(err), lsm6dsv320x_present_mask());
    }
    err = capture_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "capture task init failed: %s", esp_err_to_name(err));
    }
    err = wifi_stream_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi AP/TCP init failed: %s", esp_err_to_name(err));
    } else {
        err = web_capture_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Browser capture server failed: %s", esp_err_to_name(err));
        }
    }
    err = usb_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB command task init failed: %s", esp_err_to_name(err));
    }
}
