/*
 * main.c — BowVib application entry.
 *
 * Boot: init IIS3DWB10IS (idle), allocate the PSRAM capture buffer + capture
 * task, then start the USB-Serial-JTAG command shell. Capture begins only on a
 * START command from the PC.
 */
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "iis3dwb10is.h"
#include "capture.h"
#include "usb.h"

static const char *TAG = "bowvib";

void app_main(void)
{
    ESP_LOGI(TAG, "BowVib boot — IIS3DWB10IS SPI vibration capture");

    esp_err_t e = iis3dwb10is_init();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "sensor init failed: %s (USB shell still starts)", esp_err_to_name(e));
    }

    e = capture_init();   /* PSRAM buffer + INT1-driven capture task */
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "capture init failed: %s", esp_err_to_name(e));
    }

    usb_start();          /* command shell on the native USB port */

    ESP_LOGI(TAG, "running. USB port = commands/data, UART port = logs.");
}
