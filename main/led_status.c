#include "led_status.h"

#include "capture.h"
#include "lsm6dsv320x.h"
#include "wifi_stream.h"
#include "web_capture.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_ACTIVE_LEVEL 1
#define LED_PERIOD_MS 100U

static const gpio_num_t s_led_pins[] = {GPIO_NUM_3, GPIO_NUM_4, GPIO_NUM_5};
static TaskHandle_t s_led_task;

static void led_write(unsigned led, bool on)
{
    gpio_set_level(s_led_pins[led], on ? LED_ACTIVE_LEVEL : !LED_ACTIVE_LEVEL);
}

static void led_write_all(bool on)
{
    for (unsigned i = 0; i < sizeof(s_led_pins) / sizeof(s_led_pins[0]); ++i) {
        led_write(i, on);
    }
}

static void led_startup_pattern(void)
{
    for (unsigned i = 0; i < sizeof(s_led_pins) / sizeof(s_led_pins[0]); ++i) {
        led_write(i, true);
        vTaskDelay(pdMS_TO_TICKS(180));
        led_write(i, false);
        vTaskDelay(pdMS_TO_TICKS(90));
    }
    led_write_all(true);
    vTaskDelay(pdMS_TO_TICKS(300));
    led_write_all(false);
}

static bool blink_phase(unsigned phase, unsigned period_ticks, unsigned on_ticks)
{
    return (phase % period_ticks) < on_ticks;
}

static void led_status_task(void *arg)
{
    (void)arg;
    unsigned phase = 0;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        const uint8_t present = lsm6dsv320x_present_mask();
        const bool wifi_ready = wifi_stream_ready();
        const bool client = wifi_stream_client_connected() || web_capture_client_connected();
        const bool wifi_capture = capture_wifi_active();
        const bool capture = capture_running();
        const uint8_t overruns = capture_overrun_mask();

        /* LED1: firmware is alive and running. */
        led_write(0, true);

        /* LED2: all four sensors solid, partial discovery slow blink,
           no responding sensors fast blink. */
        if (present == 0x0FU) {
            led_write(1, true);
        } else if (present == 0U) {
            led_write(1, blink_phase(phase, 4U, 2U));
        } else {
            led_write(1, blink_phase(phase, 20U, 10U));
        }

        /* LED3: AP/client/capture state. A latched overrun is shown as a
           double flash after capture until the next capture starts. */
        if (!wifi_ready) {
            led_write(2, false);
        } else if (wifi_capture) {
            led_write(2, blink_phase(phase, 4U, 2U));
        } else if (capture) {
            led_write(2, blink_phase(phase, 10U, 5U));
        } else if (overruns != 0U) {
            const unsigned p = phase % 30U;
            led_write(2, (p < 2U) || (p >= 4U && p < 6U));
        } else if (client) {
            led_write(2, true);
        } else {
            led_write(2, blink_phase(phase, 20U, 2U));
        }

        ++phase;
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LED_PERIOD_MS));
    }
}

esp_err_t led_status_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << GPIO_NUM_3) | (1ULL << GPIO_NUM_4) |
                        (1ULL << GPIO_NUM_5),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) return err;

    led_write_all(false);
    led_startup_pattern();
    if (xTaskCreate(led_status_task, "led_status", 2048, NULL,
                    tskIDLE_PRIORITY + 1U, &s_led_task) != pdPASS) {
        led_write_all(false);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t led_status_test(unsigned cycles)
{
    if (!s_led_task) return ESP_ERR_INVALID_STATE;
    vTaskSuspend(s_led_task);
    for (unsigned cycle = 0; cycle < cycles; ++cycle) {
        for (unsigned led = 0; led < sizeof(s_led_pins) / sizeof(s_led_pins[0]); ++led) {
            led_write(led, true);
            vTaskDelay(pdMS_TO_TICKS(150));
            led_write(led, false);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        led_write_all(true);
        vTaskDelay(pdMS_TO_TICKS(300));
        led_write_all(false);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    vTaskResume(s_led_task);
    return ESP_OK;
}
