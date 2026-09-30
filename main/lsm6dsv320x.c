#include "lsm6dsv320x.h"

#include <string.h>
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define REG_FIFO_CTRL1         0x07U
#define REG_FIFO_CTRL2         0x08U
#define REG_FIFO_CTRL3         0x09U
#define REG_FIFO_CTRL4         0x0AU
#define REG_COUNTER_BDR_REG1   0x0BU
#define REG_WHO_AM_I           0x0FU
#define REG_CTRL1              0x10U
#define REG_CTRL3              0x12U
#define REG_FIFO_STATUS1       0x1BU
#define REG_FIFO_STATUS2       0x1CU
#define REG_CTRL1_XL_HG        0x4EU
#define REG_INTERNAL_FREQ_FINE 0x4FU
#define REG_FIFO_DATA_OUT_TAG  0x78U

#define LSM6DSV320X_ID          0x73U
#define FIFO_MODE_BYPASS        0x00U
#define FIFO_MODE_CONTINUOUS    0x06U
#define FIFO_OVR_IA             0x40U
#define FIFO_OVR_LATCHED        0x08U

static const int s_cs_pins[LSM6DSV320X_COUNT] = {16, 17, 18, 19};
static spi_device_handle_t s_devices[LSM6DSV320X_COUNT];
static uint8_t s_present_mask;
static uint8_t s_overrun_mask;
static uint8_t s_whoami[LSM6DSV320X_COUNT];
static int8_t s_freq_fine[LSM6DSV320X_COUNT];
static bool s_bus_initialized;
static uint8_t DRAM_ATTR s_fifo_tx[1U + LSM6DSV320X_FIFO_ROWS_PER_READ * LSM6DSV320X_FIFO_WORD_BYTES];
static uint8_t DRAM_ATTR s_fifo_rx[1U + LSM6DSV320X_FIFO_ROWS_PER_READ * LSM6DSV320X_FIFO_WORD_BYTES];

static esp_err_t transfer(spi_device_handle_t device, const void *tx,
                          void *rx, size_t bytes)
{
    if (!device) return ESP_ERR_INVALID_STATE;
    spi_transaction_t transaction = {
        .length = bytes * 8U,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_polling_transmit(device, &transaction);
}

static esp_err_t add_spi_device(unsigned imu)
{
    spi_device_interface_config_t cfg = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 3,
        .spics_io_num = s_cs_pins[imu],
        .queue_size = 1,
    };
    return spi_bus_add_device(LSM6DSV320X_SPI_HOST, &cfg, &s_devices[imu]);
}

static esp_err_t write_reg(unsigned imu, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};
    return transfer(s_devices[imu], tx, NULL, sizeof(tx));
}

static esp_err_t read_reg(unsigned imu, uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = {(uint8_t)(reg | 0x80U), 0};
    uint8_t rx[2] = {0};
    esp_err_t err = transfer(s_devices[imu], tx, rx, sizeof(tx));
    if (err == ESP_OK) {
        *value = rx[1];
    }
    return err;
}

static esp_err_t read_burst(unsigned imu, uint8_t reg, uint8_t *dst,
                            size_t len)
{
    uint8_t tx[3] = {(uint8_t)(reg | 0x80U), 0, 0};
    uint8_t rx[3] = {0};
    if (len > 2U) {
        return ESP_ERR_INVALID_SIZE;
    }
    esp_err_t err = transfer(s_devices[imu], tx, rx, len + 1U);
    if (err == ESP_OK) {
        memcpy(dst, rx + 1, len);
    }
    return err;
}

static esp_err_t setup_imu(unsigned imu)
{
    uint8_t id = 0;
    esp_err_t err = read_reg(imu, REG_WHO_AM_I, &id);
    s_whoami[imu] = err == ESP_OK ? id : 0xFFU;
    if (err != ESP_OK || id != LSM6DSV320X_ID) {
        s_freq_fine[imu] = INT8_MIN;
        return err == ESP_OK ? ESP_ERR_NOT_FOUND : err;
    }

    /* Reset the device, then enable BDU and register auto-increment. */
    err = write_reg(imu, REG_CTRL3, 0x01U);
    if (err != ESP_OK) return err;
    esp_rom_delay_us(5000);
    err = write_reg(imu, REG_CTRL3, 0x44U);
    if (err != ESP_OK) return err;

    /* The HG channel requires the low-g accelerometer to be in HP or HA mode.
       Keep low-g at 15 Hz in HP mode but do not batch it into the FIFO. */
    err = write_reg(imu, REG_CTRL1, 0x03U);
    if (err != ESP_OK) return err;

    /* FIFO stores only tagged high-g XYZ words (7 bytes per sample). */
    err = write_reg(imu, REG_FIFO_CTRL4, FIFO_MODE_BYPASS);
    if (err != ESP_OK) return err;
    err = write_reg(imu, REG_FIFO_CTRL1, 128U);
    if (err != ESP_OK) return err;
    err = write_reg(imu, REG_FIFO_CTRL2, 0U);
    if (err != ESP_OK) return err;
    err = write_reg(imu, REG_FIFO_CTRL3, 0U);
    if (err != ESP_OK) return err;
    err = write_reg(imu, REG_COUNTER_BDR_REG1, 0x08U);
    if (err != ESP_OK) return err;

    /* ODR=7680 Hz (111), full scale=320 g (100), FIFO-only output. */
    err = write_reg(imu, REG_CTRL1_XL_HG, 0x3CU);
    if (err != ESP_OK) return err;

    /* Continuous mode lets firmware drain the FIFO while acquisition runs. */
    uint8_t fine;
    s_freq_fine[imu] = read_reg(imu, REG_INTERNAL_FREQ_FINE, &fine) == ESP_OK ?
                       (int8_t)fine : INT8_MIN;
    return write_reg(imu, REG_FIFO_CTRL4, FIFO_MODE_CONTINUOUS);
}

esp_err_t lsm6dsv320x_init(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num = LSM6DSV320X_PIN_MOSI,
        .miso_io_num = LSM6DSV320X_PIN_MISO,
        .sclk_io_num = LSM6DSV320X_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    gpio_config_t cs = {
        .pin_bit_mask = (1ULL << s_cs_pins[0]) | (1ULL << s_cs_pins[1]) |
                        (1ULL << s_cs_pins[2]) | (1ULL << s_cs_pins[3]),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cs);
    if (err != ESP_OK) return err;
    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i) {
        gpio_set_level(s_cs_pins[i], 1);
    }

    err = spi_bus_initialize(LSM6DSV320X_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) return err;
    s_bus_initialized = true;

    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i) {
        err = add_spi_device(i);
        if (err != ESP_OK) return err;
    }
    return lsm6dsv320x_rescan() == 0x0FU ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t lsm6dsv320x_cs_a_test(unsigned transitions, uint32_t interval_ms)
{
    if (!s_bus_initialized || transitions == 0 || interval_ms == 0 ||
        s_devices[0] == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* GPIO16 is normally driven by the SPI peripheral as CS_A. Remove that
       routing first so the measured pin is controlled directly by the GPIO. */
    esp_err_t err = spi_bus_remove_device(s_devices[0]);
    if (err != ESP_OK) return err;
    s_devices[0] = NULL;

    gpio_reset_pin((gpio_num_t)s_cs_pins[0]);
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << s_cs_pins[0],
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&config);
    if (err == ESP_OK) {
        int level = 1;
        gpio_set_level((gpio_num_t)s_cs_pins[0], level);
        for (unsigned i = 0; i < transitions; ++i) {
            level = !level;
            gpio_set_level((gpio_num_t)s_cs_pins[0], level);
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
        }
        gpio_set_level((gpio_num_t)s_cs_pins[0], 1);
    }

    /* Return GPIO16 to hardware-CS control and attempt to identify/configure A. */
    gpio_reset_pin((gpio_num_t)s_cs_pins[0]);
    esp_err_t restore_err = add_spi_device(0);
    if (restore_err != ESP_OK) return restore_err;
    const esp_err_t sensor_err = setup_imu(0);
    if (sensor_err == ESP_OK) s_present_mask |= 0x01U;
    else s_present_mask &= (uint8_t)~0x01U;
    return err;
}

uint8_t lsm6dsv320x_rescan(void)
{
    if (!s_bus_initialized) return 0;
    s_present_mask = 0;
    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i) {
        esp_err_t err = ESP_ERR_NOT_FOUND;
        for (unsigned retry = 0; retry < 3U && err != ESP_OK; ++retry) {
            err = setup_imu(i);
            if (err != ESP_OK) esp_rom_delay_us(1000);
        }
        if (err == ESP_OK) s_present_mask |= (uint8_t)(1U << i);
    }
    return s_present_mask;
}

uint8_t lsm6dsv320x_present_mask(void)
{
    return s_present_mask;
}

uint8_t lsm6dsv320x_whoami(unsigned imu)
{
    return imu < LSM6DSV320X_COUNT ? s_whoami[imu] : 0xFFU;
}

int8_t lsm6dsv320x_internal_freq_fine(unsigned imu)
{
    if (imu >= LSM6DSV320X_COUNT || !(s_present_mask & (1U << imu))) {
        return INT8_MIN;
    }
    return s_freq_fine[imu];
}

uint16_t lsm6dsv320x_fifo_level(unsigned imu)
{
    uint8_t status[2] = {0};
    if (imu >= LSM6DSV320X_COUNT || read_burst(imu, REG_FIFO_STATUS1,
                                               status, sizeof(status)) != ESP_OK) {
        return 0;
    }
    if (status[1] & (FIFO_OVR_IA | FIFO_OVR_LATCHED)) {
        s_overrun_mask |= (uint8_t)(1U << imu);
    }
    return (uint16_t)status[0] | ((uint16_t)(status[1] & 0x01U) << 8);
}

bool lsm6dsv320x_fifo_overrun(unsigned imu)
{
    if (imu < LSM6DSV320X_COUNT) {
        uint8_t status = 0;
        if (read_reg(imu, REG_FIFO_STATUS2, &status) == ESP_OK &&
            (status & (FIFO_OVR_IA | FIFO_OVR_LATCHED))) {
            s_overrun_mask |= (uint8_t)(1U << imu);
        }
        return (s_overrun_mask & (1U << imu)) != 0;
    }
    return true;
}

void lsm6dsv320x_capture_start(void)
{
    s_overrun_mask = 0;
    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i) {
        if (!(s_present_mask & (1U << i))) continue;
        (void)write_reg(i, REG_FIFO_CTRL4, FIFO_MODE_BYPASS);
        (void)write_reg(i, REG_CTRL1, 0x03U);
        (void)write_reg(i, REG_CTRL1_XL_HG, 0x3CU);
    }
    esp_rom_delay_us(5000);
    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i) {
        if (!(s_present_mask & (1U << i))) continue;
        (void)write_reg(i, REG_FIFO_CTRL4, FIFO_MODE_CONTINUOUS);
    }
}

void lsm6dsv320x_capture_stop(void)
{
    /* Stop both accelerometer ODRs but leave queued FIFO words readable. */
    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i) {
        if (!(s_present_mask & (1U << i))) continue;
        (void)write_reg(i, REG_CTRL1_XL_HG, 0x04U); /* 320 g FS, ODR off */
        (void)write_reg(i, REG_CTRL1, 0x00U);
    }
}

void lsm6dsv320x_capture_finish(void)
{
    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i) {
        if (!(s_present_mask & (1U << i))) continue;
        (void)write_reg(i, REG_FIFO_CTRL4, FIFO_MODE_BYPASS);
    }
}

esp_err_t lsm6dsv320x_fifo_read(unsigned imu, uint8_t *dst, uint16_t rows)
{
    if (imu >= LSM6DSV320X_COUNT || !dst || rows == 0 ||
        rows > LSM6DSV320X_FIFO_ROWS_PER_READ) {
        return ESP_ERR_INVALID_ARG;
    }
    const size_t payload = (size_t)rows * LSM6DSV320X_FIFO_WORD_BYTES;
    s_fifo_tx[0] = (uint8_t)(REG_FIFO_DATA_OUT_TAG | 0x80U);
    memset(s_fifo_tx + 1, 0, payload);
    esp_err_t err = transfer(s_devices[imu], s_fifo_tx, s_fifo_rx, payload + 1U);
    if (err == ESP_OK) {
        memcpy(dst, s_fifo_rx + 1, payload);
    }
    return err;
}
