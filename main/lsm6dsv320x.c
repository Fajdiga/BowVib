#include "lsm6dsv320x.h"

#include <string.h>
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "soc/gpio_struct.h"
#include "hal/spi_ll.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define REG_FIFO_CTRL1         0x07U
#define REG_FIFO_CTRL2         0x08U
#define REG_FIFO_CTRL3         0x09U
#define REG_FIFO_CTRL4         0x0AU
#define REG_COUNTER_BDR_REG1   0x0BU
#define REG_WHO_AM_I           0x0FU
#define REG_CTRL1              0x10U
#define REG_CTRL2              0x11U
#define REG_CTRL3              0x12U
#define REG_CTRL4              0x13U
#define REG_CTRL7              0x16U
#define REG_FIFO_STATUS1       0x1BU
#define REG_FIFO_STATUS2       0x1CU
#define REG_CTRL1_XL_HG        0x4EU
#define REG_INTERNAL_FREQ_FINE 0x4FU
#define REG_FIFO_DATA_OUT_TAG  0x78U
#define REG_TIMESTAMP0         0x40U
#define REG_TIMESTAMP2         0x42U
#define REG_FUNCTIONS_ENABLE   0x50U
#define REG_HAODR_CFG          0x62U

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
static bool s_drdy_monitor;
static bool s_direct_bus;
static volatile uint32_t s_direct_read_max_us;
static uint8_t DRAM_ATTR s_fifo_tx[1U + LSM6DSV320X_FIFO_ROWS_PER_READ * LSM6DSV320X_FIFO_WORD_BYTES];
static uint8_t DRAM_ATTR s_fifo_rx[1U + LSM6DSV320X_FIFO_ROWS_PER_READ * LSM6DSV320X_FIFO_WORD_BYTES];

static esp_err_t transfer_mask(spi_device_handle_t device, uint32_t cs_mask,
                              const void *tx, void *rx, size_t bytes, int64_t *released_us)
{
    if (!device) return ESP_ERR_INVALID_STATE;
    spi_transaction_t transaction = {
        .length = bytes * 8U,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_acquire_bus(device, portMAX_DELAY);
    if (err != ESP_OK) return err;
    GPIO.out_w1tc.val = cs_mask;
    err = spi_device_polling_transmit(device, &transaction);
    GPIO.out_w1ts.val = cs_mask;
    if (released_us) *released_us = esp_timer_get_time();
    spi_device_release_bus(device);
    return err;
}

static esp_err_t transfer(spi_device_handle_t device, const void *tx, void *rx, size_t bytes)
{
    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i)
        if (device && device == s_devices[i])
            return transfer_mask(device, 1U << s_cs_pins[i], tx, rx, bytes, NULL);
    return ESP_ERR_INVALID_STATE;
}

/* Broadcast only write commands: simultaneous reads would contend on MISO. */
static esp_err_t broadcast_write(uint8_t reg, uint8_t value, int64_t *released_us)
{
    uint32_t mask = 0;
    spi_device_handle_t device = NULL;
    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i) {
        if (!(s_present_mask & (1U << i))) continue;
        mask |= 1U << s_cs_pins[i];
        device = s_devices[i];
    }
    uint8_t tx[2] = {reg, value};
    return transfer_mask(device, mask, tx, NULL, sizeof(tx), released_us);
}

static esp_err_t add_spi_device(unsigned imu)
{
    spi_device_interface_config_t cfg = {
        .clock_speed_hz = LSM6DSV320X_SPI_HZ,
        .mode = 3,
        .spics_io_num = -1, /* GPIO CS permits atomic multi-sensor writes. */
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
    uint8_t tx[5] = {(uint8_t)(reg | 0x80U), 0, 0, 0, 0};
    uint8_t rx[5] = {0};
    if (len > 4U) {
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

    /* Both low-g and the powered-down gyro must select HAODR together. */
    err = write_reg(imu, REG_CTRL2, 0x10U);
    if (err != ESP_OK) return err;
    err = write_reg(imu, REG_HAODR_CFG, 0x00U); /* 7680 Hz HAODR selection. */
    if (err != ESP_OK) return err;

    /* HAODR mode also controls the HG sampling clock. Low-g remains unbatched. */
    err = write_reg(imu, REG_CTRL1, 0x1CU);
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

    /* Keep the FIFO bypassed while idle; capture_start enables batching. */
    uint8_t fine;
    s_freq_fine[imu] = read_reg(imu, REG_INTERNAL_FREQ_FINE, &fine) == ESP_OK ?
                       (int8_t)fine : INT8_MIN;
    return ESP_OK;
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

    /* Suspend sensor A's SPI handle while its GPIO CS runs the pin test. */
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

    /* Restore the manually controlled CS and identify/configure A. */
    gpio_set_level((gpio_num_t)s_cs_pins[0], 1);
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
        if (imu < LSM6DSV320X_COUNT) s_overrun_mask |= (uint8_t)(1U << imu);
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
        /* fifo_level already reads STATUS1 then STATUS2 and latches overruns.
           Reading STATUS2 alone violates the BDU ordering requirement. */
        return (s_overrun_mask & (1U << imu)) != 0;
    }
    return true;
}

esp_err_t lsm6dsv320x_capture_start(bool timestamps, int64_t *started_us)
{
    s_overrun_mask = 0;
    esp_err_t err = ESP_OK;
    /* AN6119: at least 500 us after powering all HAODR sensors down. */
    esp_rom_delay_us(600);
    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i) {
        if (!(s_present_mask & (1U << i))) continue;
        err = write_reg(i, REG_FIFO_CTRL4, FIFO_MODE_BYPASS);
        if (err != ESP_OK) goto failed;
        /* FIFO_OVR_LATCHED clears on read. Discard only the previous capture's
           status while batching is disabled, before any new samples enter. */
        uint8_t status[2];
        err = read_burst(i, REG_FIFO_STATUS1, status, sizeof(status));
        if (err != ESP_OK) goto failed;
        err = write_reg(i, REG_CTRL1, 0x1CU);
        if (err != ESP_OK) goto failed;
        /* Only the direct path consumes register outputs. FIFO timing audits
           route DRDY but leave register output disabled, as in FIFO captures. */
        err = write_reg(i, REG_CTRL1_XL_HG, s_drdy_monitor && !timestamps ? 0xBCU : 0x3CU);
        if (err != ESP_OK) goto failed;
        err = write_reg(i, REG_FUNCTIONS_ENABLE, timestamps ? 0x40U : 0U);
        if (err != ESP_OK) goto failed;
    }
    /* With gyro off, HAODR frequency settles after 70 ms (AN6119 3.4). */
    esp_rom_delay_us(75000);
    if (timestamps) {
        err = broadcast_write(REG_TIMESTAMP2, 0xAAU, NULL);
        if (err != ESP_OK) goto failed;
        /* AN6119 requires 400 us before another write after timestamp reset. */
        esp_rom_delay_us(400);
    }
    /* Timestamp every time slot: pair each HG word with its original clock. */
    const uint8_t fifo_config = FIFO_MODE_CONTINUOUS | (timestamps ? 0x40U : 0U);
    err = broadcast_write(REG_FIFO_CTRL4, fifo_config, started_us);
    if (err != ESP_OK) goto failed;
    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i) {
        if (!(s_present_mask & (1U << i))) continue;
        uint8_t actual;
        err = read_reg(i, REG_FIFO_CTRL4, &actual);
        if (err != ESP_OK) goto failed;
        if (actual != fifo_config) { err = ESP_ERR_INVALID_RESPONSE; goto failed; }
        err = read_reg(i, REG_CTRL1, &actual);
        if (err != ESP_OK) goto failed;
        if (actual != 0x1CU) { err = ESP_ERR_INVALID_RESPONSE; goto failed; }
    }
    return ESP_OK;
failed:
    lsm6dsv320x_capture_stop();
    lsm6dsv320x_capture_finish();
    return err;
}

esp_err_t lsm6dsv320x_clock_read(unsigned imu, uint32_t *ticks,
                               int64_t *midpoint_us, uint32_t *span_us)
{
    uint8_t data[4];
    const int64_t before = esp_timer_get_time();
    esp_err_t err = read_burst(imu, REG_TIMESTAMP0, data, sizeof(data));
    const int64_t after = esp_timer_get_time();
    if (err != ESP_OK) return err;
    memcpy(ticks, data, sizeof(*ticks));
    *midpoint_us = before + (after - before) / 2;
    *span_us = (uint32_t)(after - before);
    return ESP_OK;
}

esp_err_t lsm6dsv320x_drdy_monitor(bool enabled)
{
    s_drdy_monitor = enabled;
    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i) {
        if (!(s_present_mask & (1U << i))) continue;
        esp_err_t err = write_reg(i, REG_CTRL4, enabled ? 0x02U : 0U);
        if (err == ESP_OK) err = write_reg(i, REG_CTRL7, enabled ? 0x80U : 0U);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

esp_err_t lsm6dsv320x_direct_start(int64_t *started_us)
{
    esp_err_t err = lsm6dsv320x_drdy_monitor(true);
    if (err == ESP_OK) err = lsm6dsv320x_capture_start(false, started_us);
    if (err == ESP_OK) err = broadcast_write(REG_FIFO_CTRL4, FIFO_MODE_BYPASS, started_us);
    if (err != ESP_OK) return err;
    err = spi_device_acquire_bus(s_devices[0], portMAX_DELAY);
    if (err != ESP_OK) return err;
    s_direct_bus = true;
    /* Configure the fixed 7-byte read using the normal driver, before the ISR
       owns SPI2. IRQ reads then use its 64-byte register buffer, without DMA. */
    uint8_t tx[8] = {0xB4U}, rx[8];
    spi_transaction_t t = {.length = 56U, .tx_buffer = tx, .rx_buffer = rx};
    GPIO.out_w1tc.val = 1U << 16;
    err = spi_device_polling_transmit(s_devices[0], &t);
    GPIO.out_w1ts.val = 1U << 16;
    if (err != ESP_OK) { lsm6dsv320x_direct_release(); return err; }
    spi_dev_t *hw = SPI_LL_GET_HW(LSM6DSV320X_SPI_HOST);
    spi_ll_dma_tx_enable(hw, false); spi_ll_dma_rx_enable(hw, false);
    spi_ll_set_mosi_bitlen(hw, 56U); spi_ll_set_miso_bitlen(hw, 56U);
    spi_ll_apply_config(hw);
    s_direct_read_max_us = 0;
    return ESP_OK;
}

void lsm6dsv320x_direct_release(void)
{
    if (s_direct_bus) {
        spi_device_release_bus(s_devices[0]);
        s_direct_bus = false;
    }
}

bool IRAM_ATTR lsm6dsv320x_direct_read_isr(unsigned imu, int16_t xyz[3])
{
    spi_dev_t *hw = SPI_LL_GET_HW(LSM6DSV320X_SPI_HOST);
    const int64_t before = esp_timer_get_time();
    const uint32_t cs = 1U << (16U + imu);
    hw->dma_int_clr.trans_done = 1;
    hw->data_buf[0].buf = 0xB4U; hw->data_buf[1].buf = 0;
    hw->cmd.update = 1;
    while (hw->cmd.update)
        if (esp_timer_get_time() - before > 40) return false;
    GPIO.out_w1tc.val = cs;
    spi_ll_user_start(hw);
    while (!spi_ll_usr_is_done(hw)) {
        if (esp_timer_get_time() - before > 40) {
            GPIO.out_w1ts.val = cs;
            return false;
        }
    }
    GPIO.out_w1ts.val = cs;
    const uint64_t raw = (uint64_t)hw->data_buf[0].buf | ((uint64_t)hw->data_buf[1].buf << 32);
    xyz[0] = (int16_t)(raw >> 8); xyz[1] = (int16_t)(raw >> 24); xyz[2] = (int16_t)(raw >> 40);
    const uint32_t span = (uint32_t)(esp_timer_get_time() - before);
    if (span > s_direct_read_max_us) s_direct_read_max_us = span;
    return true;
}

uint32_t lsm6dsv320x_direct_read_max_us(void) { return s_direct_read_max_us; }

void lsm6dsv320x_capture_stop(void)
{
    /* Stop both accelerometer ODRs but leave queued FIFO words readable. */
    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i) {
        if (!(s_present_mask & (1U << i))) continue;
        (void)write_reg(i, REG_CTRL1_XL_HG, 0x04U); /* 320 g FS, ODR off */
        (void)write_reg(i, REG_CTRL1, 0x10U); /* HAODR selected, ODR off. */
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
