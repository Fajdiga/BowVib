/*
 * iis3dwb10is.c — minimal ST IIS3DWB10IS driver (SPI), ESP-IDF.
 * Register map / bit fields from ST iis3dwb10is_reg.h (datasheet DS14585).
 */
#include "iis3dwb10is.h"

#include <string.h>
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "iis3dwb10is";

static spi_device_handle_t s_spi;
static int s_spi_khz;
static bool s_direct_mode;
static bool s_fifo_overrun_latched;
static iis3dwb10is_bits_t s_output_bits = IIS3DWB10IS_BITS_20;

/* INT1 → capture engine plumbing */
static iis3dwb10is_int1_cb_t s_int1_cb;
static void                  *s_int1_arg;

/* Keep FIFO DMA payloads at 1000 bytes. The register address is sent in the
   SPI command phase, so the DMA buffers contain only complete 10-byte rows. */
#define ROWS_PER_XFER 100
static uint8_t DRAM_ATTR s_tx[ROWS_PER_XFER * FIFO_ROW_BYTES];
static uint8_t DRAM_ATTR s_rx[ROWS_PER_XFER * FIFO_ROW_BYTES];

/* One direct XYZ read is 12 bytes (four sign-extended bytes per axis), plus
   the command byte.  Keep both buffers in internal DMA-capable RAM. */
#define DIRECT_DATA_BYTES 12U
static uint8_t DRAM_ATTR s_direct_tx[DIRECT_DATA_BYTES + 1];
static uint8_t DRAM_ATTR s_direct_rx[DIRECT_DATA_BYTES + 1];

/* Centralized transaction helper. */
static esp_err_t spi_transmit(spi_device_handle_t device, spi_transaction_t *t)
{
    return spi_device_polling_transmit(device, t);
}

float iis3dwb10is_fs_to_mglsb(iis3dwb10is_fs_t fs)
{
    switch (fs) {
    case IIS3DWB10IS_FS_50G:  return 0.095f;
    case IIS3DWB10IS_FS_100G: return 0.191f;
    default:                  return 0.381f;   /* ±200 g */
    }
}

float iis3dwb10is_fs_to_mglsb_bits(iis3dwb10is_fs_t fs, iis3dwb10is_bits_t bits)
{
    if (bits == IIS3DWB10IS_BITS_16) {
        switch (fs) {
        case IIS3DWB10IS_FS_50G:  return 1.526f;
        case IIS3DWB10IS_FS_100G: return 3.052f;
        default:                  return 6.104f;
        }
    }
    return iis3dwb10is_fs_to_mglsb(fs);
}

uint8_t iis3dwb10is_read(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0 };
    uint8_t rx[2] = { 0 };
    spi_transaction_t t = {
        .length = 16, .tx_buffer = tx, .rx_buffer = rx,
    };
    spi_transmit(s_spi, &t);
    return rx[1];
}

void iis3dwb10is_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg, val };
    spi_transaction_t t = {
        .length = 16, .tx_buffer = tx, .rx_buffer = NULL,
    };
    spi_transmit(s_spi, &t);
}

void iis3dwb10is_read_burst(uint8_t reg, uint8_t *dst, uint16_t len)
{
    uint8_t tx[32] = { 0 };
    uint8_t rx[32] = { 0 };
    tx[0] = (uint8_t)(reg | 0x80);
    spi_transaction_t t = {
        .length = (uint32_t)(1 + len) * 8, .tx_buffer = tx, .rx_buffer = rx,
    };
    spi_transmit(s_spi, &t);
    memcpy(dst, rx + 1, len);
}

bool iis3dwb10is_whoami_ok(void)
{
    return iis3dwb10is_read(REG_WHO_AM_I) == IIS3DWB10IS_ID;
}

uint16_t iis3dwb10is_fifo_level(void)
{
    uint8_t b[2];
    iis3dwb10is_read_burst(REG_FIFO_STATUS1, b, 2);
    if (b[1] & 0x40U) {
        s_fifo_overrun_latched = true;
    }
    return (uint16_t)b[0] | (((uint16_t)(b[1] & 0x0F)) << 8);
}

bool iis3dwb10is_fifo_overrun(void)
{
    if (iis3dwb10is_read(REG_FIFO_STATUS2) & 0x40U) {
        s_fifo_overrun_latched = true;
    }
    return s_fifo_overrun_latched;
}

void iis3dwb10is_fifo_clear_overrun(void)
{
    (void)iis3dwb10is_read(REG_FIFO_STATUS2);
    s_fifo_overrun_latched = false;
}

void iis3dwb10is_fifo_flush_running(void)
{
    /* Bypass clears the complete FIFO immediately while the accelerometer
       remains in continuous mode. Entering bypass also clears FIFO_EN on this
       part, so restore stream/batching first and then re-enable FIFO_EN. */
    iis3dwb10is_write(REG_FIFO_CTRL3, FIFO_CTRL3_MODE_BYPASS);
    esp_rom_delay_us(50);
    iis3dwb10is_fifo_clear_overrun();
    iis3dwb10is_write(REG_FIFO_CTRL3,
                      FIFO_CTRL3_XL_BATCH | FIFO_CTRL3_MODE_CONTINUOUS);
    iis3dwb10is_write(REG_CTRL3, CTRL3_BDU | CTRL3_FIFO_EN | CTRL3_IF_INC);
}

void iis3dwb10is_fifo_read(uint8_t *dst, uint16_t nrows)
{
    uint16_t off = 0;
    while (nrows) {
        uint16_t n = (nrows > ROWS_PER_XFER) ? ROWS_PER_XFER : nrows;
        uint16_t len = n * FIFO_ROW_BYTES;
        spi_transaction_ext_t t = { 0 };
        t.base.flags = SPI_TRANS_VARIABLE_CMD;
        t.base.cmd = (uint16_t)(REG_FIFO_OUT | 0x80U);
        t.base.length = (uint32_t)len * 8U;
        t.base.rxlength = (uint32_t)len * 8U;
        t.base.tx_buffer = s_tx;
        t.base.rx_buffer = s_rx;
        t.command_bits = 8;
        spi_transmit(s_spi, &t.base);
        memcpy(dst + off, s_rx, len);   /* dst may be PSRAM (CPU copy; DMA can't reach it) */
        off += len;
        nrows -= n;
    }
}

void iis3dwb10is_start(iis3dwb10is_odr_t odr, iis3dwb10is_fs_t fs)
{
    /* At 2.5/5/10 kS/s the direct DRDY path avoids FIFO watermark latency and
       still leaves ample SPI/USB headroom.  Higher ODRs retain FIFO batching. */
    s_direct_mode = (odr == IIS3DWB10IS_ODR_2P5K ||
                     odr == IIS3DWB10IS_ODR_5K ||
                     odr == IIS3DWB10IS_ODR_10K);
    iis3dwb10is_write(REG_CTRL1, IIS3DWB10IS_ODR_IDLE);       /* pause */
    iis3dwb10is_write(REG_CTRL2, (uint8_t)(fs << 5));         /* full-scale */
    /* Autoswitch selects the datasheet-recommended LPF1 for each ODR and the
       full 20 kHz LPF1 setting at 40/80 kS/s. */
    iis3dwb10is_write(REG_ST_CTRL, ST_CTRL_LPF1_AUTO);
    iis3dwb10is_write(REG_FIFO_CTRL3, FIFO_CTRL3_MODE_BYPASS); /* flush FIFO */
    esp_rom_delay_us(50);
    iis3dwb10is_fifo_clear_overrun();
    if (s_direct_mode) {
        /* No FIFO: INT1 asserts for each accelerometer data-ready sample. */
        iis3dwb10is_write(REG_CTRL3, CTRL3_BDU | CTRL3_IF_INC);
        iis3dwb10is_write(REG_CTRL4, (uint8_t)(CTRL4_AXES |
                                      (s_output_bits == IIS3DWB10IS_BITS_16 ?
                                       CTRL4_ROUNDING_16 : 0U)));
        iis3dwb10is_write(REG_INT_CTRL2, INT1_DRDY_XL);
    } else {
        iis3dwb10is_write(REG_FIFO_CTRL3,
                          FIFO_CTRL3_XL_BATCH | FIFO_CTRL3_MODE_CONTINUOUS);
        /* Entering bypass clears FIFO_EN on this part. Re-enable it after the
           FIFO mode write, as ST's fifo_mode_set() implementation does. */
        iis3dwb10is_write(REG_CTRL3, CTRL3_BDU | CTRL3_FIFO_EN | CTRL3_IF_INC);
        /* Do not drive FIFO events onto INT1. Sustained 80 kS/s testing showed
           the sensor resetting after a variable number of watermark edges.
           The capture task polls the 2048-row FIFO every RTOS tick instead. */
        iis3dwb10is_write(REG_INT_CTRL2, 0x00);
    }
    /* ODR transitions require at least 0.45 ms of serial-interface silence
       immediately before the ODR write, after all other configuration. */
    esp_rom_delay_us(500);
    iis3dwb10is_write(REG_CTRL1, (uint8_t)odr);               /* continuous @ odr */
}

void iis3dwb10is_stop(void)
{
    iis3dwb10is_write(REG_INT_CTRL2, 0x00);                   /* disable INT1 */
    iis3dwb10is_write(REG_CTRL1, IIS3DWB10IS_ODR_IDLE);
    iis3dwb10is_write(REG_FIFO_CTRL3, FIFO_CTRL3_MODE_BYPASS); /* bypass/flush */
    s_direct_mode = false;
}

bool iis3dwb10is_direct_mode(void)
{
    return s_direct_mode;
}

void iis3dwb10is_set_output_bits(iis3dwb10is_bits_t bits)
{
    s_output_bits = (bits == IIS3DWB10IS_BITS_16) ?
                    IIS3DWB10IS_BITS_16 : IIS3DWB10IS_BITS_20;
}

iis3dwb10is_bits_t iis3dwb10is_output_bits(void)
{
    return s_output_bits;
}

int iis3dwb10is_spi_khz(void) { return s_spi_khz; }

bool iis3dwb10is_data_ready(void)
{
    return s_direct_mode && ((iis3dwb10is_read(REG_STATUS_REG) & 0x01U) != 0U);
}

void iis3dwb10is_temperature_read(int16_t *raw, float *celsius)
{
    uint8_t bytes[2] = { 0 };
    iis3dwb10is_read_burst(REG_OUT_TEMP_L, bytes, sizeof(bytes));
    int16_t value = (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
    if (raw) {
        *raw = value;
    }
    if (celsius) {
        *celsius = 25.0f + (float)value / 200.0f;
    }
}

bool iis3dwb10is_direct_read_row(uint8_t *dst)
{
    if (!dst || !s_direct_mode) {
        return false;
    }

    const unsigned data_bytes = (s_output_bits == IIS3DWB10IS_BITS_16) ? 6U : 12U;
    s_direct_tx[0] = (uint8_t)(REG_OUTX_L_A | 0x80U);
    memset(s_direct_tx + 1, 0, data_bytes);
    spi_transaction_t t = {
        .length = (data_bytes + 1U) * 8U,
        .tx_buffer = s_direct_tx,
        .rx_buffer = s_direct_rx,
    };
    /* Queue + wait for the DMA completion rather than busy-polling the SPI
       peripheral.  The task is awakened by INT1 and does no register polling. */
    esp_err_t err = spi_device_queue_trans(s_spi, &t, portMAX_DELAY);
    if (err != ESP_OK) {
        return false;
    }
    spi_transaction_t *done = NULL;
    err = spi_device_get_trans_result(s_spi, &done, portMAX_DELAY);
    if (err != ESP_OK) {
        return false;
    }
    (void)done;

    /* Preserve the existing host framing: tag + three little-endian fields in
       20-bit containers.  In 16-bit mode CTRL4 rounding makes a six-byte
       burst from OUTX_L_A return X_L/X_M, Y_L/Y_M, Z_L/Z_M. */
    dst[0] = 0x10U;
    for (unsigned axis = 0; axis < 3; ++axis) {
        const unsigned off = 1U + axis *
                             (s_output_bits == IIS3DWB10IS_BITS_16 ? 2U : 4U);
        uint32_t raw;
        if (s_output_bits == IIS3DWB10IS_BITS_16) {
            int16_t value = (int16_t)((uint16_t)s_direct_rx[off] |
                                      ((uint16_t)s_direct_rx[off + 1U] << 8));
            raw = (uint32_t)((int32_t)value) & 0x000FFFFFU;
        } else {
            raw = (uint32_t)s_direct_rx[off] |
                  ((uint32_t)s_direct_rx[off + 1U] << 8) |
                  ((uint32_t)s_direct_rx[off + 2U] << 16) |
                  ((uint32_t)s_direct_rx[off + 3U] << 24);
            raw &= 0x000FFFFFU;
        }
        const unsigned out = 1U + axis * 3U;
        dst[out] = (uint8_t)raw;
        dst[out + 1U] = (uint8_t)(raw >> 8);
        dst[out + 2U] = (uint8_t)(raw >> 16);
    }
    return true;
}

static void IRAM_ATTR int1_isr(void *arg)
{
    (void)arg;
    if (s_int1_cb) {
        s_int1_cb(s_int1_arg);
    }
}

void iis3dwb10is_set_int1_callback(iis3dwb10is_int1_cb_t cb, void *arg)
{
    s_int1_cb = cb;
    s_int1_arg = arg;
}

esp_err_t iis3dwb10is_init(void)
{
    s_output_bits = IIS3DWB10IS_BITS_20;
    memset(s_tx, 0, sizeof(s_tx));
    memset(s_direct_tx, 0, sizeof(s_direct_tx));

    spi_bus_config_t buscfg = {
        .miso_io_num = IIS3DWB10IS_PIN_MISO,
        .mosi_io_num = IIS3DWB10IS_PIN_MOSI,
        .sclk_io_num = IIS3DWB10IS_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = IIS3DWB10IS_SPI_HZ,
        .mode = 3,                       /* SPI mode 3 (CPOL=1, CPHA=1) */
        .spics_io_num = IIS3DWB10IS_PIN_CS,
        .queue_size = 4,
        .flags = 0,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(IIS3DWB10IS_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(IIS3DWB10IS_SPI_HOST, &devcfg, &s_spi));
    ESP_ERROR_CHECK(spi_device_get_actual_freq(s_spi, &s_spi_khz));

    /* Software edge reduction was tested at the two weaker drive settings,
       but both caused missed/corrupted FIFO traffic. Use the normal setting;
       MISO is driven by the sensor and cannot be adjusted from the ESP32. */
    ESP_ERROR_CHECK(gpio_set_drive_capability(IIS3DWB10IS_PIN_SCK,
                                               GPIO_DRIVE_CAP_2));
    ESP_ERROR_CHECK(gpio_set_drive_capability(IIS3DWB10IS_PIN_MOSI,
                                               GPIO_DRIVE_CAP_2));
    ESP_ERROR_CHECK(gpio_set_drive_capability(IIS3DWB10IS_PIN_CS,
                                               GPIO_DRIVE_CAP_2));
    ESP_LOGI(TAG, "SPI=%d kHz for registers and FIFO, drive=normal", s_spi_khz);

    /* INT1 is used for data-ready in low-rate direct-register mode. High-rate
       FIFO capture is polled to avoid a board-level reset seen on watermark
       edges at 80 kS/s. */
    gpio_config_t g = {
        .pin_bit_mask = (1ULL << IIS3DWB10IS_PIN_INT1),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&g);
    (void)gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(IIS3DWB10IS_PIN_INT1, int1_isr, NULL);

    if (!iis3dwb10is_whoami_ok()) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch (expected 0x%02X) — check wiring/CS", IIS3DWB10IS_ID);
        return ESP_ERR_NOT_FOUND;
    }

    /* Reload the complete power-on configuration and calibration. ST's
       reference example uses SW_POR (not the shorter UI-only SW_RESET) and
       specifies a 20 ms completion delay. */
    iis3dwb10is_write(REG_CTRL1, IIS3DWB10IS_ODR_IDLE);
    vTaskDelay(pdMS_TO_TICKS(1));
    iis3dwb10is_write(REG_RAM_ACCESS, RAM_ACCESS_SW_POR);
    vTaskDelay(pdMS_TO_TICKS(20));
    iis3dwb10is_write(REG_CTRL3, CTRL3_BDU | CTRL3_FIFO_EN | CTRL3_IF_INC);   /* 0x54 */
    /* DS14585 section 4.4 requires 1.5 ms after enabling FIFO before any
       remaining sensor configuration. Keep the serial interface idle here. */
    esp_rom_delay_us(1500);
    iis3dwb10is_write(REG_CTRL4, CTRL4_AXES);                                 /* 0x1C */
    iis3dwb10is_write(REG_CTRL2, (uint8_t)(IIS3DWB10IS_FS_200G << 5));        /* ±200 g */
    iis3dwb10is_write(REG_ST_CTRL, ST_CTRL_LPF1_AUTO);                        /* ODR autoswitch */
    iis3dwb10is_write(REG_CTRL1, IIS3DWB10IS_ODR_IDLE);                       /* idle */
    iis3dwb10is_write(REG_FIFO_CTRL1, (uint8_t)(FIFO_WATERMARK & 0xFF));
    iis3dwb10is_write(REG_FIFO_CTRL2, (uint8_t)((FIFO_WATERMARK >> 8) & 0x0F));
    iis3dwb10is_write(REG_INT_CTRL2, 0x00);                                   /* FIFO is polled */
    iis3dwb10is_write(REG_FIFO_CTRL3, FIFO_CTRL3_MODE_BYPASS);                 /* bypass */

    /* Readback self-check: catches a wrong write immediately at boot. */
    uint8_t c2 = iis3dwb10is_read(REG_CTRL2);
    uint8_t c3 = iis3dwb10is_read(REG_CTRL3);
    uint8_t st = iis3dwb10is_read(REG_ST_CTRL);
    ESP_LOGI(TAG, "ready. WHO_AM_I=0x%02X CTRL2=%02X (exp 40) "
                  "CTRL3=%02X (exp 54) ST_CTRL=%02X (exp 80)",
             IIS3DWB10IS_ID, c2, c3, st);
    if (c2 != 0x40 || c3 != 0x54 || st != ST_CTRL_LPF1_AUTO) {
        ESP_LOGE(TAG, "config readback mismatch — sensor may be wrong part");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}
