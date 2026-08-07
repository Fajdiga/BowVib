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

/* INT1 → capture engine plumbing */
static iis3dwb10is_int1_cb_t s_int1_cb;
static void                  *s_int1_arg;

/* Internal DMA-capable scratch for large FIFO bulk reads (1 + 4000 bytes). */
#define ROWS_PER_XFER 400
static uint8_t DRAM_ATTR s_tx[ROWS_PER_XFER * FIFO_ROW_BYTES + 1];
static uint8_t DRAM_ATTR s_rx[ROWS_PER_XFER * FIFO_ROW_BYTES + 1];

float iis3dwb10is_fs_to_mglsb(iis3dwb10is_fs_t fs)
{
    switch (fs) {
    case IIS3DWB10IS_FS_50G:  return 0.095f;
    case IIS3DWB10IS_FS_100G: return 0.191f;
    default:                  return 0.381f;   /* ±200 g */
    }
}

uint8_t iis3dwb10is_read(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0 };
    uint8_t rx[2] = { 0 };
    spi_transaction_t t = {
        .length = 16, .tx_buffer = tx, .rx_buffer = rx,
    };
    spi_device_polling_transmit(s_spi, &t);
    return rx[1];
}

void iis3dwb10is_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg, val };
    spi_transaction_t t = {
        .length = 16, .tx_buffer = tx, .rx_buffer = NULL,
    };
    spi_device_polling_transmit(s_spi, &t);
}

void iis3dwb10is_read_burst(uint8_t reg, uint8_t *dst, uint16_t len)
{
    uint8_t tx[32] = { 0 };
    uint8_t rx[32] = { 0 };
    tx[0] = (uint8_t)(reg | 0x80);
    spi_transaction_t t = {
        .length = (uint32_t)(1 + len) * 8, .tx_buffer = tx, .rx_buffer = rx,
    };
    spi_device_polling_transmit(s_spi, &t);
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
    return (uint16_t)b[0] | (((uint16_t)(b[1] & 0x0F)) << 8);
}

bool iis3dwb10is_fifo_overrun(void)
{
    return (iis3dwb10is_read(REG_FIFO_STATUS2) & 0x40) != 0;
}

void iis3dwb10is_fifo_clear_overrun(void)
{
    iis3dwb10is_read(REG_FIFO_STATUS2);
}

void iis3dwb10is_fifo_read(uint8_t *dst, uint16_t nrows)
{
    uint16_t off = 0;
    s_tx[0] = (uint8_t)(REG_FIFO_OUT | 0x80);
    while (nrows) {
        uint16_t n = (nrows > ROWS_PER_XFER) ? ROWS_PER_XFER : nrows;
        uint16_t len = n * FIFO_ROW_BYTES;
        spi_transaction_t t = { 0 };
        t.length = (uint32_t)(1 + len) * 8;
        t.tx_buffer = s_tx;
        t.rx_buffer = s_rx;
        spi_device_polling_transmit(s_spi, &t);
        memcpy(dst + off, s_rx + 1, len);   /* dst may be PSRAM (CPU copy; DMA can't reach it) */
        off += len;
        nrows -= n;
    }
}

void iis3dwb10is_start(iis3dwb10is_odr_t odr, iis3dwb10is_fs_t fs)
{
    iis3dwb10is_write(REG_CTRL1, IIS3DWB10IS_ODR_IDLE);       /* pause */
    iis3dwb10is_write(REG_CTRL2, (uint8_t)(fs << 5));         /* full-scale */
    iis3dwb10is_write(REG_FIFO_CTRL3, 0x00);                  /* bypass = flush FIFO */
    esp_rom_delay_us(50);
    iis3dwb10is_fifo_clear_overrun();
    iis3dwb10is_write(REG_CTRL1, (uint8_t)odr);               /* continuous @ odr */
    iis3dwb10is_write(REG_FIFO_CTRL3, 0x02 | 0x08);           /* STREAM + XL batch */
}

void iis3dwb10is_stop(void)
{
    iis3dwb10is_write(REG_CTRL1, IIS3DWB10IS_ODR_IDLE);
    iis3dwb10is_write(REG_FIFO_CTRL3, 0x00);                  /* bypass/flush */
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
    memset(s_tx, 0, sizeof(s_tx));

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

    /* INT1: input, pull-down, rising edge = FIFO watermark. */
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

    /* Soft reset, then idle config. */
    iis3dwb10is_write(REG_CTRL3, CTRL3_SW_RESET);
    vTaskDelay(pdMS_TO_TICKS(5));
    iis3dwb10is_write(REG_CTRL3, CTRL3_BDU | CTRL3_FIFO_EN | CTRL3_IF_INC);   /* 0x54 */
    iis3dwb10is_write(REG_CTRL4, CTRL4_AXES);                                 /* 0x1C */
    iis3dwb10is_write(REG_CTRL2, (uint8_t)(IIS3DWB10IS_FS_200G << 5));        /* ±200 g */
    iis3dwb10is_write(REG_CTRL1, IIS3DWB10IS_ODR_IDLE);                       /* idle */
    iis3dwb10is_write(REG_FIFO_CTRL1, (uint8_t)(FIFO_WATERMARK & 0xFF));
    iis3dwb10is_write(REG_FIFO_CTRL2, (uint8_t)((FIFO_WATERMARK >> 8) & 0x0F));
    iis3dwb10is_write(REG_INT_CTRL2, INT1_FIFO_TH);                           /* wtm → INT1 */
    iis3dwb10is_write(REG_FIFO_CTRL3, 0x00);                                  /* bypass */

    /* Readback self-check: catches a wrong write immediately at boot. */
    uint8_t c2 = iis3dwb10is_read(REG_CTRL2);
    uint8_t c3 = iis3dwb10is_read(REG_CTRL3);
    ESP_LOGI(TAG, "ready. WHO_AM_I=0x%02X  CTRL2=%02X (exp 40)  CTRL3=%02X (exp 54)",
             IIS3DWB10IS_ID, c2, c3);
    if (c2 != 0x40 || c3 != 0x54) {
        ESP_LOGE(TAG, "config readback mismatch — sensor may be wrong part");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}
