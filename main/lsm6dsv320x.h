#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#define LSM6DSV320X_COUNT       4U
#define LSM6DSV320X_ODR_HZ      7680U
#define LSM6DSV320X_FS_G        320U
#define LSM6DSV320X_SENS_MG_LSB 10.417f
#define LSM6DSV320X_FIFO_WORD_BYTES 7U
#define LSM6DSV320X_FIFO_ROWS_PER_READ 256U

/* Pins follow the supplied ESP32-C6-MINI-1 schematic. */
#define LSM6DSV320X_SPI_HOST SPI2_HOST
#define LSM6DSV320X_PIN_SCK  6
#define LSM6DSV320X_PIN_MISO 2
#define LSM6DSV320X_PIN_MOSI 7

esp_err_t lsm6dsv320x_init(void);
uint8_t lsm6dsv320x_rescan(void);
uint8_t lsm6dsv320x_present_mask(void);
uint8_t lsm6dsv320x_whoami(unsigned imu);
int8_t lsm6dsv320x_internal_freq_fine(unsigned imu);
esp_err_t lsm6dsv320x_cs_a_test(unsigned transitions, uint32_t interval_ms);
uint16_t lsm6dsv320x_fifo_level(unsigned imu);
bool lsm6dsv320x_fifo_overrun(unsigned imu);
void lsm6dsv320x_capture_start(void);
void lsm6dsv320x_capture_stop(void);
void lsm6dsv320x_capture_finish(void);
esp_err_t lsm6dsv320x_fifo_read(unsigned imu, uint8_t *dst, uint16_t rows);
