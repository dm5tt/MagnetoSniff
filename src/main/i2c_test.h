#ifndef I2C_SCANNER_H
#define I2C_SCANNER_H

#include <stdbool.h>
#include "esp_err.h"

esp_err_t i2c_scanner_init(int sda_pin, int scl_pin, uint32_t freq_hz, int i2c_num);
esp_err_t i2c_scanner_start_task(uint32_t scan_interval_ms, int task_priority, uint32_t stack_size);
esp_err_t i2c_scanner_stop_task(void);
esp_err_t i2c_scanner_scan_once(void);

#endif /* I2C_SCANNER_H */