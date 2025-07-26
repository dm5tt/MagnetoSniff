#ifndef I2C_USER_H
#define I2C_USER_H

#include "driver/i2c.h"
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

esp_err_t i2c_user_init(gpio_num_t sda, gpio_num_t scl, uint32_t clk_hz, i2c_port_t port);
esp_err_t i2c_user_read_reg(i2c_port_t port, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t i2c_user_write_reg(i2c_port_t port, uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, size_t len);

#endif /* I2C_USER_H */
