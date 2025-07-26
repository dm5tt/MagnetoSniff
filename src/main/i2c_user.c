#include "i2c_user.h"

#include <string.h>

#include "esp_log.h"

#define I2C_USER_TIMEOUT_MS 1000

esp_err_t i2c_user_init(gpio_num_t sda, gpio_num_t scl, uint32_t clk_hz, i2c_port_t port)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clk_hz};
    esp_err_t err;

    err = i2c_param_config(port, &conf);
    if (err != ESP_OK)
        return err;

    err = i2c_driver_install(port, conf.mode, 0, 0, 0);
    return err;
}

esp_err_t i2c_user_read_reg(i2c_port_t port, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    if (!data || len == 0)
        return ESP_ERR_INVALID_ARG;

    return i2c_master_write_read_device(
        port,
        dev_addr,
        &reg_addr,
        1,
        data,
        len,
        I2C_USER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t i2c_user_write_reg(i2c_port_t port, uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, size_t len)
{
    if (!data || len == 0)
        return ESP_ERR_INVALID_ARG;

    uint8_t buf[len + 1];
    buf[0] = reg_addr;
    memcpy(&buf[1], data, len);

    return i2c_master_write_to_device(
        port,
        dev_addr,
        buf,
        len + 1,
        I2C_USER_TIMEOUT_MS / portTICK_PERIOD_MS);
}
