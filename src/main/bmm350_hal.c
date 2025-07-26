
#include "bmm350_hal.h"

#include "esp_log.h"
#include "esp_rom_sys.h"

#include "task.h"
#include "signalling.h"
#include "i2c_user.h"
#include "system_settings.h"

static const char *TAG = "BMM350_HAL";

static uint8_t dev_addr;

BMM350_INTF_RET_TYPE bmm350_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t *)intf_ptr;

    (void)intf_ptr;

    return i2c_user_read_reg(SYSTEM_SETTINGS_I2C_PORT, device_addr, reg_addr, reg_data, length);
}

BMM350_INTF_RET_TYPE bmm350_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t *)intf_ptr;

    (void)intf_ptr;

    return i2c_user_write_reg(SYSTEM_SETTINGS_I2C_PORT, device_addr, reg_addr, reg_data, length);
}

void bmm350_delay(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;
    vTaskDelay(period_us/1000);
}

void bmm350_error_codes_print_result(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
    case BMM350_OK:
        break;

    case BMM350_E_NULL_PTR:
        ESP_LOGE(TAG, "%s Error [%d] : Null pointer", api_name, rslt);
        break;
    case BMM350_E_COM_FAIL:
        ESP_LOGE(TAG, "%s Error [%d] : Communication fail", api_name, rslt);
        break;
    case BMM350_E_DEV_NOT_FOUND:
        ESP_LOGE(TAG, "%s Error [%d] : Device not found", api_name, rslt);
        break;
    case BMM350_E_INVALID_CONFIG:
        ESP_LOGE(TAG, "%s Error [%d] : Invalid configuration", api_name, rslt);
        break;
    case BMM350_E_BAD_PAD_DRIVE:
        ESP_LOGE(TAG, "%s Error [%d] : Bad pad drive", api_name, rslt);
        break;
    case BMM350_E_RESET_UNFINISHED:
        ESP_LOGE(TAG, "%s Error [%d] : Reset unfinished", api_name, rslt);
        break;
    case BMM350_E_INVALID_INPUT:
        ESP_LOGE(TAG, "%s Error [%d] : Invalid input", api_name, rslt);
        break;
    case BMM350_E_SELF_TEST_INVALID_AXIS:
        ESP_LOGE(TAG, "%s Error [%d] : Self-test invalid axis selection", api_name, rslt);
        break;
    case BMM350_E_OTP_BOOT:
        ESP_LOGE(TAG, "%s Error [%d] : OTP boot", api_name, rslt);
        break;
    case BMM350_E_OTP_PAGE_RD:
        ESP_LOGE(TAG, "%s Error [%d] : OTP page read", api_name, rslt);
        break;
    case BMM350_E_OTP_PAGE_PRG:
        ESP_LOGE(TAG, "%s Error [%d] : OTP page prog", api_name, rslt);
        break;
    case BMM350_E_OTP_SIGN:
        ESP_LOGE(TAG, "%s Error [%d] : OTP sign", api_name, rslt);
        break;
    case BMM350_E_OTP_INV_CMD:
        ESP_LOGE(TAG, "%s Error [%d] : OTP invalid command", api_name, rslt);
        break;
    case BMM350_E_OTP_UNDEFINED:
        ESP_LOGE(TAG, "%s Error [%d] : OTP undefined", api_name, rslt);
        break;
    case BMM350_E_ALL_AXIS_DISABLED:
        ESP_LOGE(TAG, "%s Error [%d] : All axis are disabled", api_name, rslt);
        break;
    case BMM350_E_PMU_CMD_VALUE:
        ESP_LOGE(TAG, "%s Error [%d] : Unexpected PMU CMD value", api_name, rslt);
        break;
    default:
        ESP_LOGE(TAG, "%s Error [%d] : Unknown error code", api_name, rslt);
        break;
    }
}

/*!
 *  @brief Function to select the interface.
 */
int8_t bmm350_interface_init(struct bmm350_dev *dev)
{
    int8_t rslt = BMM350_OK;

    if (dev != NULL)
    {

        dev_addr = BMM350_I2C_ADSEL_SET_LOW;
        dev->intf_ptr = &dev_addr;
        dev->read = bmm350_i2c_read;
        dev->write = bmm350_i2c_write;
        dev->delay_us = bmm350_delay;
    }
    else
    {
        rslt = BMM350_E_NULL_PTR;
    }

    return rslt;
}