#include "task.h"

#include "esp_log.h"

#include "wifi.h"
#include "signalling.h"
#include "sntp.h"
#include "task.h"
// #include "i2c_test.h"
#include "i2c_user.h"
#include "sender.h"


#include "system_settings.h"

void app_main(void)
{

    i2c_user_init(SYSTEM_SETTINGS_I2C_SDA_PIN, SYSTEM_SETTINGS_I2C_SCL_PIN, SYSTEM_SETTINGS_I2C_FREQ_HZ, SYSTEM_SETTINGS_I2C_PORT);

    signalling_init();

    wifi_init_sta();

    sntp_init_user();

    /*
        esp_err_t ret = i2c_scanner_init(SYSTEM_SETTINGS_I2C_SDA_PIN, SYSTEM_SETTINGS_I2C_SCL_PIN, SYSTEM_SETTINGS_I2C_FREQ_HZ, SYSTEM_SETTINGS_I2C_PORT);

        if (ret != ESP_OK)
        {
            ESP_LOGE("MAIN", "Failed to initialize I2C scanner");
            return;
        }

        i2c_scanner_scan_once();

    */
    sender_init();

    task_example_create();

}