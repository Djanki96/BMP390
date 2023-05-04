#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO    22          // GPIO22 als SCL
#define I2C_MASTER_SDA_IO    21          // GPIO21 als SDA
#define I2C_MASTER_NUM       I2C_NUM_0   // I2C-Port 0
#define I2C_MASTER_FREQ_HZ   100000      // I2C-Taktfrequenz 100 kHz
#define BMP390_SENSOR_ADDR   0x77       // I2C-Adresse des BMP390
#define BMP390_REGISTER      0x04   

void app_main()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    while (1) {
        uint8_t data[6];
        uint8_t datasend[2];
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (BMP390_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x36, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK) {
            printf("1. Could not read pressure from BMP390 sensor\n");
            vTaskDelay(1000 / portTICK_RATE_MS);
            continue;
        }

        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        //i2c_master_write_byte(cmd, (BMP390_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
        datasend[0] = BMP390_SENSOR_ADDR << 1 | I2C_MASTER_READ;
        datasend[1] = 0x04;
        i2c_master_write(cmd, &datasend[0], 1, true);
        //i2c_master_write_byte(cmd, BMP390_REGISTER, true);
        for (int i = 0; i < 5; i++) {
           i2c_master_read_byte(cmd, &data[i], I2C_MASTER_ACK);
        }
        i2c_master_read_byte(cmd, &data[5], I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK) {
            printf("Error: %d\n", ret);
            vTaskDelay(1000 / portTICK_RATE_MS);
            continue;
        }

        int32_t pressure_raw = (int32_t)(data[1] << 16 | data[2] << 8 | data[2]);
        float pressure = (float)pressure_raw / 4096.0f;
        printf("Pressure: %f hPa\n", pressure);

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}