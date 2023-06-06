#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>

#define I2C_MASTER_NUM    I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_INTR_FLAG_NONE 0

#define I2C_SCL_PIN     22
#define I2C_SDA_PIN     21
#define BMP390_I2C_ADDR 0x77

#define BMP390_REG_PRESS_XLSB 0x04
#define BMP390_REG_TEMP_XLSB 0x07

static const i2c_config_t i2c_config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_SDA_PIN,
    .scl_io_num = I2C_SCL_PIN,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master = {
        .clk_speed = I2C_MASTER_FREQ_HZ,
    },
};

static void i2c_master_init()
{
    i2c_param_config(I2C_MASTER_NUM, &i2c_config);
    i2c_driver_install(I2C_MASTER_NUM, i2c_config.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, I2C_MASTER_INTR_FLAG_NONE);
}

static void bmp390_read_data()
{
    uint8_t data[6];
    uint32_t pressure;
    uint32_t temperature;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (BMP390_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x1B, true);
    i2c_master_write_byte(cmd, 0b00110011, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP390_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP390_REG_PRESS_XLSB, true);
    i2c_master_write_byte(cmd, BMP390_REG_TEMP_XLSB, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP390_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    pressure = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    printf("Druck:  %.0f hPa\n", (float)pressure);

    temperature = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
    printf("Temp:   %.0f °C\n", (float)temperature);
}

void app_main()
{
    i2c_master_init();

    while (1) {
        bmp390_read_data();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}