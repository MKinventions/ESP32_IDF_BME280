#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

#define I2C_MASTER_SCL_IO           22    /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO           21    /*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// BME280 I2C address
#define BME280_ADDR                 0x76

// BME280 Registers
#define BME280_REG_ID               0xD0
#define BME280_REG_RESET            0xE0
#define BME280_REG_CTRL_HUM         0xF2
#define BME280_REG_CTRL_MEAS        0xF4
#define BME280_REG_CONFIG           0xF5
#define BME280_REG_TEMP_MSB         0xFA
#define BME280_REG_HUM_MSB          0xFD

static const char *TAG = "BME280";

// Calibration data structure
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_calib_data_t;

bme280_calib_data_t calib_data;
int32_t t_fine;

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t bme280_write(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t bme280_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd); // repeated start
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Compensation formulas
int32_t bme280_compensate_T(int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) * ((int32_t)calib_data.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}


uint32_t bme280_compensate_H(int32_t adc_H) {
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib_data.dig_H4) << 20) - (((int32_t)calib_data.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                   (((((((v_x1_u32r * ((int32_t)calib_data.dig_H6)) >> 10) *
                   (((v_x1_u32r * ((int32_t)calib_data.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                   ((int32_t)2097152)) * ((int32_t)calib_data.dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib_data.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t)(v_x1_u32r >> 12);  // or return as float for percentage
}


static void bme280_read_calibration_data(void)
{
    uint8_t calib[26];
    bme280_read(0x88, calib, 26);

    calib_data.dig_T1 = (calib[1] << 8) | calib[0];
    calib_data.dig_T2 = (calib[3] << 8) | calib[2];
    calib_data.dig_T3 = (calib[5] << 8) | calib[4];

    uint8_t hcalib[7];
    bme280_read(0xA1, &calib_data.dig_H1, 1);
    bme280_read(0xE1, hcalib, 7);
    calib_data.dig_H2 = (hcalib[1] << 8) | hcalib[0];
    calib_data.dig_H3 = hcalib[2];
    calib_data.dig_H4 = (hcalib[3] << 4) | (hcalib[4] & 0x0F);
    calib_data.dig_H5 = (hcalib[5] << 4) | ((hcalib[4] >> 4) & 0x0F);
    calib_data.dig_H6 = hcalib[6];
}

static void bme280_init(void)
{
    // Reset BME280
    bme280_write(BME280_REG_RESET, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Read calibration data
    bme280_read_calibration_data();

    // Humidity oversampling x1
    bme280_write(BME280_REG_CTRL_HUM, 0x01);

    // Set normal mode, temperature and pressure oversampling x1
    bme280_write(BME280_REG_CTRL_MEAS, 0x27);

    // Set configuration (filter and standby time)
    bme280_write(BME280_REG_CONFIG, 0xA0);
}

static void bme280_read_data(void)
{
    uint8_t data[5];
//    bme280_read(BME280_REG_TEMP_MSB, data, 5);
//    int32_t adc_T = (int32_t)((((uint32_t)(data[0]) << 12) | ((uint32_t)(data[1]) << 4) | ((data[2]) >> 4)));
//    int32_t adc_H = (int32_t)((((uint16_t)(data[3]) << 8) | (uint16_t)data[4]));


    bme280_read(BME280_REG_HUM_MSB, data, 5);
    int32_t adc_H = (int32_t)((((uint16_t)(data[0]) << 8) | (uint16_t)data[1]));
    int32_t adc_T = (int32_t)((((uint32_t)(data[2]) << 12) | ((uint32_t)(data[3]) << 4) | ((data[4]) >> 4)));



    int32_t temp = bme280_compensate_T(adc_T);
    uint32_t hum = bme280_compensate_H(adc_H);

    ESP_LOGI(TAG, "Temperature: %.2fdegC, Humidity: %.2f%%", temp / 100.0, hum / 1024.0);


}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    bme280_init();

    while (1)
    {
        bme280_read_data();
        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay between readings
    }
}
