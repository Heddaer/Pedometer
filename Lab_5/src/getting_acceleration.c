#include <stdio.h>
#include <stdlib.h>
#include <driver/i2c.h>
#include <math.h>

#include "getting_acceleration.h"

#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_ADDR 0x68

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define sdapin 33
#define sclpin 32

void initI2C()
{
    // configure and install driver
    i2c_config_t conf;
    // ESP32 acts as master
    conf.mode = I2C_MODE_MASTER;
    // pin used for SDA
    conf.sda_io_num = sdapin; 
    // The SDA and SCL lines are active low, so they should be pulled up with resistors 
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE; 
    // SCL pin number
    conf.scl_io_num = sclpin; 
    // enables pullup on SDA  
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE; 
    // Standard mode (100 Kbit/s)
    conf.master.clk_speed = 100000;
    conf.clk_flags = 0; 
    // configure I2C controller 0
    esp_err_t res = i2c_param_config(I2C_NUM_0, &conf); 
    ESP_ERROR_CHECK(res);
    // install driver, no buffers needed in master mode nor special interrupts config 
    res = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0); 
    ESP_ERROR_CHECK(res);

    // configure power mode, here we set all bits of the PWR_MGMT_1 register to 0
    // create command
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // start command
    res = i2c_master_start(cmd);
    ESP_ERROR_CHECK(res);
    // set address + write and check for ack
    res = i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, 1); 
    ESP_ERROR_CHECK(res);
    // write the register address and check for ack
    res = i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, 1); 
    ESP_ERROR_CHECK(res);
    // write value of the regiter: 0, and check for ack
    res = i2c_master_write_byte(cmd, 0x00, 1); 
    ESP_ERROR_CHECK(res);
    // end of command
    res = i2c_master_stop(cmd); ESP_ERROR_CHECK(res);
    // send the command, 1 second timeout
    res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS); 
    ESP_ERROR_CHECK(res);
    // delete command now that it's not needed
    i2c_cmd_link_delete(cmd);

    // set the sampling frequency
    // the sampling freq is gyro sampling freq / (1 + divider) 
    // setting divider to 800 leads to sampling freq. of 10 Hz
    cmd = i2c_cmd_link_create();
    res = i2c_master_start(cmd);
    ESP_ERROR_CHECK(res);
    res = i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, 1); // WRITE bit set! 
    ESP_ERROR_CHECK(res);
    res = i2c_master_write_byte(cmd, MPU6050_SMPLRT_DIV, 1); // write to SMPLRT_DIV
    ESP_ERROR_CHECK(res);
    res = i2c_master_write_byte(cmd, 799, 1); // set SMPLRT_DIV to 800 IF NOT working try lower divider 
    ESP_ERROR_CHECK(res);
    res = i2c_master_stop(cmd);
    ESP_ERROR_CHECK(res);
    res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS); 
    // ESP_ERROR_CHECK(res);
    i2c_cmd_link_delete(cmd);
}

void writeI2C(uint8_t address, uint8_t reg, uint8_t data)
{
    esp_err_t res;
    // create command
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // start command
    res = i2c_master_start(cmd);
    ESP_ERROR_CHECK(res);
    // set address + write and check for ack
    res = i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_WRITE, 1);
    ESP_ERROR_CHECK(res);
    // write the register address and check for ack
    res = i2c_master_write_byte(cmd, reg, 1);
    ESP_ERROR_CHECK(res);
    // write value of the regiter: 0, and check for ack
    res = i2c_master_write_byte(cmd, data, 1);
    ESP_ERROR_CHECK(res);
    // end of command
    res = i2c_master_stop(cmd);
    ESP_ERROR_CHECK(res);
    // send the command, 1 second timeout
    res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS); 
    ESP_ERROR_CHECK(res);
    // delete command now that it's not needed
    i2c_cmd_link_delete(cmd);
}

int readI2C(uint8_t regLow, uint8_t regHigh)
{
    // to store the answer
    uint8_t buffer;
    //temporary storage
    int16_t temp = 0;
    esp_err_t res;

    // create command
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // start command
    res = i2c_master_start(cmd);
    ESP_ERROR_CHECK(res);

    // First read register of low
    cmd = i2c_cmd_link_create(); 
    res = i2c_master_start(cmd); 
    ESP_ERROR_CHECK(res);
    // WRITE bit set on address!
    res = i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, 1);  
    ESP_ERROR_CHECK(res);
    res = i2c_master_write_byte(cmd, regLow, 1); 
    ESP_ERROR_CHECK(res);
    res = i2c_master_stop(cmd);
    ESP_ERROR_CHECK(res);
    res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS); 
    ESP_ERROR_CHECK(res);
    i2c_cmd_link_delete(cmd);

    // wait a little
    vTaskDelay(10 / portTICK_RATE_MS);

    // Read answer of low 
    cmd = i2c_cmd_link_create();
    res = i2c_master_start(cmd);
    ESP_ERROR_CHECK(res);
    res = i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_READ, 1); 
    ESP_ERROR_CHECK(res);
    res = i2c_master_read(cmd, &buffer, 1, I2C_MASTER_NACK); 
    ESP_ERROR_CHECK(res);
    res = i2c_master_stop(cmd);
    ESP_ERROR_CHECK(res);
    res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS); 
    ESP_ERROR_CHECK(res);
    i2c_cmd_link_delete(cmd);

    temp = buffer;

    // read register of high
    cmd = i2c_cmd_link_create(); 
    res = i2c_master_start(cmd); 
    ESP_ERROR_CHECK(res);
    // WRITE bit set on address!
    res = i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, 1);  
    ESP_ERROR_CHECK(res);
    // read low first on register
    res = i2c_master_write_byte(cmd, regHigh, 1); 
    ESP_ERROR_CHECK(res);
    res = i2c_master_stop(cmd);
    ESP_ERROR_CHECK(res);
    res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS); 
    ESP_ERROR_CHECK(res);
    i2c_cmd_link_delete(cmd);

    // wait a little
    vTaskDelay(10 / portTICK_RATE_MS);

    // Read answer of high 
    cmd = i2c_cmd_link_create();
    res = i2c_master_start(cmd);
    ESP_ERROR_CHECK(res);
    res = i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_READ, 1); 
    ESP_ERROR_CHECK(res);
    res = i2c_master_read(cmd, &buffer, 1, I2C_MASTER_NACK); 
    ESP_ERROR_CHECK(res);
    res = i2c_master_stop(cmd);
    ESP_ERROR_CHECK(res);
    res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS); 
    ESP_ERROR_CHECK(res);
    i2c_cmd_link_delete(cmd);

    // combine high and low registers into a signed integer
    temp |= ((int16_t)buffer) << 8;

    return temp;
}

double calculatedAcceleration()
{
    double x, y, z;

    x = (double) readI2C(ACCEL_XOUT_L, ACCEL_XOUT_H);
    y = (double) readI2C(ACCEL_YOUT_L, ACCEL_YOUT_H);
    z = (double) readI2C(ACCEL_ZOUT_L, ACCEL_ZOUT_H);

    double acceleration = sqrt(pow(x,2)+pow(y,2)+pow(z,2));

    return acceleration;
}
