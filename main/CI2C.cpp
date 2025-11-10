#include "CI2C.h"

extern "C" 
{
    #include "driver/i2c_master.h"
    #include <stdio.h>
    #include "esp_err.h"
    #include "esp_log.h"
}

#define I2C_SENSOR_BUS_PORT     1
#define I2C_SENSOR_SDA          gpio_num_t::GPIO_NUM_33
#define I2C_SENSOR_SCL          gpio_num_t::GPIO_NUM_32


//I2C Port Configuration for sensor

//I2C Bus Handler and Configuration for sensor
i2c_master_bus_handle_t i2c_sensor_bus = NULL;    
i2c_master_bus_config_t sensor_bus_config = 
{
    .i2c_port = I2C_SENSOR_BUS_PORT,    
    .sda_io_num = I2C_SENSOR_SDA,
    .scl_io_num = I2C_SENSOR_SCL,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,    
    .flags
    {
        .enable_internal_pullup = true,
    }
};

CI2C::CI2C()
{

}

CI2C::~CI2C()
{

}

void CI2C::init()
{
    ESP_ERROR_CHECK(i2c_new_master_bus(&sensor_bus_config, &i2c_sensor_bus));
}

bool CI2C::transmit(uint8_t address, uint8_t *buffer, uint32_t length)
{    
    i2c_master_dev_handle_t i2c_handle = NULL;
    i2c_device_config_t i2c_config = 
    {
        .dev_addr_length  = I2C_ADDR_BIT_LEN_7,         /*!< Select the address length of the slave device. */
        .device_address  = address,                     /*!< I2C device raw address. (The 7/10 bit address without read/write bit) */
        .scl_speed_hz  = 400000,                        /*!< I2C SCL line frequency. */
        .scl_wait_us = 1000000,                         /*!< Timeout value. (unit: us). Please note this value should not be so small that it can handle stretch/disturbance properly. If 0 is set, that means use the default reg value*/
        .flags = 
        {
            .disable_ack_check = 0                       /*!< Disable ACK check. If this is set false, that means ack check is enabled, the transaction will be stopped and API returns error when nack is detected. */
        }
    };

    i2c_master_bus_add_device(i2c_sensor_bus, &i2c_config, &i2c_handle);       
    bool ret = i2c_master_transmit(i2c_handle, buffer, length, 20) == ESP_OK;
    i2c_master_bus_rm_device(i2c_handle);
    
    return ret;
}

bool CI2C::receive(uint8_t address, uint8_t *buffer, uint32_t length)
{    
    i2c_master_dev_handle_t i2c_handle = NULL;
    i2c_device_config_t i2c_config = 
    {
        .dev_addr_length  = I2C_ADDR_BIT_LEN_7,         /*!< Select the address length of the slave device. */
        .device_address  = address,                     /*!< I2C device raw address. (The 7/10 bit address without read/write bit) */
        .scl_speed_hz  = 400000,                        /*!< I2C SCL line frequency. */
        .scl_wait_us = 1000000,                         /*!< Timeout value. (unit: us). Please note this value should not be so small that it can handle stretch/disturbance properly. If 0 is set, that means use the default reg value*/
        .flags = 
        {
            .disable_ack_check = 0                       /*!< Disable ACK check. If this is set false, that means ack check is enabled, the transaction will be stopped and API returns error when nack is detected. */
        }
    };

    i2c_master_bus_add_device(i2c_sensor_bus, &i2c_config, &i2c_handle);       
    bool ret = i2c_master_receive(i2c_handle, buffer, length, 20) == ESP_OK;
    i2c_master_bus_rm_device(i2c_handle);

    return ret;
}