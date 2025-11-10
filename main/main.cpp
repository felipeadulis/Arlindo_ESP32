extern "C" 
{
    #include "driver/i2c_master.h"
    #include "driver/gpio.h"
    #include <stdio.h>
    #include "esp_err.h"
    #include "esp_log.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"    
    #include "esp_lvgl_port.h"
    #include "lvgl.h"
    #include "displaySSD1306.h"    
}

#include "cSMP3011.h"
#include "CBMP280.h"
#include "CGlobalResources.h"

#define LED_PIN     gpio_num_t::GPIO_NUM_16  

/*
    PROTOTYPES
*/
void sensorSMP3011Task(void *pvParameters);
void sensorBMP280Task(void *pvParameters);

void statusLedTask(void *pvParameters);

/*
    VARIABLES
*/
cSMP3011    SMP3011;
CBMP280     BMP280;
SemaphoreHandle_t sensorMutex;

/**
 * @brief Entry point of the application.
 *
 * This function configures the I2C master mode and scans the bus for devices.
 * The bus is configured to use GPIO 5 for SDA and GPIO 4 for SCL, and the
 * clock speed is set to 100000 Hz. The scan starts from address 1 and goes
 * to address 126 (inclusive). If a device is found at an address, a message
 * is printed to the console with the address of the device.
 */
extern "C" void app_main() 
{
    //------------------------------------------------
    // Status LED
    //------------------------------------------------
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    xTaskCreate(statusLedTask, "statusLedTask", 4096, NULL, 1, NULL);

    //------------------------------------------------
    // I2C Initialization
    //------------------------------------------------    
    I2C.init();

    //------------------------------------------------
    // SMP3011 Initialization
    //------------------------------------------------ 
    SMP3011.init();
    BMP280.init();
    sensorMutex = xSemaphoreCreateMutex();
    xTaskCreate(sensorSMP3011Task, "sensorSMP3011Task", 4096, NULL, 1, NULL);
    xTaskCreate(sensorBMP280Task, "sensorBMP280Task", 4096, NULL, 1, NULL);


    //------------------------------------------------
    // LVGL
    //------------------------------------------------
    displayInit();
    
    //------------------------------------------------
    // Create a Label
    //------------------------------------------------
    lvgl_port_lock(portMAX_DELAY);
    lv_obj_t *scr = lv_disp_get_scr_act(NULL);
        
    lv_obj_t *lblPressure = lv_label_create(scr);    
    lv_label_set_text_fmt(lblPressure, "P: %5.1f", SMP3011.getPressure());    
    lv_obj_set_width(lblPressure, LCD_H_RES);
    lv_obj_align(lblPressure, LV_ALIGN_TOP_MID, 0, 0);    
    lv_obj_set_y(lblPressure, 0);

    lv_obj_t *lblTemperature = lv_label_create(scr);    
    lv_label_set_text_fmt(lblTemperature, "T: %3.0f", SMP3011.getTemperature());    
    lv_obj_set_width(lblTemperature, LCD_H_RES);
    lv_obj_align(lblTemperature, LV_ALIGN_TOP_MID, 0, 0);    
    lv_obj_set_y(lblTemperature, 16);


    lv_obj_t *lblPressure2 = lv_label_create(scr);    
    lv_label_set_text_fmt(lblPressure2, "P: %5.1f", BMP280.getPressure());    
    lv_obj_set_width(lblPressure2, LCD_H_RES);
    lv_obj_align(lblPressure2, LV_ALIGN_TOP_MID, 0, 0);    
    lv_obj_set_y(lblPressure2, 32);

    lv_obj_t *lblTemperature2 = lv_label_create(scr);    
    lv_label_set_text_fmt(lblTemperature2, "T: %3.0f", BMP280.getTemperature());    
    lv_obj_set_width(lblTemperature2, LCD_H_RES);
    lv_obj_align(lblTemperature2, LV_ALIGN_TOP_MID, 0, 0);    
    lv_obj_set_y(lblTemperature2, 48);    

    lvgl_port_unlock();

    while(1)
    {
        lvgl_port_lock(portMAX_DELAY);        
        lv_label_set_text_fmt(lblPressure     , "P1: %5.1f kPa", SMP3011.getPressure());    
        lv_label_set_text_fmt(lblTemperature  , "T1: %3.0f oC" , SMP3011.getTemperature());    
        lv_label_set_text_fmt(lblPressure2    , "P2: %5.1f kPa", BMP280.getPressure());    
        lv_label_set_text_fmt(lblTemperature2 , "T2: %3.0f oC" , BMP280.getTemperature());  
        lvgl_port_unlock();
        vTaskDelay(100/portTICK_PERIOD_MS);
    }    
}

void sensorSMP3011Task(void *pvParameters) 
{
    while(1)
    {
        //xSemaphoreTake( sensorMutex, portMAX_DELAY );
        SMP3011.poll(); 
        //xSemaphoreGive( sensorMutex );
        vTaskDelay(1/portTICK_PERIOD_MS);
    }
}

void sensorBMP280Task(void *pvParameters) 
{
    while(1)
    {
        //xSemaphoreTake( sensorMutex, portMAX_DELAY );
        BMP280.poll();
        //xSemaphoreGive( sensorMutex );
        vTaskDelay(1/portTICK_PERIOD_MS);
    }
}

void statusLedTask(void *pvParameters) 
{
    while(1)
    {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(250/portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(250/portTICK_PERIOD_MS);
    }
}   