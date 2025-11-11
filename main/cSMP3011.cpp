extern "C" 
{
    #include <stdio.h>
    #include "esp_err.h"
    #include "esp_log.h"
}

#include "cSMP3011.h"
#include "CGlobalResources.h"


#define SMP3011_ADDRESS 0x78

cSMP3011::cSMP3011()
{
    pressure = 0;
    temperature = 0;
}

cSMP3011::~cSMP3011()
{

}

void cSMP3011::init()
{
    uint8_t PressSensorCommand = 0xAC;  //Comando para iniciar conversor ADC
    I2C.transmit(SMP3011_ADDRESS, (uint8_t *)(&PressSensorCommand), 1); 
}

void cSMP3011::poll()
{
    uint8_t PressSensorBuffer[6];
    if( !I2C.receive(SMP3011_ADDRESS, (uint8_t *)(&PressSensorBuffer), sizeof(PressSensorBuffer)))
    {
        return;
    }

    if((PressSensorBuffer[0]&0x20) == 0)   //Bit5 do status está em 0 significa que a conversão está pronta
    {              
        //printf("Raw Data: %02X %02X %02X %02X %02X %02X\n", PressSensorBuffer[0], PressSensorBuffer[1], PressSensorBuffer[2], PressSensorBuffer[3], PressSensorBuffer[4], PressSensorBuffer[5]);
        
        uint8_t PressSensorCommand = 0xAC;  //Comando para iniciar conversor ADC
        I2C.transmit(SMP3011_ADDRESS, (uint8_t *)(&PressSensorCommand), 1); 

        float pressurePercentage = (((uint32_t)PressSensorBuffer[1]<<16)|((uint32_t)PressSensorBuffer[2]<<8)|((uint32_t)PressSensorBuffer[3]));        
        pressurePercentage = (pressurePercentage / 16777215.0f);        
        pressurePercentage -= 0.15f;
        pressurePercentage /= 0.7f;
        pressurePercentage *= 500000.0f;

        float temperaturePercentage = (((uint32_t)PressSensorBuffer[4]<<8)|((uint32_t)PressSensorBuffer[5]));
        temperaturePercentage /= 65535.0f;        
        temperaturePercentage = ((150.0f - (-40.0f))*temperaturePercentage) - 40.0f;

        //printf("Pressure: %f  Temperature: %f \n", pressurePercentage, temperaturePercentage);
        
        // Pressao em KPa: pressure = pressurePercentage/1000.0f;
        pressure = pressurePercentage * 0.0001450377f;
        pressure -= 0.05f; //remove erro de offset
        temperature = temperaturePercentage;
    }
}

float cSMP3011::getPressure()
{
    return pressure;
}

float cSMP3011::getTemperature()
{
    return temperature;
}