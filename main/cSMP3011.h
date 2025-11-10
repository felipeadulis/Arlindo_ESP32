#ifndef CSMP3011_H
#define CSMP3011_H

#pragma once

class cSMP3011
{
private:
    float pressure;
    float temperature;

public:
    cSMP3011();
    ~cSMP3011();
    void init();
    void poll();
    float getPressure();
    float getTemperature();

private:

};

#endif