#ifndef CI2C_H
#define CI2C_H

#pragma once

#include <cstdint>

class CI2C
{
public:
    CI2C();
    ~CI2C();
    void init();
    bool transmit(uint8_t address, uint8_t *buffer, uint32_t length);
    bool receive(uint8_t address, uint8_t *buffer, uint32_t length);

private:

};

#endif