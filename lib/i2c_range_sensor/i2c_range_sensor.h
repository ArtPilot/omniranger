/*
  i2c_range_sensor.h - Library to Controll Range Sensors that are Connected Via TCA9548
  MIT 2021 art-pilot.de
*/

// ensure this library description is only included once
#ifndef i2c_range_sensor_h
#define i2c_range_sensor_h

// TODO: See if this is realy needed
#include <Arduino.h>
#include <tca9548.h>

namespace OmniRanger
{
    constexpr uint8_t sensor_range_commands[2] = {byte(0x52),byte(0x51)}; 
    constexpr uint8_t sensor_range_scalers[2] = {1,10}; 
    constexpr uint16_t sensor_max_range[2] = {2000,7200}; 

    class I2CRangeSensor
    {
    public:
        I2CRangeSensor(uint8_t type, uint8_t channel, uint8_t address = 0);
        bool trigger(void);
        bool read(void);
        bool maxReached(void);
        unsigned int range = 0;
        bool triggered = false;

    private:
        uint8_t _address;
        uint8_t _channel;
        uint8_t _type;
    };

};
#endif