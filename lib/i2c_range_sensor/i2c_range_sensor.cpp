/*
  i2c_range_sensor.h - Library to Controll Range Sensors that are Connected Via TCA9548
  MIT 2021 art-pilot.de
*/

#include <Arduino.h>
#include <Wire.h>
#include <i2c_range_sensor.h>

// GY-US42v2
#define SRF_I2C_ADDRESS byte(0x70) // 112 DEC in 7bit, 224 DEC in 8bit

// TOF10120
#define LRF_I2C_ADDRESS byte(0x52) // 82 DEC in 7bit, 164 DEC in 8bit

namespace OmniRanger
{
    I2CRangeSensor::I2CRangeSensor(uint8_t type, uint8_t channel, uint8_t address = 0)
    {
        // TODO: Renovate this to make it easier to configure for the user
        // and easier for developers to add new sensor types
        // also it should use less ram redundant information should not be
        // hold per object
        if (address != 0)
        {
            _address = address;
        }
        else if (type == 0)
        {
            _address = LRF_I2C_ADDRESS;
        }
        else if (type == 1)
        {
            _address = SRF_I2C_ADDRESS;
        }
        _channel = channel;
        _type = type;
        triggered = false;

        // This should only run when the I2C Interface
        // has not been initialized before yet
        if ((TWCR & _BV(TWEN)) == 0)
        {
            Wire.begin();
        }
    }

    bool I2CRangeSensor::trigger(void)
    {
        if (!TCA9548.select_channel(_channel))
            return false;
        byte error;
        Wire.beginTransmission(_address);
        Wire.write(sensor_range_commands[_type]); //send range command
        error = Wire.endTransmission();
        if (error == 0)
        {
            triggered = true; // TODO: check if there has been an error and only then set to triggered but then don't try to Trigger this sensor again
        }
        else
        {
            triggered = false; // TODO: check if there has been an error and only then set to triggered but then don't try to Trigger this sensor again
        }
        return triggered;
    };

    // TODO: TEST THIS
    bool I2CRangeSensor::maxReached(void)
    {
        return (sensor_max_range[_type] == range ? true : false);
    }

    bool I2CRangeSensor::read()
    {
        if (triggered == false)
            return false;
        triggered = false;
        if (!TCA9548.select_channel(_channel))
            return false;
        Wire.requestFrom(_address, byte(2));
        if (Wire.available() >= 2)
        {                                   //Sensor responded with the two bytes
            uint8_t HighByte = Wire.read(); //Read the high byte back
            uint8_t LowByte = Wire.read();  //Read the low byte back
            range = ((HighByte * 256) + LowByte) * sensor_range_scalers[_type];
            return true;
        }
        else
        {
            return false;
        }
    }
}
