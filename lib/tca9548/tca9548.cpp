/*
  tca9548.h - Library to controll a single TCA9548
  MIT 2021 art-pilot.de
*/

#include "tca9548.h"

namespace TCA9548Lib
{

  boolean TCAClass::begin(uint8_t address)
  {
    _address = address;
    if ((TWCR & _BV(TWEN)) == 0)
    {
      Wire.begin();
    }
    uint8_t error;
    Wire.beginTransmission(_address);
    error = Wire.endTransmission();
    return (error == 0 ? true : false);
  }

  boolean TCAClass::select_channel(uint8_t channel)
  {
    if (channel > 7)
      return false;
    if (channel == _channel)
    {
      return true;
    }
    byte error;
    Wire.beginTransmission(_address);
    Wire.write(1 << channel);
    Wire.endTransmission();
    error = Wire.endTransmission();
    if (error == 0)
    {
      _channel = channel;
      return true;
    }
    else
    {
      // Maybe hacky but in case of an error selecting a channel
      // we set the channel to 10 - so that following commands will try to
      // reset the channel
      _channel = 10;
      return false;
    }
  }

  TCAClass TCA9548;

};
