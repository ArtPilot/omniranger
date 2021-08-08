/*
  tca9548.h - Library to controll a single TCA9548
  MIT 2021 Art-Pilot
*/

// TCA9548a
// Pin 7 GND, Pin 6 5V, PIN 5 SCL, PIN 4 SDA
// TODO: Requires pull‑ups for SCL and SDA connected to 5V to work reliably

#ifndef __tca9548_H__
#define __tca9548_H__

#include "Arduino.h"
#include "Wire.h"

namespace TCA9548Lib
{

  class TCAClass
  {

  private:
    uint8_t _address;
    uint8_t _channel = 10; // The 10 is used when we do not know the current channel the tca switch to

  public:
    // This needs to be called to set up the connection to the TCA9548
    // You can change the default address of the device if you call it
    // with an address as param
    boolean begin(uint8_t address = byte(0x70));
    boolean select_channel(uint8_t channel);
  };

  extern TCAClass TCA9548;

};

using namespace TCA9548Lib;

#endif
