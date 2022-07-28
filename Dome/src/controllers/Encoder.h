#pragma once

#include <Arduino.h>
#include "NexDome.h"

class Encoder
{
  public:
    Encoder(uint8_t ENCA, uint8_t ENCB);
    int32_t GetPosition();
    static volatile int32_t encoderPosition; // specify encoder as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/  
    static void updateEncoder();
  
  private:
    static uint8_t _ENCA;
    static uint8_t _ENCB;
};
