#pragma once

#include <Arduino.h>
#include "NexDome.h"

namespace SHIELDMD10
{
class Motor
{
  public:
    Motor();
    void run(int dir, int pwm);
    void stop();
    void brake();
    bool isRunning();
    int readCurrent();

  private:
    bool _isRunning;
};
}