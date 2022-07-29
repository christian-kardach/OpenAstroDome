#include "SHIELDMD10Controller.h"

namespace SHIELDMD10
{
Motor::Motor()
{
    _isRunning = false;    
    pinMode(MOTOR_DIRECTION_PIN, OUTPUT);
    pinMode(MOTOR_PWM_PIN, OUTPUT);
}

void Motor::run(int dir, int pwm)  // dir is 0 or 1
{
    digitalWrite(MOTOR_DIRECTION_PIN, dir);
    analogWrite(MOTOR_PWM_PIN, pwm);
    _isRunning = true;
}
void Motor::stop()
{
    analogWrite(MOTOR_PWM_PIN, LOW);
    _isRunning = false;
}

void Motor::brake()
{
    analogWrite(MOTOR_PWM_PIN, LOW);
    _isRunning = false;
}

bool Motor::isRunning()
{
    return _isRunning;
}

int Motor::readCurrent()
{
    return 0;
}
}  // namespace SHIELDMD10