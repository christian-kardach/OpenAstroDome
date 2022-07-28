#include "Encoder.h"

Encoder::Encoder(uint8_t ENCA, uint8_t ENCB)
{
    _ENCA = ENCA;
    _ENCB = ENCB;
    pinMode(_ENCA, INPUT_PULLUP);
    pinMode(_ENCB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_ENCA),updateEncoder,RISING);
}

int32_t Encoder::GetPosition()
{
    int32_t position = 0;
    noInterrupts();  // disable interrupts temporarily while reading
    position = encoderPosition;
    interrupts(); // turn interrupts back on
    return position;
}

void Encoder::updateEncoder(){
  if(digitalRead(_ENCB) > 0){
    encoderPosition++;
  } else{
    encoderPosition--;
  }
}