/**
 * @brief LED
 */
 
// Author: Gustavo Diaz

// Requiered Libraries
#include "LedClass.h"

uint8_t LedClass::getState(void)
{
    return state_;
}

void LedClass::blink(void)
{
    digitalWrite(led_pin_,HIGH);
    delay(500);
    digitalWrite(led_pin_,LOW);
    delay(500);
    digitalWrite(led_pin_,HIGH);
    delay(500);
    digitalWrite(led_pin_,LOW);
    delay(500);
    digitalWrite(led_pin_,HIGH);
    delay(500);
    digitalWrite(led_pin_,LOW);
    state_ = 0;
}

void LedClass::on(void)
{
    digitalWrite(led_pin_,HIGH);
    state_ = 1;
}

void LedClass::off(void)
{
    digitalWrite(led_pin_,LOW);
    state_ = 0;
}