/**
 * @file
 * @brief BeltSorter
 */
#ifndef BeltSorter_h
#define BeltSorter_h

#include "Arduino.h"
#include <Servo.h>

#define BELTSORTER_ECHO_PIN     7  // Echo Pin
#define BELTSORTER_TRIGGER_PIN  4  // Trigger Pin
#define BELTSORTER_LED_PIN      13 // Onboard LED

#define BELTSORTER_ENCODER_PIN  2  // Onboard LED

#define BELTSORTER_PUSHER1_PIN  5 
#define BELTSORTER_PUSHER2_PIN  3     

#define BELTSORTER_ANALOG_PIN   0 
#define BELTSORTER_BELT_PIN     6

class Pusher
{
public:
  Pusher(uint8_t open_pos, uint8_t close_pos);
  void attach(int pin);
  void open();
  void close();
private:
  const uint8_t _open_pos;
  const uint8_t _close_pos;
  Servo _servo;
};


class BeltSorter
{

public:
  BeltSorter();
  void begin();
  void loop();
  
  // Pusher functions
  void openPusherA();
  void openPusherB();
  void closePusherA();
  void closePusherB();

  float getHeight();
  float getBeltDistance();
  float getBeltSpeed();
  void sleep();
  void setPeriod(unsigned long period_us);
  

private:
  uint32_t _urDuration;
  uint32_t _encoderPeriod;
  uint32_t _beltLastTime;
  uint32_t _period;
  uint32_t _lastPeriod;
  float _urHeight;
  float _beltSpeed;
  float _beltLastDistance;
  
  Pusher _pusherA;
  Pusher _pusherB;

  const uint16_t _urMaxRange = 1040U; // Maximum range needed
  const uint16_t _urMinRange = 500U; // Minimum range needed
  const float _encoderConstant = 550.0f;

};

#endif
