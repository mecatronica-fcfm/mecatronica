/**
 * @file
 * @brief BeltSorter
 */
#ifndef BeltSorter_h
#define BeltSorter_h

#include "Arduino.h"
#include <Servo.h>

#define BELTSORTER_ECHO_PIN       7 // Echo Pin
#define BELTSORTER_TRIGGER_PIN    4 // Trigger Pin
#define BELTSORTER_LED_PIN        13 // Onboard LED

#define BELTSORTER_ENCODER_PIN    2 // Onboard LED

#define BELTSORTER_PUSHER1_PIN     5 
#define BELTSORTER_PUSHER2_PIN     3     

#define BELTSORTER_ANALOG_PIN     0 
#define BELTSORTER_BELT_PIN       6

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
  void spin();
  // Pusher functions
  void openPusherA();
  void openPusherB();
  void closePusherA();
  void closePusherB();

  float getHeight();
  float getBeltDistance();
  float getBeltSpeed();
  void waitForPeriod();
  void setPeriod(unsigned long period_us);
  void begin();

private:
  Pusher _pusherA;
  Pusher _pusherB;
  const int _max_range = 1040; // Maximum range needed
  const int _min_range = 500; // Minimum range needed
  long beltsorter_duration;
  float beltsorter_distance; // Duration used to calculate distance
  unsigned long beltsorter_belt_period;
  unsigned long beltsorter_last_time;
  unsigned long beltsorter_period;
  unsigned long beltsorter_last_period;
  const float _encoder_constant;
  float beltsorter_belt_speed;
  float beltsorter_belt_last_distance;

};

#endif
