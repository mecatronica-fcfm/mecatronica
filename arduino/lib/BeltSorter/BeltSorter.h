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

#define BELTSORTER_SERVO1_PIN     5 
#define BELTSORTER_SERVO2_PIN     3     

#define BELTSORTER_ANALOG_PIN     0 
#define BELTSORTER_BELT_PIN       6


class BeltSorter
{

public:
  BeltSorter();
  void loop();
  void setOffsetServo1(int center);
  void setOffsetServo2(int center);
  void setServo1(int pos);
  void setServo2(int pos);
  float getHeight();
  void setBeltConstant(float cte);
  float getBeltDistance();
  float getBeltSpeed();
  void waitForPeriod();
  void setPeriod(unsigned long period_us);

private:
  Servo beltsorter_servo1;
  Servo beltsorter_servo2;

  int beltsorter_servo1_pos;
  int beltsorter_servo2_pos;
  int beltsorter_servo1_center;
  int beltsorter_servo2_center;
  int beltsorter_maximumRange = 200; // Maximum range needed
  int beltsorter_minimumRange = 0; // Minimum range needed
  long beltsorter_duration;
  float beltsorter_distance; // Duration used to calculate distance
  unsigned long beltsorter_belt_period;
  unsigned long beltsorter_last_time;
  unsigned long beltsorter_period;
  unsigned long beltsorter_last_period;
  float beltsorter_belt_constant;
  float beltsorter_belt_speed;
  float beltsorter_belt_last_distance;

};

#endif
