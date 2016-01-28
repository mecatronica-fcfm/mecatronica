#include <BeltSorter.h>

BeltSorter::BeltSorter():
  beltsorter_servo1_pos(0),
  beltsorter_servo2_pos(0),
  beltsorter_servo1_center(90),
  beltsorter_servo2_center(90),
  beltsorter_belt_period(0UL),
  beltsorter_belt_last_distance(0.0f),
  beltsorter_belt_speed(1.0f),
  beltsorter_belt_constant(1.0f),
  beltsorter_period(50000UL)
{
  pinMode(BELTSORTER_TRIGGER_PIN, OUTPUT);
  pinMode(BELTSORTER_ECHO_PIN, INPUT);
  pinMode(BELTSORTER_LED_PIN, OUTPUT); // Use LED indicator (if required)
  pinMode(BELTSORTER_ENCODER_PIN, INPUT);
  beltsorter_servo1.attach(BELTSORTER_SERVO1_PIN); 
  beltsorter_servo2.attach(BELTSORTER_SERVO2_PIN);
  beltsorter_last_time = micros();
  beltsorter_last_period = micros();
}


void BeltSorter::loop()
{
  digitalWrite(BELTSORTER_TRIGGER_PIN, LOW); 
  delayMicroseconds(2); 
  digitalWrite(BELTSORTER_TRIGGER_PIN, HIGH);
  delayMicroseconds(10); 
  digitalWrite(BELTSORTER_TRIGGER_PIN, LOW);
  beltsorter_duration = pulseIn(BELTSORTER_ECHO_PIN, HIGH);
  beltsorter_distance = beltsorter_duration/5.82;
  beltsorter_servo1.write(beltsorter_servo1_pos+beltsorter_servo1_center);
  beltsorter_servo2.write(beltsorter_servo2_pos+beltsorter_servo2_center);
  beltsorter_belt_period = pulseIn(BELTSORTER_ENCODER_PIN,HIGH,10000);
  analogWrite(BELTSORTER_BELT_PIN, analogRead(BELTSORTER_ANALOG_PIN)/2);
  if (beltsorter_belt_period != 0)
  {
    beltsorter_belt_speed = beltsorter_belt_constant / beltsorter_belt_period;
    beltsorter_belt_last_distance= beltsorter_belt_last_distance + ((micros()-beltsorter_last_time)/1000000.0)*beltsorter_belt_speed;
    beltsorter_last_time= micros();
  }
}

void BeltSorter::setOffsetServo1(int center)
{
  beltsorter_servo1_center = center;
}
void BeltSorter::setOffsetServo2(int center)
{
  beltsorter_servo2_center = center;
}
void BeltSorter::setServo1(int pos)
{
  beltsorter_servo1_pos  = pos;
}
void BeltSorter::setServo2(int pos)
{
  beltsorter_servo2_pos = pos;
}
float BeltSorter::getHeight()
{
  if (beltsorter_distance >= beltsorter_maximumRange || beltsorter_distance <= beltsorter_minimumRange)
  {
    return -1;
  }
  else
  {
    return beltsorter_distance;
  }
}

void BeltSorter::setBeltConstant(float cte)
{
  beltsorter_belt_constant = cte;
}

float BeltSorter::getBeltDistance()
{
  float aux;
  aux = beltsorter_belt_last_distance;
  beltsorter_belt_last_distance = 0;
  return aux;
}

float BeltSorter::getBeltSpeed()
{
  return beltsorter_belt_speed;
}

void BeltSorter::waitForPeriod()
{
  while(micros()<beltsorter_last_period+beltsorter_period)
  {
  }
  beltsorter_last_period = micros();
  
}

void BeltSorter::setPeriod(unsigned long period_us)
{
  beltsorter_period = period_us;
}