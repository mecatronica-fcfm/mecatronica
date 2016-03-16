#include <BeltSorter.h>

/*
 * ----------------------------------------------
 * Pusher class
 *
*/
Pusher::Pusher(uint8_t open_pos, uint8_t close_pos):
  _open_pos(open_pos),
  _close_pos(close_pos)
{
}

void Pusher::attach(int pin)
{
  _servo.attach(pin);
}

void Pusher::open()
{
  _servo.write(_open_pos);
}

void Pusher::close()
{
  _servo.write(_close_pos);
}

/*
 * ----------------------------------------------
 * BeltSorter class
 *
*/
BeltSorter::BeltSorter():
  _pusherA(90,5),
  _pusherB(90,175),
  beltsorter_belt_period(0UL),
  beltsorter_belt_last_distance(0.0f),
  beltsorter_belt_speed(1.0f),
  _encoder_constant(550.0f),
  beltsorter_period(50000UL)
{
  pinMode(BELTSORTER_TRIGGER_PIN, OUTPUT);
  pinMode(BELTSORTER_ECHO_PIN, INPUT);
  pinMode(BELTSORTER_LED_PIN, OUTPUT); // Use LED indicator (if required)
  pinMode(BELTSORTER_ENCODER_PIN, INPUT);


  beltsorter_last_time = micros();
  beltsorter_last_period = micros();
}

void BeltSorter::begin()
{
   // Attach servos
  _pusherA.attach(BELTSORTER_PUSHER1_PIN);
  _pusherB.attach(BELTSORTER_PUSHER2_PIN);
}

void BeltSorter::spin()
{
  // Ultrasonic pulse
  digitalWrite(BELTSORTER_TRIGGER_PIN, LOW); 
  delayMicroseconds(2); 
  digitalWrite(BELTSORTER_TRIGGER_PIN, HIGH);
  delayMicroseconds(10); 
  digitalWrite(BELTSORTER_TRIGGER_PIN, LOW);
  // Meeasure distance
  beltsorter_duration = pulseIn(BELTSORTER_ECHO_PIN, HIGH);
  //Serial.println(beltsorter_duration);
  // Saturation range
  beltsorter_duration = beltsorter_duration > _max_range ? _max_range : (beltsorter_duration < _min_range ? _min_range : beltsorter_duration);
  beltsorter_distance = 175.0f - (float)beltsorter_duration/5.9f;
  
  // Encoder
  beltsorter_belt_period = pulseIn(BELTSORTER_ENCODER_PIN,HIGH,10000);
  analogWrite(BELTSORTER_BELT_PIN, analogRead(BELTSORTER_ANALOG_PIN)/2);
  if (beltsorter_belt_period != 0)
  {
    beltsorter_belt_speed = _encoder_constant/beltsorter_belt_period;
    beltsorter_belt_last_distance= beltsorter_belt_last_distance + ((micros()-beltsorter_last_time)/1000000.0)*beltsorter_belt_speed;
    beltsorter_last_time= micros();
  }
}

void BeltSorter::openPusherA()
{
  _pusherA.open();
}

void BeltSorter::openPusherB()
{
  _pusherB.open();
}

void BeltSorter::closePusherA()
{
  _pusherA.close();
}

void BeltSorter::closePusherB()
{
  _pusherB.close();
}

float BeltSorter::getHeight()
{
  return beltsorter_distance;
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
