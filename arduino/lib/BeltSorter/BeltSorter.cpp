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
  _urDuration(0UL),
  _encoderPeriod(0UL),
  _beltLastTime(0UL),
  _period(50000UL),
  _lastPeriod(0UL),
  _urHeight(0.0f),
  _beltSpeed(0.0f),
  _beltLastDistance(0.0f),
  _pusherA(90,5),
  _pusherB(90,175)
{
  // Pin config
  pinMode(BELTSORTER_TRIGGER_PIN, OUTPUT);
  pinMode(BELTSORTER_ECHO_PIN, INPUT);
  pinMode(BELTSORTER_LED_PIN, OUTPUT); // Use LED indicator (if required)
  pinMode(BELTSORTER_ENCODER_PIN, INPUT);
}

void BeltSorter::begin()
{
   // Attach servos
  _pusherA.attach(BELTSORTER_PUSHER1_PIN);
  _pusherB.attach(BELTSORTER_PUSHER2_PIN);
  // Set init times
  _beltLastTime = micros();
  _lastPeriod = micros();
}

void BeltSorter::loop()
{
  // Ultrasonic range
  digitalWrite(BELTSORTER_TRIGGER_PIN, LOW); 
  delayMicroseconds(2); 
  digitalWrite(BELTSORTER_TRIGGER_PIN, HIGH);
  delayMicroseconds(10); 
  digitalWrite(BELTSORTER_TRIGGER_PIN, LOW);
  // Meeasure distance
  _urDuration = pulseIn(BELTSORTER_ECHO_PIN, HIGH);
  // Saturation range
  _urDuration = _urDuration > _urMaxRange ? _urMaxRange : (_urDuration < _urMinRange ? _urMinRange : _urDuration);
  // Calc object height (empirical constants)
  _urHeight = 239.0f - (float)_urDuration/5.9f;
  _urHeight = _urHeight < 0.0f ? 0.0f : _urHeight;
  
  // Encoder
  _encoderPeriod = pulseIn(BELTSORTER_ENCODER_PIN,HIGH,10000);
  if (_encoderPeriod != 0)
  {
    _beltSpeed = _encoderConstant/_encoderPeriod;
    _beltLastDistance = _beltLastDistance + _beltSpeed*((micros()-_beltLastTime)/1000000.0);// d = d0 + v*t
    _beltLastTime = micros();
  }
  // Modify belt speed
  analogWrite(BELTSORTER_BELT_PIN, analogRead(BELTSORTER_ANALOG_PIN)/2);
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
  return _urHeight;
}

float BeltSorter::getBeltDistance()
{
  float aux;
  aux = _beltLastDistance;
  _beltLastDistance = 0;
  return aux;
}

float BeltSorter::getBeltSpeed()
{
  return _beltSpeed;
}

void BeltSorter::sleep()
{
  while(micros() < _lastPeriod + _period)
  {
  }
  _lastPeriod = micros();
}

void BeltSorter::setPeriod(unsigned long period_us)
{
  _period = period_us;
}
