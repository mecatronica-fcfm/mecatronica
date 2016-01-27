#include <Servo.h>
#define BELTSORTER_ECHO_PIN       7 // Echo Pin
#define BELTSORTER_TRIGGER_PIN    4 // Trigger Pin
#define BELTSORTER_LED_PIN        13 // Onboard LED

#define BELTSORTER_ENCODER_PIN    2 // Onboard LED

#define BELTSORTER_SERVO1_PIN     5 
#define BELTSORTER_SERVO2_PIN     3     

#define BELTSORTER_ANALOG_PIN     0 
#define BELTSORTER_BELT_PIN       6



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

void BeltSorter_setup() 
{
 Serial.begin (9600);
 pinMode(BELTSORTER_TRIGGER_PIN, OUTPUT);
 pinMode(BELTSORTER_ECHO_PIN, INPUT);
 pinMode(BELTSORTER_LED_PIN, OUTPUT); // Use LED indicator (if required)
 pinMode(BELTSORTER_ENCODER_PIN, INPUT);
 beltsorter_servo1.attach(BELTSORTER_SERVO1_PIN); 
 beltsorter_servo2.attach(BELTSORTER_SERVO2_PIN);
 beltsorter_servo1_pos = 0;
 beltsorter_servo2_pos = 0;
 beltsorter_servo1_center = 90;
 beltsorter_servo2_center = 90;
 beltsorter_belt_period = 0;
 beltsorter_belt_last_distance =0;
 beltsorter_belt_speed = 1;
 beltsorter_belt_constant = 1.0;
 beltsorter_period = 50000;
 beltsorter_last_time=micros();
 beltsorter_last_period = micros();
}


void BeltSorter_loop()
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
 if (beltsorter_belt_period != 0 )
 {
    beltsorter_belt_speed = beltsorter_belt_constant / beltsorter_belt_period;
    beltsorter_belt_last_distance= beltsorter_belt_last_distance + ((micros()-beltsorter_last_time)/1000000.0)*beltsorter_belt_speed;
    beltsorter_last_time= micros();
 }

}

void BeltSorter_set_offset_servo1(int center)
{
  beltsorter_servo1_center = center;
}
void BeltSorter_set_offset_servo2(int center)
{
  beltsorter_servo2_center = center;
}
void BeltSorter_set_servo1(int pos)
{
  beltsorter_servo1_pos  = pos;
}
void BeltSorter_set_servo2(int pos)
{
 beltsorter_servo2_pos = pos;
}
float BeltSorter_get_height()
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

void BeltSorter_print(float n)
{
  Serial.print(n);
}

void BeltSorter_print_tab()
{
  Serial.print("\t");
}
void BeltSorter_print_nl()
{
  Serial.println();
}
void BeltSorter_print_sensor()
{
  Serial.println(beltsorter_distance);
}

void BeltSorter_set_belt_constant(float cte)
{
  beltsorter_belt_constant = cte;
}
float BeltSorter_get_belt_distance()
{
  float aux;
  aux = beltsorter_belt_last_distance;
  beltsorter_belt_last_distance = 0;
  return aux;
}
float BeltSorter_get_belt_speed()
{
  return beltsorter_belt_speed;
}
void BeltSorter_waitForPeriod()
{
  while(micros()<beltsorter_last_period+beltsorter_period)
  {
  }
  beltsorter_last_period = micros();
  
}
void BeltSorter_set_period(unsigned long period_us)
{
  beltsorter_period = period_us;
}




void setup() {
  // put your setup code here, to run once:
  BeltSorter_setup();
  BeltSorter_set_belt_constant(10000);
}

void loop() {
  // put your main code here, to run repeatedly:
  BeltSorter_set_servo1(0);
  BeltSorter_set_servo2(0);
  BeltSorter_loop();
}





