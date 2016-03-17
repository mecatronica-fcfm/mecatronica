#include <BeltSorter.h>
#include <Servo.h>

BeltSorter bs;

void setup()
{
  bs.begin();
  Serial.begin(9600);
}


void loop()
{
  // Open pushers
  bs.openPusherA();
  bs.openPusherB();
  Serial.println("Opening pushers");

  while(true)
  {
    // Update values
    bs.loop();
    // Get object height and compare
    if(bs.getHeight() > 10.0)
    {
      // Close pusher and wait
      bs.closePusherA();
      // Pusher A at 35 cm
      delay(35.0/bs.getBeltSpeed()*1000.0);
      // Open Pusher A
      bs.openPusherA();
    }
    bs.sleep();
  }
}