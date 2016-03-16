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
  Serial.println("Opening pushers");
  bs.openPusherA();
  bs.openPusherB();
  delay(300);
  Serial.println("Closing pushers");
  bs.closePusherA();
  bs.closePusherB();
  
  while(true)
  {
    bs.spin();
    // Print belt speed
    Serial.print("Belt speed: ");
    Serial.print(bs.getBeltSpeed());
    Serial.println(" [cm/s]");
    // Print Height sensor value
    Serial.print("Height: ");
    Serial.print(bs.getHeight());
    Serial.println(" [mm]");
    delay(10);
  }
}
