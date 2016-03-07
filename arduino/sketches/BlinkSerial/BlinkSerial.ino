/*
  ASCII to Bin Led
  
  Serial drives LEDS 0 to 7 using port mapping

  created 8 Mar 2016
  by Kenzo Lobos
 */

void setup() {
  // Configure Pins (Pin 0 to 7 as output). 
  DDRD = B11111111;

  // Initialize Serial (the use of pin 0 and 1 are therefore disabled !)
  Serial.begin(9600);
}

// Loop forever and ever
void loop() {

  while (Serial.available()) 
  {
    // Receive Incoming Char
    PORTD = (char)Serial.read();

    // Echo Back received char (if no casting is done, a string will be written back...)
    Serial.print((char)PORTD);

    // Delay, so we can see if multiple bytes are received.
    delay(1000);  
  }
  
}
