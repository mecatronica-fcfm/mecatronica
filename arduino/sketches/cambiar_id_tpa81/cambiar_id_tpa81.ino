/********************************************
*    arduino example for TPA81 and LCD03    *
*         TPA81 controlled by I2C           *
*        LCD03 controlled by serial         *
*                                           *
*         By James Henderson 2012           *
********************************************/

#include <Wire.h>

#define ADDRESS 0xD2                                   // Address of TPA81

int temperature[] = {0,0,0,0,0,0,0,0};                   // Array to hold temperature data

void setup(){                                    // Starts software serial port for LCD03
  Wire.begin();
  Serial.begin(9600);
  delay(100);                                            // Wait to make sure everything is powerd up

// Cambiar ID
//  Wire.beginTransmission(ADDRESS>>1);                        // Begin communication with TPA81
//  Wire.write(0x00);
//  Wire.write(0xA0);
//  Wire.endTransmission();
//  delayMicroseconds(80);
//
//  Wire.beginTransmission(ADDRESS>>1);                        // Begin communication with TPA81
//  Wire.write(0x00);
//  Wire.write(0xAA);
//  Wire.endTransmission();
//  delayMicroseconds(80);
//  
//  Wire.beginTransmission(ADDRESS>>1);                        // Begin communication with TPA81
//  Wire.write(0x00);
//  Wire.write(0xA5);
//  Wire.endTransmission();
//  delayMicroseconds(80);
//  
//  Wire.beginTransmission(ADDRESS>>1);                        // Begin communication with TPA81
//  Wire.write(0x00);
//  Wire.write(0xD2);
//  Wire.endTransmission();
//  delayMicroseconds(80);
  
}
  
void loop(){
  for(int i = 0; i < 8; i++){                            // Loops and stores temperature data in array
    temperature[i] = getData(i+2);
    Serial.print(temperature[i]);
    Serial.print(",");
  }
  Serial.println(".");
  delay(200);
}

byte getData(byte reg){                                   // Function to receive one byte of data from TPA81
  Wire.beginTransmission(ADDRESS>>1);                        // Begin communication with TPA81
  Wire.write(reg);                                      // Send reg to TPA81
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS>>1, 1);                           // Request 1 byte
  while(Wire.available() < 1);                            // Wait for byte to arrive
  byte data = Wire.read();                                // Get byte
  return(data);                                           // return byte
}
