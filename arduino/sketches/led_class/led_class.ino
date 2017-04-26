#include "LedClass.h"

#define LED_PIN 13

LedClass led(LED_PIN);
uint8_t led_state;
uint8_t cmd;

void setup()
{
	Serial.begin(9600);
	led.on();
}

void loop()
{
	if(Serial.available() >= 1)
	{
		cmd = Serial.parseInt(); //Leer un entero por serial
		if(cmd == 1)
		{
			led.on();
			Serial.println("LED ON");
		}
		else if (cmd == 2)
		{
			led.off();
			Serial.println("LED OFF");
		}
		else if (cmd == 3)
		{
			led_state = led.getState();
			Serial.print("LED State: ");
			Serial.println(led_state);
		}
		else if (cmd == 4)
		{
			Serial.println("LED Blinking");
			led.blink();
			Serial.println("LED OFF");
		}
	}
}