#include <adk.h>
#define LED 13 // Use built in LED
#include <SPI.h>

USB Usb;
ADK adk(&Usb, "Mecatronica", // Manufacturer Name
              "roverto", // Model Name
              "Rover controller using cellular network", // Description (user-visible string)
              "1.0", // Version
              "https://github.com/mecatronica-fcfm", // URL (web page to visit if no installed apps support the accessory)
              "123456789"); // Serial Number (optional)

uint32_t timer;
bool connected;

uint8_t msg[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t msg_cpy[16];
size_t msg_len = sizeof(msg);

void setup() {
  Serial.begin(115200);
  if (Usb.Init() == -1) {
    Serial.print("\r\nOSCOKIRQ failed to assert");
    while (1); // halt
  }
  pinMode(LED, OUTPUT);
  Serial.print("\r\nRoverto Started");
}

void loop() {
  Usb.Task();

  if (adk.isReady()) {
    if (!connected) {
      connected = true;
      Serial.println(F("Connected to accessory"));
    }

    uint8_t cmd[1];
    uint16_t cmd_len = sizeof(cmd);
    uint8_t rcode = adk.RcvData(&cmd_len, cmd);
    if (cmd_len > 0) {
      Serial.print(F("Command received: "));
      Serial.println(cmd[0]);
      digitalWrite(LED, cmd[0] ? HIGH : LOW);
    }

    // Run every 1000 ms
    if (millis() - timer >= 1000)
    {
      timer = millis();

      // Get data
      update_msg();
      // Send data
      memcpy(msg_cpy, msg, 16); // Copy data
      rcode = adk.SndData(msg_len, msg_cpy);
      if (rcode != hrNAK) {
        Serial.println(F("Message sent"));
      }
      
    }
  } else {
    if (connected) {
      connected = false;
      Serial.println(F("Disconnected from accessory"));
      digitalWrite(LED, LOW);
    }
  }
}

void update_msg()
{
  // Fill de message with sensor data
  // Example
  for(uint8_t i=1; i < msg_len; ++i)
    msg[i]=i;
  msg[0]+=1;
}

