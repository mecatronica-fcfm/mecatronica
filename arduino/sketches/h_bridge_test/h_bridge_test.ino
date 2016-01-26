/*
 * H Bridge test
 */

// Inputs
#define I1 5
#define I2 7
// Enable (PWM)
#define EN 6

void setup()
{   
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(EN, OUTPUT);  
}

void loop()
{
  digitalWrite(I1, HIGH);
  digitalWrite(I2, HIGH);
  analogWrite(EN, 125);
  delay(500);

  digitalWrite(I1, LOW);
  digitalWrite(I2, HIGH);
  analogWrite(EN, 125);
  delay(500);
  
  digitalWrite(I1, HIGH);
  digitalWrite(I2, LOW);
  analogWrite(EN, 125);
  delay(500);

  digitalWrite(I1, LOW);
  digitalWrite(I2, LOW);
  analogWrite(EN, 125);
  delay(500);
}
