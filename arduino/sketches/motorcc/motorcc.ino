/*
Control de motores de CC con CI L298



*/
// Pines de entrada para CI L298
#define MOTOR_CTL1  8  // I1 Input 1
#define MOTOR_CTL2  10  // I2 Input 1
#define MOTOR_PWM  9 // EA Enable A

#define MOTOR_DIR_FORWARD  0   // Adelante
#define MOTOR_DIR_BACKWARD  1  // Atras

void setup()
{
   // Configuracion de pines para control del motor
   
   // Control de sentido de giro
   pinMode(MOTOR_CTL1,OUTPUT);
   pinMode(MOTOR_CTL2,OUTPUT);
   // Control de velocidad
   pinMode(MOTOR_PWM,OUTPUT);
}

// Control de velocidad mediante PWM
// 0 < motor_speed < 255
void setSpeed(byte motor_speed)
{
  analogWrite(MOTOR_PWM, motor_speed);
}

// Cambiar el sentido de giro
void motorMove(boolean direction)
{
   switch (direction)
   {
     case MOTOR_DIR_FORWARD:
     {
       digitalWrite(MOTOR_CTL1,LOW);
       digitalWrite(MOTOR_CTL2,HIGH);          
     }
     break; 
          
     case MOTOR_DIR_BACKWARD:
     {
        digitalWrite(MOTOR_CTL1,HIGH);
        digitalWrite(MOTOR_CTL2,LOW);          
     }
     break;         
   }
}

// Frenar el motor
void motorStop()
{
   setSpeed(255); // Habilitar
   digitalWrite(MOTOR_CTL1,HIGH); // Frenar
   digitalWrite(MOTOR_CTL2,HIGH);
}

// Motor libre
void motorFree()
{
   setSpeed(255); // Habilitar
   digitalWrite(MOTOR_CTL1,LOW); // Liberar
   digitalWrite(MOTOR_CTL2,LOW);
}

void loop()
{
  // Movimiento en avance
  motorMove(MOTOR_DIR_FORWARD);
  setSpeed(210);
  delay(2000);
  setSpeed(255);// Aumentar velocidad
  delay(1000);
  // Motor en retroceso
  motorMove(MOTOR_DIR_BACKWARD);
  delay(2000);
  setSpeed(210);// Reducir Velocidad
  delay(1000);
  // Frenado
  motorStop();
  delay(2000);
  // Libre
  motorFree();
  delay(2000);
}
