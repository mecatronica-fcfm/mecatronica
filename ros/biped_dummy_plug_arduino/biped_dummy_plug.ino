/*
 * rosserial Biped Subscriber Example
 * Use ServoArray Message to Activate Servos
 */
    
#include <ros.h>
#include <c_msg/ServoArray.h>
#include <Servo.h>     
    
// ROS Handle     
ros::NodeHandle nh;
   
// Servos
Servo leftHipServo;
Servo leftKneeServo;
Servo leftAnkleServo;
Servo rightHipServo;
Servo rightKneeServo;
Servo rightAnkleServo;
   
void messageCb( const c_msg::ServoArray& msg)
{
  leftHipServo.write(msg.left_hip_angle);
}
   
ros::Subscriber<c_msg::ServoArray> sub("/biped_dummy_plug_node/t_servo_array", &messageCb );
   
void setup()
{
  pinMode(13, OUTPUT);
  
  // Init ROS Node and Subscriber
  nh.initNode();
  nh.subscribe(sub);
  
  // Attach Servos
  leftHipServo.attach(11);
  leftKneeServo.attach(10);
  leftAnkleServo.attach(9);
  
  rightHipServo.attach(6);
  rightKneeServo.attach(5);
  rightAnkleServo.attach(3);
}
   
void loop()
{
  nh.spinOnce();
  delay(1);
}
