#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <ESP32Servo.h>
#include "ArduinoHardware.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
ros::NodeHandle nh;


int dir1=4;
int pwm1=0; 

int dir2=33;
int pwm2=32;

int dir3=26; 
int pwm3=25;
int dir4=12; //scissor lift
int pwm4=14;
int dir5=27; // augre
int pwm5=13;
int dir6=5; //magnetic mixer
int pwm6=15;
int en=18;
int dirst=19;
int step=23;
float b[13];
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;
Servo servo8;



void Callback( const std_msgs::Float32MultiArray& val){
  

  memcpy(b, val.data, sizeof(b));
}
ros::Subscriber<std_msgs::Float32MultiArray> Sub("publish", &Callback);


void callback1( const std_msgs::Int16& val){
  
 servo1.write(val.data);
}
ros::Subscriber<std_msgs::Int16> Sub1("servoc1", &callback1);

void callback2( const std_msgs::Int16& val){
  
 servo2.write(val.data);
}
ros::Subscriber<std_msgs::Int16> Sub2("servoc2", &callback2);

void callback3( const std_msgs::Int16& val){
  
 servo3.write(val.data);
}
ros::Subscriber<std_msgs::Int16> Sub3("servoc3", &callback3);


void setup() {
  pinMode(dir1,OUTPUT);
  pinMode(dir2,OUTPUT);
  pinMode(dir3,OUTPUT);
  pinMode(dir4,OUTPUT);
  pinMode(dir5,OUTPUT);
  pinMode(dir6,OUTPUT);
  pinMode(pwm1,OUTPUT);
  pinMode(pwm2,OUTPUT);
  pinMode(pwm3,OUTPUT);
  pinMode(pwm4,OUTPUT);
  pinMode(pwm5,OUTPUT);
  pinMode(pwm6,OUTPUT);
  pinMode(en,OUTPUT);
  pinMode(dirst,OUTPUT);
  pinMode(step,OUTPUT);
  digitalWrite(en,HIGH);
  nh.getHardware()->setBaud(115200);
  nh.subscribe(Sub);
  nh.subscribe(Sub1);
  nh.subscribe(Sub2);
  nh.subscribe(Sub3);

  nh.initNode();
  servo1.attach(22);
  servo2.attach(21);
  servo3.attach(17);





}

void loop() {
  analogWrite(pwm1,0);
  analogWrite(pwm2,0);
  analogWrite(pwm3,0);
  analogWrite(pwm4,0);
  analogWrite(pwm5,0);
  analogWrite(pwm6,0);



  if(b[0]==1)
  { //pumps are unidirectional and on digital high
    digitalWrite(dir1, HIGH); 
 
    digitalWrite(pwm1,HIGH);  
  
  }
  else if(b[0]==2)
  {
    digitalWrite(dir2, HIGH); 
 
    digitalWrite(pwm2,HIGH);  
  
  }
  else if(b[0]==3)
  {
    digitalWrite(dir3, HIGH); 
 
    digitalWrite(pwm3,HIGH);  
  
  }
  
  // the 4 buttons on the right. If rightmost is pressed then b[2]=1 (scissor lift) if uppermost is pressed then b[2] is 2 (augre) and if leftmost is pressed then b[2] is 3 (magnet)
  // have to hold down the mentioned button and then move the left joystick up or down to change direction (button must be pressed while doing this)
  else if(b[2]==1 && b[3]>0.1)  // scissor lift, magnetic spinner and augre are bi directional and analog
  {
    digitalWrite(dir4, HIGH); 

    analogWrite(pwm4,b[3]*255);  

 
  }
  else if(b[2]==1 && b[3]<-0.1)
  {
    digitalWrite(dir4, LOW); 

    analogWrite(pwm4,(-1*b[3])*255);  

 
  }
  else if(b[2]==2 && b[1]>0.1)
  {
    digitalWrite(dir5, HIGH); 

    analogWrite(pwm5,b[3]*255);  

  }
  else if(b[2]==2 && b[1]<-0.1)
  {
    digitalWrite(dir5, LOW); 

    analogWrite(pwm5,(-1*b[3])*255);  

  }
  else if(b[2]==3 && b[3]>0.1)
  {
    digitalWrite(dir6, HIGH); 

    analogWrite(pwm6,b[3]*255);  

  
  }
  else if(b[2]==3 && b[3]<-0.1)
  {
    digitalWrite(dir6, LOW); 

    analogWrite(pwm6,(-1*b[3])*255);  


 
  }
  //b[4] is 1 when left trigger is pressed: enabled. b[4]=0 when right trigger is pressed. b[5] is 1 when the lowermost of the left 4 buttons is pressed. is -1 when lowermost of right 4 buttons is pressed.
  if(b[4]==1){
    digitalWrite(en,LOW); // stepper is enabled 
    if(b[5]==1)
   {  digitalWrite(dirst,LOW); //Changes the direction of rotation
  for(int x = 0; x < 5; x++) {
    digitalWrite(step,HIGH);
    delayMicroseconds(500);
    digitalWrite(step,LOW);
    delayMicroseconds(500);
  
   }
   }
    else if(b[5]==-1)
    {digitalWrite(dirst,HIGH); // Enables the motor to move in a particular direction
     for(int x = 0; x < 5; x++) {
    digitalWrite(step,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(step,LOW); 
    delayMicroseconds(500);
  
    }
  }
  }
  if(b[4]==0)
  {
    digitalWrite(en,HIGH);
  }
  
  nh.spinOnce();

}