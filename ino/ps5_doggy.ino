#include <ps5Controller.h>
#include <Cytron_SmartDriveDuo.h>

#define IN1 27
#define IN2 14
#define IN3 12
#define BAUDRATE 115200

Cytron_SmartDriveDuo motor_back(SERIAL_SIMPLIFIED, IN1, BAUDRATE);
Cytron_SmartDriveDuo motor_mid(SERIAL_SIMPLIFIED, IN2, BAUDRATE);
Cytron_SmartDriveDuo motor_front(SERIAL_SIMPLIFIED, IN3, BAUDRATE);

float right_wheel_front = 0;
float left_wheel_front = 0;
float right_wheel_mid = 0;
float left_wheel_mid = 0;
float right_wheel_back = 0;
float left_wheel_back = 0;

float rightY = 0;
float rightX = 0;
float leftY = 0;

float rover_x = 0;
float rover_z = 0;

void setup() {
  Serial.begin(115200);
  
  ps5.begin("7c:66:ef:54:80:8a");  //MAC address of our controller
  Serial.println("Ready.");
}

void loop() {
  while (ps5.isConnected() == true) {
    leftY = ps5.LStickY() / 100.0;
    if (leftY < 0.1) {
    motor_back.control(0,0);
    motor_mid.control(0,0);
    motor_front.control(0,0);
      delay(100);
      break;
    }
    if (ps5.RStickX()) {
      rightX = ps5.RStickX() / 100.0;
    }
    if (ps5.RStickY()) {
      rightY = ps5.RStickY() / 100.0;
    }
    rover_x = (rightY * ((leftY + 1) / 2) + rightX * ((leftY + 1) / 2)) * 100;
    rover_z = (rightY * ((leftY + 1) / 2) - rightX * ((leftY + 1) / 2)) * 100;

    if (rover_x > 100) {
      rover_x = 100;
    }
    if (rover_x < -100) {
      rover_x = -100;
    }
    if (rover_z > 100) {
      rover_z = 100;
    }
    if (rover_z < -100) {
      rover_z = -100;
    }

    /*Serial.print(rover_x);
  Serial.print(" ");
  Serial.println(rover_z);*/

    // Case 1: Both rover_x and rover_z are positive
    if (rover_x >= 0 && rover_z >= 0) {
      left_wheel_front = ((rover_x + rover_z) / 2 + (rover_x - rover_z) / 2) * 0.44;
      right_wheel_front = ((rover_x + rover_z) / 2 - (rover_x - rover_z) / 2) * 0.5;
      right_wheel_mid = ((rover_x + rover_z) / 2 + (rover_x - rover_z) / 2) * 0.501;
      left_wheel_mid = ((rover_x + rover_z) / 2 - (rover_x - rover_z) / 2) * 0.46;
      right_wheel_back = ((rover_x + rover_z) / 2 + (rover_x - rover_z) / 2);
      left_wheel_back = ((rover_x + rover_z) / 2 - (rover_x - rover_z) / 2) * 0.45;
    }

    // Case 2: Both rover_x and rover_z are negative
    else if (rover_x <= 0 && rover_z <= 0) {
      right_wheel_front = ((rover_x + rover_z) / 2 + (rover_x - rover_z) / 2) * 0.54;
      left_wheel_front = ((rover_x + rover_z) / 2 - (rover_x - rover_z) / 2) * 0.52;
      right_wheel_mid = ((rover_x + rover_z) / 2 + (rover_x - rover_z) / 2) * 0.57;
      left_wheel_mid = ((rover_x + rover_z) / 2 - (rover_x - rover_z) / 2) * 0.54;
      right_wheel_back = ((rover_x + rover_z) / 2 + (rover_x - rover_z) / 2);
      left_wheel_back = ((rover_x + rover_z) / 2 - (rover_x - rover_z) / 2) * 0.56;
    }

    // Case 3: rover_x positive, rover_z negative
    else if (rover_x >= 0 && rover_z <= 0) {
      right_wheel_front = ((rover_x + rover_z) / 2 + (rover_x - rover_z) / 2) * 0.5;
      left_wheel_front = ((rover_x + rover_z) / 2 - (rover_x - rover_z) / 2) * 0.52;
      right_wheel_mid = ((rover_x + rover_z) / 2 + (rover_x - rover_z) / 2) * 0.501;
      left_wheel_mid = ((rover_x + rover_z) / 2 - (rover_x - rover_z) / 2) * 0.54;
      right_wheel_back = ((rover_x + rover_z) / 2 + (rover_x - rover_z) / 2);
      left_wheel_back = ((rover_x + rover_z) / 2 - (rover_x - rover_z) / 2) * 0.56;
    }

    // Case 4: rover_x negative, rover_z positive
    else if (rover_x <= 0 && rover_z >= 0) {
      right_wheel_front = ((rover_x + rover_z) / 2 + (rover_x - rover_z) / 2) * 0.54;
      left_wheel_front = ((rover_x + rover_z) / 2 - (rover_x - rover_z) / 2) * 0.5;
      right_wheel_mid = ((rover_x + rover_z) / 2 + (rover_x - rover_z) / 2) * 0.57;
      left_wheel_mid = ((rover_x + rover_z) / 2 - (rover_x - rover_z) / 2) * 0.46;
      right_wheel_back = ((rover_x + rover_z) / 2 + (rover_x - rover_z) / 2);
      left_wheel_back = ((rover_x + rover_z) / 2 - (rover_x - rover_z) / 2) * 0.45;
    }

    // Case 5: Both rover_x and rover_z are zero
    else if (rover_x == 0 && rover_z == 0) {
      right_wheel_front = 0;
      left_wheel_front = 0;
      right_wheel_mid = 0;
      left_wheel_mid = 0;
      right_wheel_back = 0;
      left_wheel_back = 0;
    }

    /*Serial.print("LF: ");           /debugging statements
    Serial.print(left_wheel_front);
    Serial.print(" RF: ");
    Serial.print(right_wheel_front);
    Serial.print(" LM: ");
    Serial.print(left_wheel_mid);
    Serial.print(" RM: ");
    Serial.print(right_wheel_mid);
    Serial.print(" LB: ");
    Serial.print(left_wheel_back);
    Serial.print(" RB: ");
    Serial.println(right_wheel_back);*/

    motor_back.control(right_wheel_front, left_wheel_front);
    motor_mid.control(right_wheel_mid, left_wheel_mid);
    motor_front.control(right_wheel_back, left_wheel_back);
  }
}