
#include <Dynamixel2Arduino.h>
#include <SoftwareSerial.h>
#include <ESP32Servo.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle  nh;

//Software serial for dynamixel - for rs485, remember to change Hardwareserial to softwareserial.
//SoftwareSerial (RX, TX)
//SoftwareSerial (RO, DI)
SoftwareSerial toRS485(34, 22);
const uint8_t DXL_DIR_PIN = 15; // DYNAMIXEL DIR PIN (RE/DE)

const uint8_t wristRot = 15;
const float DXL_PROTOCOL_VERSION = 2.0;

#define DXL_SERIAL   toRS485
#define DEBUG_SERIAL Serial

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);


// create fourteen servo objects
Servo wristFlex;
Servo wristPiv;
//Servo wristRot;
Servo thumbMeta;
Servo thumbPiv;
Servo indexDis;
Servo indexMeta;
Servo indexPiv;
Servo middleDis;
Servo middleMeta;
Servo middlePiv;
Servo ringDis;
Servo ringMeta;
Servo pinkyDis;
Servo pinkyMeta;

Servo elbowLockA;
Servo elbowLockB;

int minUs = 500;
int maxUs = 2500;
int steps = 0;
int pos = 0;      // position in degrees
ESP32PWM pwm;

void servo_cb( const std_msgs::Float32MultiArray& cmd_msg) {

  wristFlex.write(map(cmd_msg.data[0], -15, 90, 10, 165));
  wristPiv.write(map(cmd_msg.data[1], -45, 45, 20, 160));
  //wristRot.write(map(cmd_msg.data[2], -45, 200, 0, 180));
  // 0 = 3075 -90 = 2040 270 = 6250
  dxl.setGoalPosition(wristRot, map(cmd_msg.data[2], -180, 180, 1024, 5125));
  
  thumbMeta.write(map(cmd_msg.data[3], 0, 90, 10, 170));
  thumbPiv.write(map(cmd_msg.data[4], 0, 90, 30, 150));

  indexDis.write(map(cmd_msg.data[5], 0, 180, 10, 175));
  indexMeta.write(map(cmd_msg.data[6], 0, 90, 10, 175));

  indexPiv.write(map(cmd_msg.data[7], -10, 20, 10, 100));

  middleDis.write(map(cmd_msg.data[8], 0, 180, 10, 179));
  middleMeta.write(map(cmd_msg.data[9], 0, 90, 10, 179));
  
  middlePiv.write(map(cmd_msg.data[10], -45, 45, 30, 150));
  
  ringDis.write(map(cmd_msg.data[11], 0, 180, 10, 170));
  ringMeta.write(map(cmd_msg.data[12], 0, 90, 30, 179));

  pinkyDis.write(map(cmd_msg.data[13], 0, 180, 10, 150));
  pinkyMeta.write(map(cmd_msg.data[14], 0, 90, 10, 170));

  /*if(cmd_msg.data[15] > 50)
  { //lock
    //elbowLockA.write(map(cmd_msg.data[15], 0, 1, 20, 50));
    //elbowLockB.write(map(cmd_msg.data[15], 0, 1, 150, 100));
    elbowLockA.write(20);
    elbowLockB.write(150);
  }
  else
  {//unlocked
    elbowLockA.write(50);
    elbowLockB.write(100);
  }
  */
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("rightservo", servo_cb);


void setup() {
  /***************************************************************
    Dynamixel communication and servo setup
  ***************************************************************/
  // comment out serial monitor when done:
  //DEBUG_SERIAL.begin(57600);
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(wristRot);
  dxl.torqueOff(wristRot);
  dxl.setOperatingMode(wristRot, OP_CURRENT_BASED_POSITION);
  dxl.torqueOn(wristRot);
  dxl.setGoalPosition(wristRot, 3075);

  /***************************************************************
    ESP32 Servo setup
  ***************************************************************/

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  //  wristFlex.setPeriodHertz(49.8);
  //  wristPiv.setPeriodHertz(50);
  //  wristRot.setPeriodHertz(50);
  //  thumbMeta.setPeriodHertz(50);
  //  thumbPiv.setPeriodHertz(50);
  //  indexDis.setPeriodHertz(50);
  //  indexMeta.setPeriodHertz(50);
  //  indexPiv.setPeriodHertz(50);
  //  middleDis.setPeriodHertz(50);
  //  middleMeta.setPeriodHertz(49.8);
  //  middlePiv.setPeriodHertz(50);
  //  ringDis.setPeriodHertz(50);
  //  ringMeta.setPeriodHertz(50);
  //  pinkyDis.setPeriodHertz(50);
  //  pinkyMeta.setPeriodHertz(50);

  //servo1.setPeriodHertz(50);      // Standard 50hz servo
  //servo2.setPeriodHertz(50);      // Standard 50hz servo
  //servo3.setPeriodHertz(330);      // Standard 50hz servo
  //servo4.setPeriodHertz(200);      // Standard 50hz servo
  //servo5.setPeriodHertz(50);      // Standard 50hz servo

  wristFlex.attach(33, minUs, maxUs);
  wristPiv.attach(32, minUs, maxUs);
  //wristRot.attach(, minUs, maxUs);
  thumbMeta.attach(2, minUs, maxUs);
  thumbPiv.attach(27, minUs, maxUs);
  indexDis.attach(14, minUs, maxUs);
  indexMeta.attach(4, minUs, maxUs);
  indexPiv.attach(12, minUs, maxUs);
  middleDis.attach(26, minUs, maxUs);
  middleMeta.attach(17, minUs, maxUs);
  middlePiv.attach(18, minUs, maxUs);
  ringDis.attach(25, minUs, maxUs);
  ringMeta.attach(16, minUs, maxUs);
  pinkyDis.attach(21, minUs, maxUs);
  pinkyMeta.attach(19, minUs, maxUs);

  
  elbowLockA.attach(23);
  elbowLockB.attach(13);
  /***************************************************************
    Rosnode setup
  ***************************************************************/
  nh.initNode();
  nh.subscribe(sub);


}

void loop() {
  nh.spinOnce();
  delay(1);
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  if (x > in_max)
  {
    return out_max;
  }
  else if (x < in_min)
  {
    return out_min;
  }
  else
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
}
