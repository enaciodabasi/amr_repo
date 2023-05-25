//#define _SAM3XA_
//#define USE_USBCON

#include <BMI160.h>
#include <BMI160Gen.h>
#include <CurieIMU.h>

#include <I2Cdev.h>
#include <Wire.h>
#include <MPU6050.h>

#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <amr_custom_interfaces/WheelVelCmd.h>
#include <amr_custom_interfaces/EncoderData.h>
#include <sensor_msgs/Imu.h>
#include <ArduinoHardware.h>


#define LOW 0
#define HIGH 1

MPU6050 six_axis_imu;

int16_t ax, ay, az, gx, gy, gz;

const byte leftEncoderPinA = 2;
const byte leftEncoderPinB = 2;
const byte leftMotorPin1 = 1; // MC33886 IN1
const byte leftMotorPin2 = 1; // MC33886 IN2

const byte rightEncoderPinA = 2;
const byte rightEncoderPinB = 2;
const byte rightMotorPin1 = 1; // MC33886 IN3
const byte rightMotorPin2 = 1; // MC33886 IN4

int leftMotorPwmCommand = 0;
int rightMotorPwmCommand = 0; 

struct EncoderInfo
{
  volatile int count = 0;
  bool direction = true; // true = forwards
                        // false = backwards

};

EncoderInfo leftMotorEncoder;
EncoderInfo rightMotorEncoder; 

// ROS related variables:
ros::NodeHandle nh;
amr_custom_interfaces::EncoderData encoderDataMsg;
ros::Publisher encoderDataPub("/amr/hardware/encoder_data", &encoderDataMsg);

void pwm_command_callback(const amr_custom_interfaces::WheelVelCmd& pwm_commands)
{
  leftMotorPwmCommand = pwm_commands.leftWHeelVelCmd;
  rightMotorPwmCommand = pwm_commands.rightWheelVelCmd;
}
ros::Subscriber<amr_custom_interfaces::WheelVelCmd> pwmCmdSub("/amr/hardware/pwm_commands", &pwm_command_callback);

void shutdownArduino()
{

}


/*
  X2 encoding method:
*/
void leftMotorEncoderInterruptFunc()
{
  int pinAState = digitalRead(leftEncoderPinA);
  int pinBState = digitalRead(leftEncoderPinB);

  if(pinBState == LOW)
  {
    if(pinAState == LOW) // B leads A
    {
      leftMotorEncoder.count -= 1;
    }
    else if(pinAState == HIGH) // B lags A
    {
      leftMotorEncoder.count += 1;
    }
  }
  else if(pinBState == HIGH)
  {
    if(pinAState == LOW)
    {
      leftMotorEncoder.count += 1;
    }
    else if(pinAState == HIGH)
    {
      leftMotorEncoder.count -= 1;
    }
  }

}

void rightMotorEncoderInterruptFunc()
{
  int pinAState = digitalRead(rightEncoderPinA);
  int pinBState = digitalRead(rightEncoderPinB);

  if(pinBState == LOW)
  {
    if(pinAState == LOW) // B leads A
    {
      rightMotorEncoder.count -= 1;
    }
    else if(pinAState == HIGH) // B lags A
    {
      rightMotorEncoder.count += 1;
    }
  }
  else if(pinBState == HIGH)
  {
    if(pinAState == LOW)
    {
      rightMotorEncoder.count += 1;
    }
    else if(pinAState == HIGH)
    {
      rightMotorEncoder.count -= 1;
    }
  }

}

void read() // Reads data from the hardware.
{
  //encoderDataMsg.encoder_left = leftMotorEncoder.count;
  //encoderDataMsg.encoder_right = rightMotorEncoder.count;

  encoderDataMsg.encoder_left = 500;
  encoderDataMsg.encoder_right = 500;

  encoderDataPub.publish(&encoderDataMsg);
}

void write()
{
  int pwmCmdLeft = 0;
  auto leftMotorDirection = HIGH; // true = forward, false = backward;
  int pwmCmdRight = 0;
  auto rightMotorDirection = HIGH;

  if(leftMotorPwmCommand < 0)
  {
    pwmCmdLeft = abs(leftMotorPwmCommand);
    leftMotorDirection = LOW;
  }
  else
  {
    pwmCmdLeft = leftMotorPwmCommand;
    leftMotorDirection = HIGH;
  }
  if(rightMotorPwmCommand < 0)
  {
    pwmCmdRight = abs(rightMotorPwmCommand);
    rightMotorDirection = LOW;
  }
  else
  {
    pwmCmdRight = rightMotorPwmCommand;
    rightMotorDirection = HIGH;
  }

  if(leftMotorDirection == HIGH)
  {
    digitalWrite(leftMotorPin1, LOW);
    analogWrite(leftMotorPin2, pwmCmdLeft);
  }
  else
  {
    digitalWrite(leftMotorPin2, LOW);
    digitalWrite(leftMotorPin1, pwmCmdLeft);
  }

  if(rightMotorDirection == HIGH)
  {
    digitalWrite(rightMotorPin1, LOW);
    analogWrite(rightMotorPin2, pwmCmdRight);
  }
  else
  {
    digitalWrite(rightMotorPin1, LOW);
    analogWrite(rightMotorPin2, pwmCmdRight);
  }

}



void setup() {
  Wire.begin();
  nh.getHardware()->setBaud(115200);

  nh.initNode();
  nh.advertise(encoderDataPub);

  while(!nh.connected()) {nh.spinOnce();}
  six_axis_imu.initialize();

  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);

  
  if(!six_axis_imu.testConnection())
  {
  }

 

}


void loop() {

  nh.spinOnce(); // Get data via callbacks.

  read();
  
  write();

  
  
  delay(4);
    
}



