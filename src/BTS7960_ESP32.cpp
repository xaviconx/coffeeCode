/******************************************************************************

******************************************************************************/

#include "BTS7960_ESP32.h"
#include <Arduino.h>
#include <driver/ledc.h>

MotorBTS::MotorBTS(int RPWMpin, int LPWMpin, int offset, int freq, int resolution, int channel1_pin, int channel2_pin)
{
//   In1 = In1pin;
//   In2 = In2pin;
  RPWM = RPWMpin;
  LPWM = LPWMpin;
  //Standby = STBYpin;
  Offset = offset;
  Channel1=channel1_pin;
  Channel2=channel2_pin;

//   pinMode(In1, OUTPUT);
//   pinMode(In2, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  //pinMode(Standby, OUTPUT);

  ledcSetup(Channel1, freq, resolution);
  ledcAttachPin(RPWM, Channel1);
  ledcSetup(Channel2, freq, resolution);
  ledcAttachPin(LPWM, Channel2);

}

void MotorBTS::drive(int speed)
{
  //digitalWrite(Standby, HIGH);
  speed = speed * Offset;
  if (speed>=0) fwd(speed);
  else rev(-speed);
}
void MotorBTS::drive(int speed, int duration)
{
  drive(speed);
  delay(duration);
}

void MotorBTS::fwd(int speed)
{   
//    digitalWrite(In1, HIGH);
//    digitalWrite(In2, LOW);
   ledcWrite(Channel1, speed);
   ledcWrite(Channel2, 0);

}

void MotorBTS::rev(int speed)
{
//    digitalWrite(In1, LOW);
//    digitalWrite(In2, HIGH);
   ledcWrite(Channel1, 0);
   ledcWrite(Channel2, speed);
}

void MotorBTS::brake()
{
//    digitalWrite(In1, HIGH);
//    digitalWrite(In2, HIGH);
   ledcWrite(Channel1, 0);
   ledcWrite(Channel2, 0);
}

// void Motor::standby()
// {
//    digitalWrite(Standby, LOW);
// }

// void forward(Motor motor1, Motor motor2, int speed)
// {
// 	motor1.drive(speed);
// 	motor2.drive(speed);
// }
// void forward(Motor motor1, Motor motor2)
// {
// 	motor1.drive(DEFAULTSPEED);
// 	motor2.drive(DEFAULTSPEED);
// }


// void back(Motor motor1, Motor motor2, int speed)
// {
// 	int temp = abs(speed);
// 	motor1.drive(-temp);
// 	motor2.drive(-temp);
// }
// void back(Motor motor1, Motor motor2)
// {
// 	motor1.drive(-DEFAULTSPEED);
// 	motor2.drive(-DEFAULTSPEED);
// }
// void left(Motor left, Motor right, int speed)
// {
// 	int temp = abs(speed)/2;
// 	left.drive(-temp);
// 	right.drive(temp);

// }


// void right(Motor left, Motor right, int speed)
// {
// 	int temp = abs(speed)/2;
// 	left.drive(temp);
// 	right.drive(-temp);

// }
// void brake(Motor motor1, Motor motor2)
// {
// 	motor1.brake();
// 	motor2.brake();
// }
