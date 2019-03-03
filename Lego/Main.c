//
//  Main.c
//  Botball 2018-2019
//
//  Created by RZJHS Robotics.
//  Copyright © 2019 RZJHS Robotics. All rights reserved.
//
#include <kipr/botball.h>
#include <math.h>
typedef enum { false, true } bool;
double pos[] = {0,0,0};
#define PI 3.141592653589793

/*
 * Motor Ports: 0 - Pulling Chain, 1 - Chain, 2 - Claw Wrist, 3 - Claw
 * Analog Ports: 0 - Light sensor, 1 - Left Line sensor, 2 - Right Line sensor
 * Digital Ports: 0 - Touch sensor
 */

#define CHAIN_PULL 0
#define CHAIN 1
#define CLAW_WRIST 2
#define CLAW 3

#define LIGHT_SENSOR 0
#define L_LINE_SENSOR 1
#define R_LINE_SENSOR 2

#define LINE_DIST 1.1

#define TOUCH_SENSOR 0

//PID Constants

//The Proportion Constant
#define kP 0

//The Integral Constant
#define kI 0

//The Derivative Constant
#define kD 0


//Change this variable from false to true if it is the actual competition
//Enables the line sensor
#define comp false


//Set this variable to false if you are using the Roomba, set this variable to true if you are using the Lego robot
#define robot false

//If you are using the Lego robot these ports must also be set:
//The left wheel port
#define lWheel 0

//The right wheel port
#define rWheel 1

double roombaLeftSpeed = 0.0;
double roombaRightSpeed = 0.0;

double dabs(double x) {
  return x>=0?x:-x;
};

void move_at_power(double lSpeed, double rSpeed) {
  if(robot) {
    motor_power(lWheel,lSpeed);
    motor_power(rWheel,rSpeed);
  } else {
    roombaLeftSpeed = lSpeed;
    roombaRightSpeed = rSpeed;
    create_drive_direct(lSpeed, rSpeed);
  };
}

void stop_moving() {
  if(robot) {
    ao();
  } else {
    roombaLeftSpeed = 0.0;
    roombaRightSpeed = 0.0;
    create_stop();
  };
}

void stop_moving_left() {
  if(robot) {
    motor_power(lWheel,0);
  } else {
    roombaLeftSpeed = 0.0;
    create_drive_direct(0,roombaRightSpeed);
  };
}

void stop_moving_right() {
  if(robot) {
    motor_power(rWheel,0);
  } else {
    roombaRightSpeed = 0.0;
    create_drive_direct(roombaLeftSpeed,0);
  };
}


//Reminder: as dt goes down, precision increases
double PID_control(double Error,double pError,double Integral,float dt) {
    double p = kP*Error;
    double i = kI*Integral;
    double d = kD*(Error-pError)/dt;
    return p+i+d;
}


int whiteValue = 0;
int blackValue = 0;
//TODO: Test this function 
void go_to_line(float lSpeed, float rSpeed, float dt) {
  whiteValue = 0.5*(analog(L_LINE_SENSOR)+analog(R_LINE_SENSOR));
  move_at_power(lSpeed,rSpeed);
  float t = 0.0;
  double stDev = 0.0;
  double m = whiteValue;
  while(t <= 0.1) {
    stDev = (stDev*(t-dt)+dt*0.5*(analog(L_LINE_SENSOR)+analog(R_LINE_SENSOR)))/t;
    msleep(1000.0*dt);
    t += dt;
  };
  while(dabs(0.5*(analog(L_LINE_SENSOR)+analog(R_LINE_SENSOR))-m)<=stDev*2.0) {
    msleep(1000.0*dt);
    t += dt;
  };
  stop_moving();
  blackValue = (analog(L_LINE_SENSOR)+analog(R_LINE_SENSOR))/2;
}

void go_to_line_perpendicular(float lSpeed, float rSpeed, float dt) {
  whiteValue = 0.5*(analog(L_LINE_SENSOR)+analog(R_LINE_SENSOR));
  move_at_power(lSpeed,rSpeed);
  float t = 0.0;
  double stDev = 0.0;
  double m = whiteValue;
  while(t <= 0.1) {
    stDev = (stDev*(t-dt)+dt*0.5*(analog(L_LINE_SENSOR)+analog(R_LINE_SENSOR)))/t;
    msleep(1000.0*dt);
    t += dt;
  };
  bool right = true;
  bool left = true;
  while(right || left) {
    if(dabs(analog(L_LINE_SENSOR)-m)<=stDev*2.0) {
      stop_moving_left();
      left = false;
    };
    if(dabs(analog(R_LINE_SENSOR)-m)<=stDev*2.0) {
      stop_moving_right();
      right = false;
    };
    msleep(1000.0*dt);
    t += dt;
  };
  blackValue = (analog(L_LINE_SENSOR)+analog(R_LINE_SENSOR))/2;
}

void follow_line(float Speed, float dist, float dt) {
  double pError = 0.0;
  double Integral = 0.0;
  float t = 0.0;
  for(t = 0.0;t<=dist/Speed;t+=dt) {
    float lSense = analog(L_LINE_SENSOR);
    float rSense = analog(R_LINE_SENSOR);
    if(lSense < blackValue) { //check which color gives higher values
      blackValue = lSense;
    };
    if(rSense < blackValue) {
      blackValue = rSense;
    };
    double error = (lSense>(blackValue+whiteValue)/2) ? 0.5*LINE_DIST : ((rSense>(blackValue+whiteValue)/2) ? -0.5*LINE_DIST : 0.0);
    Integral += error*dt;
    double control = PID_control(error,pError,Integral,dt);
    pError = error;
    move_at_power(Speed*(1.0-control),Speed*(1.0+control));
    msleep(1000.0*dt);
  };
  stop_moving();
}


void code() {
  //Put your code here
}


int main() {
  if(!robot) {
    create_connect();
  };
  enable_servos();
  if(comp) {
    wait_for_light(startPort);
  };
  code();
  disable_servos();
  if(!robot) {
    create_disconnect();
  };
  return 0;
}
