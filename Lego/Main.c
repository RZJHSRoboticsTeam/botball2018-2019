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
#define PI 3.141592653589793;

#define LIGHT_SENSOR 0
#define L_LINE_SENSOR 1
#define R_LINE_SENSOR 2

#define LINE_DIST 1.1

#define TOUCH_SENSOR 0

//PID Constants

//The Proportion Constant
#define kP 0;

//The Integral Constant
#define kI 0;

//The Derivative Constant
#define kD 0;


//Change this variable from false to true if it is the actual competition
//Enables the line sensor
#define comp false;


//Set this variable to false if you are using the Roomba, set this variable to true if you are using the Lego robot
#define robot true;

//If you are using the Lego robot these ports must also be set:
//The left wheel port
#define lWheel 0;

//The right wheel port
#define rWheel 1;


void move_at_power_n(double lSpeed, double rSpeed) {
  if(robot) {
    motor_power(lWheel,lSpeed);
    motor_power(rWheel,rSpeed);
  } else {
    create_drive_direct(lSpeed, rSpeed);
  };
}
void stop_moving() {
  if(robot) {
    ao();
  } else {
    create_stop();
  };
}


//Reminder: as dt goes down, precision increases
double PID_control(double Error,double pError,double Integral,float dt) {
    double p = kP*Error;
    double i = kI*Integral;
    double d = kD*(Error-pError)/dt
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

  blackValue = (analog(L_LINE_SENSOR)+analog(R_LINE_SENSOR))/2;
}

void follow_line(float Speed, float dist, float dt) {
  double pError = 0.0;
  double Integral = 0.0;
  for(float t = 0.0;t<=dist/Speed;t+=dt) {
    float lSense = analog(L_LINE_SENSOR);
    float rSense = analog(R_LINE_SENSOR);
    if(lSense < blackValue) {
      blackValue = lSense;
    };
    if(rSense < blackValue) {
      blackValue = rSense;
    };
    double error = (lSense<(blackValue+whiteValue)/2) ? 0.5*LINE_DIST : ((lSense<(blackValue+whiteValue)/2) ? -0.5*LINE_DIST : 0.0);
    Integral += error*dt;
    double control = PID_control(error,pError,Integral,dt);
    pError = error;
    move_at_power(Speed*(1.0-control),Speed*(1.0+control));
    msleep(1000.0*dt);
  };
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
