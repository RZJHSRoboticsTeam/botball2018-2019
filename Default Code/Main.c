//
//  Main.c
//  Botball 2018-2019
//
//  Created by RZJHS Robotics.
//  Copyright © 2018 RZJHS Robotics. All rights reserved.
//

//TODO: Come on guys.  Use branches to keep new/experimental parts of the code from screwing with others.  Eg. a PID line branch and and bang-bang line branch.


#include <kipr/botball.h>
#include <math.h>
//TODO: break some functions off into .h files
typedef enum { false, true } bool;
double pos[] = {0,0,0};
double PI = 3.141592653589793;


//PID Constants

//The Proportion Constant
double kP = 0;

//The Integral Constant
double kI = 0;

//The Derivative Constant
double kD = 0;


//Change this variable from false to true if it is the actual competition
bool comp = false;


//Set this variable to false if you are using the Roomba, set this variable to true if you are using the Lego robot
//TODO: implement auto switching of drive functions
bool robot = false;

//Change these variables to the ports that the robot is in:

//The port for the light sensor that starts the robot:
unsigned int startPort = 0;

//The port for the left line sensor:
unsigned int lLineSensorPort = 0;

//The port for the right line sensor:
unsigned int rLineSensorPort = 0;

//The distance between the two line sensors:
double lineSensorDist = 0.0;

//If you are using the Lego robot these ports must also be set:

//The robots left wheel port
unsigned int lWheel = 0;

//The robots right wheel port
unsigned int rWheel = 1;

//TODO: move.h
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
//TODO: PID.h
double PID_control(Error,pError,Integral,dt) {
    double p = kP*Error;
    double i = kI*Integral;
    double d = kD*(Error-pError)/dt
    return p+i+d;
}
//movement function with PID error correction
void move_at_power(double lSpeed, double rSpeed, double time, double dt) {
  double t = 0;
  double pError = 0;
  double Integral = 0;
  while(t<time) {
    double error = /*Get error*/;
    Integral += error*dt;
    double control = PID_control(error,pError,Integral,dt);
    move_at_power_n(lSpeed-control, rSpeed+control);
    msleep(1000.0*dt);
    t += dt;
    pError = error;
  };
  stop_moving();
}
double whiteValue = 0;
double blackValue = 0;
void go_to_line(double lSpeed, double rSpeed, double dt) {
  whiteValue = (analog(lLineSensorPort)+analog(rLineSensorPort))/2;
  //take code from move_at_power but change the end condition
  blackValue = (analog(lLineSensorPort)+analog(rLineSensorPort))/2;
}
void follow_line(double Speed, double dt) {
  double pError = 0;
  double Integral = 0;
  while(/*Condition to continue following*/) {
    double lSense = analog(lLineSensorPort);
    double rSense = analog(rLineSensorPort);
    if(lSense < blackValue) {
      blackValue = lSense;
    };
    if(rSense < blackValue) {
      blackValue = rSense;
    };
    double error = (dabs(lSense - blackValue)<25.0) ? -0.5*lineSensorDist : ((dabs(rSense - blackValue)<25.0) ? 0.5*lineSensorDist : 0.0);
    Integral += error*dt;
    double control = PID_control(error,pError,Integral,dt);
    pError = error;
    move_at_power_n(Speed-control,Speed+control);
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
