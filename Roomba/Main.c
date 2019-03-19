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

#define CHAIN 0
#define RAISE_CHAIN 2
#define CLAW 3
#define CLAW_WRIST 1

#define L_LINE_SENSOR 0
#define R_LINE_SENSOR 1
#define LIGHT_SENSOR 2

#define TOUCH_SENSOR 0

//PID Constants

//The Proportion Constant
#define kP 2.3

//The Integral Constant
#define kI 1.75

//The Derivative Constant
#define kD 0.01

//Constant for detecting when something is on the white
#define kStDev 25.0

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

#define CHAIN_LIFT 1884
#define CHAIN_LOWER 836
#define CLAW_OPEN 1633
#define CLAW_CLOSE 750
#define CLAW_CLICK 750
#define WRIST_HORIZONTAL 650
#define WRIST_VERTICAL 1809
void lift_chain() {
  set_servo_position(RAISE_CHAIN,CHAIN_LIFT);
};

void lower_chain() {
  set_servo_position(RAISE_CHAIN,CHAIN_LOWER);
};

void wrist_horizontal() {
  set_servo_position(CLAW_WRIST,WRIST_HORIZONTAL);
};

void wrist_vertical() {
  set_servo_position(CLAW_WRIST,WRIST_VERTICAL);
};

void open_claw() {
  set_servo_position(CLAW,CLAW_OPEN);
};

void close_claw() {
  set_servo_position(CLAW,CLAW_CLOSE);
};

bool close_claw_until_button() {
  open_claw();
  double pos;
  for(pos = 0.0;pos<1.0;pos+=0.01) {
    set_servo_position(CLAW,CLAW_OPEN+(CLAW_CLICK-CLAW_OPEN)*pos);
    if(digital(TOUCH_SENSOR)) {
      return true;
    };
    msleep(20);
  };
  msleep(100);
  return digital(TOUCH_SENSOR);
};

void spin_chain(double distance, double speed) {
    motor_power(CHAIN,speed);
    msleep(dabs(distance/speed));
    motor_power(CHAIN,0);
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

double diff = 0.0;
int whiteValueL = 0;
int blackValueL = 0;
int whiteValueR = 0;
int blackValueR = 0;
double stDevL = 0.0;
double stDevR = 0.0;
//TODO: Test this function 
void go_to_line(float lSpeed, float rSpeed, float dt) {
  double mL = analog(L_LINE_SENSOR);
  whiteValueL = mL;
  double mR = analog(R_LINE_SENSOR);
  whiteValueR = mR;
  move_at_power(lSpeed,rSpeed);
  float t = dt;
  stDevL = 0.0;
  stDevR = 0.0;
  msleep(1000.0*dt);
  while(t <= 0.1) {
    double sqNumL = (analog(L_LINE_SENSOR)-whiteValueL);
    double sqNumR = (analog(R_LINE_SENSOR)-whiteValueR);
    stDevL = (stDevL*(t-dt)+dt*sqNumL*sqNumL)/t;
    stDevR = (stDevR*(t-dt)+dt*sqNumR*sqNumR)/t;
    msleep(1000.0*dt);
    t += dt;
  };
  stDevL = sqrt(stDevL);
  stDevR = sqrt(stDevR);
  while(dabs(analog(L_LINE_SENSOR)-mL)<=kStDev*stDevL&&dabs(analog(R_LINE_SENSOR)-mR)<=kStDev*stDevR) {
    msleep(1000.0*dt);
    t += dt;
  };
  blackValueL = analog(L_LINE_SENSOR);
  blackValueR = analog(R_LINE_SENSOR);
}

void go_to_line_perpendicular(float lSpeed, float rSpeed, float dt) {
  double mL = analog(L_LINE_SENSOR);
  whiteValueL = mL;
  double mR = analog(R_LINE_SENSOR);
  whiteValueR = mR;
  move_at_power(lSpeed,rSpeed);
  float t = dt;
  stDevL = 0.0;
  stDevR = 0.0;
  msleep(1000.0*dt);
  while(t <= 0.1) {
    double sqNumL = (analog(L_LINE_SENSOR)-whiteValueL);
    double sqNumR = (analog(R_LINE_SENSOR)-whiteValueR);
    stDevL = (stDevL*(t-dt)+dt*sqNumL*sqNumL)/t;
    stDevR = (stDevR*(t-dt)+dt*sqNumR*sqNumR)/t;
    msleep(1000.0*dt);
    t += dt;
  };
  stDevL = sqrt(stDevL);
  stDevR = sqrt(stDevR);
  bool right = true

  bool left = true;
  while(right || left) {
    if(dabs(analog(L_LINE_SENSOR)-mL)>kStDev*stDevL) {
      stop_moving_left();
      left = false;
    };
    if(dabs(analog(R_LINE_SENSOR)-mR)>kStDev*stDevR) {
      stop_moving_right();
      right = false;
    };
    msleep(1000.0*dt);
    t += dt;
  };
  blackValueL = analog(L_LINE_SENSOR);
  blackValueR = analog(R_LINE_SENSOR);
}

void follow_line(float Speed, float dist, float dt) {
  double pError = 0.0;
  double Integral = 0.0;
  float t = 0.0;
  for(t = 0.0;t<=dist/Speed;t+=dt) {
    float lSense = analog(L_LINE_SENSOR);
    float rSense = analog(R_LINE_SENSOR);
    if(lSense > blackValueL) {
      blackValueL = lSense;
    };
    if(rSense > blackValueR) {
      blackValueR = rSense;
    };
    double error = (analog(L_LINE_SENSOR)-analog(R_LINE_SENSOR)-diff)/4095.0;
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
  diff = analog(L_LINE_SENSOR)-analog(R_LINE_SENSOR);
  if(!robot) {
    create_connect();
  };
  enable_servos();
  if(comp) {
    wait_for_light(LIGHT_SENSOR);
  };
  code();
  disable_servos();
  if(!robot) {
    create_disconnect();
  };
  return 0;
}