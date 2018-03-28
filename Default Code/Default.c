#include <kipr/botball.h>
#include <math.h>
typedef enum { false, true } bool;
double pos[] = {0,0,0};
double PI = 3.141592;


//These constants are required for PID
double kP = 0;
double kI = 0;
double kD = 0;


//Change this variable from false to true if it is the actual competition
bool comp = false;


//Set this variable to false if you are using the Roomba, set this variable to true if you are using the Lego
bool robot = false;

//Change these variables to the ports that the robot is in:

//The port for the light sensor that starts the robot:
unsigned int startPort = 0;

//The port for the right line sensor:
unsigned int rLineSensorPort = 0;

//The port for the left line sensor:
unsigned int lLineSensorPort = 0;

//The distance between the two line sensors:
double lineSensorDist = 0.0;

//If you are using the Lego robot these ports must also be set:

//The robots left wheel port
unsigned int lWheel = 0;

//The robots right wheel port
unsigned int rWheel = 1;


void move_at_power(double lSpeed, double rSpeed) {
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
double PID_control(Error,pError,Integral,dt) {
    double p = kP*Error;
    double i = kI*Integral;
    double d = kD*(Error-pError)/dt
    return p+i+d;
}
double whiteValue = 0;
double blackValue = 0;
void go_to_line() {
}
void follow_line(double Speed, double dt) {
  double pError = 0;
  double Integral = 0;
  while(/*Condition to continue following*/) {
    double lSense = analog(lLineSensorPort);
    double rSense = analog(rLineSensorPort);
    // simulates colors on a quadratic curve where the center of the robot is at 0
    // ax^2+bx+c
    // Finds the color sensors x-coordinate
    // lSpot = -lineSensorDist/2;
    // rSpot = lineSensorDist/2;
    // a(lSpot)^2+b(lSpot)+c=lSense
    // a(rSpot)^2+b(rSpot)+c=rSense
    // a((rSpot)^2-(lSpot)^2)+b((rSpot)-(lSpot))=rSense-lSense
    // d = lineSensorDist
    // a(d/2)^2+b(d/2)-a(-d/2)^2-b(-d/2)=rSense-lSense
    // a(d/2)^2-a(d/2)^2+bd/2+bd/2=rSense-lSense
    // bd=rSense-lSense
    // b=(rSense-lSense)/d
    // a(d/2)^2+((rSense-lSense)/d)(d/2)=rSense
    // a(d/2)^2+(rSense-lSense)/2=rSense
    // a(d/2)^2=rSense/2+lSense/2
    // a=((rSense+lSense)/2)*4/(d^2)
    // a=2*(rSense+lSense)/(d^2)
    // by taking the derivative, the extrema is -b/2a so:
    // error = -b/2a
    // error = -b/(4*(rSense+lSense)/(d^2))
    // error = -b*(d^2)/(4*(rSense+lSense))
    // error = -(rSense-lSense)*d/(4*(rSense+lSense))
    // for this purpose, the calculation is easier if the error is positive. so:
    double error = (rSense-lSense)*lineSensorDist/(4*(rSense+lSense));
    lineSensorDist
    Integral += error*dt;
    double control = PID_control(error,pError,Integral,dt);
    pError = error;
    move_at_power(Speed-control,Speed+control);
    msleep(timeStep);
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