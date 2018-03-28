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

//The port for the light that starts the robot:
unsigned int startPort = 0;

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
void go_to_line() {
}
void follow_line(double Speed, double dt) {
  double pError = 0;
  double Integral = 0;
  while(/*Condition to continue following*/) {
    double error = /*Get error from somewhere*/;
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