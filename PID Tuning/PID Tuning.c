#include <kipr/botball.h>
#include <math.h>
typedef enum { false, true } bool;
double pos[] = {0,0,0};
double PI = 3.141592653589793;

//Proportion rate of tuning
double dP = 0.2;

//Integral rate of tuning
double dI = 0.2;

//Derivative rate of tuning
double dD = 0.2;

//Set this variable to false if you are using the Roomba, set this variable to true if you are using the Lego robot
bool robot = false;

//Change these variables to the ports that the robot is in:

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
double kP = 1;
double kI = 0;
double kD = 0;
double PID_control(Error,pError,Integral,dt) {
    double p = kP*Error;
    double i = kI*Integral;
    double d = kD*(Error-pError)/dt
    return p+i+d;
}
double whiteValue = 0;
double blackValue = 0;
void go_to_line(double lSpeed, double rSpeed, double dt) {
  whiteValue = (analog(lLineSensorPort)+analog(rLineSensorPort))/2;
  //take code from move_at_power but change the end condition
  blackValue = (analog(lLineSensorPort)+analog(rLineSensorPort))/2;
}
void tune(double Time, double Speed, double dt, double dP, double dI, double dD) {
  double pError = 0;
  double Integral = 0;
  double t = 0;
  while(t<Time) {
    double lSense = analog(lLineSensorPort);
    double rSense = analog(rLineSensorPort);
    double mult = 1;
    if(whiteValue < blackValue) {
      double mult = -1;
    };
    if(mult * blackValue > mult * lSense) {
      blackValue = lSense;
    };
    if(mult * blackValue > mult * rSense) {
      blackValue = rSense;
    };
    double error = lineSensorDist * (rSense + lSense - 2.0 * blackValue) / (2.0 * (rSense - lSense));
    Integral += error*dt;
    double control = PID_control(error,pError,Integral,dt);
    pError = error;
    if(error > 0.15) {
      kP += dP/dt;
      if(kD>0.05 && dresp < 0) kD -= dD/dt;
    };
    if(error < -0.15) {
      kP += dP/dt;
      if(kD>0.05 && dresp > 0) kD -= dD/dt;
    };
    if(dabs(error - pError)/dt<0.1 && dabs(error)>0.07) {
      kI += dI/dt;
    };
    if((pError<0&&error>0)||(pError>0&&error<0)) {
      if(kP>0.2) kP -= dP/dt;
      if(kI>0.2) kI -= dI/dt;
      kD += dD/dt;
    };
    move_at_power_n(Speed-control,Speed+control);
    msleep(1000.0*dt);
    t += dt;
  };
  stop_moving();
  if(kP<0) kP = 0;
  if(kI<0) kI = 0;
  if(kD<0) kD = 0;
  printf("Proportional: %f\n",kP);
  printf("Integral:     %f\n",kI);
  printf("Derivative:   %f\n",kD);
}
int main() {
  if(!robot) {
    create_connect();
  };
  go_to_line(300,300,0.1);
  tune();
  if(!robot) {
    create_disconnect();
  };
  return 0;
}