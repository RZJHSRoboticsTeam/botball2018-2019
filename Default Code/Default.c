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

//The robots left weel port
unsigned int lWeel = 0;

//The robots right weel port
unsigned int rWeel = 1;


void move_at_power(double lSpeed, double rSpeed) {
  if(robot) {
    motor_power(lWeel,lSpeed);
    motor_power(rWeel,rSpeed);
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
void go_to_line() {
}
void follow_line() {
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