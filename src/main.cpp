/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       enoze                                                     */
/*    Created:      8/20/2026, 2:45:57 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
 
using namespace vex;
 
// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;
 
// define your global instances of motors and other devices here
 
 
int main() {
 
    Brain.Screen.printAt( 10, 50, "Hello V5" );
 
    #include "vex.h"
 
using namespace vex;
 
// Define the 4 individual motors (Port, Gearset, Reversed)
motor LeftFront  = motor(PORT1, ratio18_1, false);
motor LeftBack   = motor(PORT2, ratio18_1, false);
motor RightFront = motor(PORT3, ratio18_1, true); 
motor RightBack  = motor(PORT4, ratio18_1, true);  
 
// Group them into motor groups
motor_group LeftDriveSmart  = motor_group(LeftFront, LeftBack);
motor_group RightDriveSmart = motor_group(RightFront, RightBack);
 
// Combine groups into a Drivetrain object (Groups, WheelTravel, TrackWidth, WheelBase, Unit)
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.2, 320, 240, mm, 1.0);
 
// Define the controller
controller Controller1 = controller(primary);
 
while (true) {
    LeftDriveSmart.spin(forward, Controller1.Axis3.position(), percent);
    RightDriveSmart.spin(forward, Controller1.Axis2.position(), percent);
 
    wait(20, msec);
  }
 
}
