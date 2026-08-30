/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Eno                                                       */
/*    Created:      8/29/2026, 6:09:06 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
motor frontLeft(PORT1, gearSetting::ratio6_1, false);
motor frontRight(PORT2, gearSetting::ratio6_1, true);
motor backLeft(PORT3, gearSetting::ratio6_1, false);
motor backRight(PORT4, gearSetting::ratio6_1, true);
motor lift(PORT5, gearSetting::ratio6_1, false);
motor intake(PORT6, gearSetting::ratio18_1, false);
motor claw(PORT7, gearSetting::ratio18_1, false);
controller Controller1;
motor_group LeftDrive(frontLeft, backLeft);
motor_group RightDrive(frontRight, backRight);






void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}



void autonomous(void) {

}


void usercontrol(void) {
  while (true) {
        // Right joystick UP/DOWN (Forward/Backward)
        int forward = Controller1.Axis2.position();
        // Right joystick LEFT/RIGHT (Turning)
        int turn    = Controller1.Axis1.position();

        // Small deadband to prevent the robot from drifting when you let go of the stick
        if (abs(forward) < 5) forward = 0;
        if (abs(turn) < 5) turn = 0;

        // Calculate final motor speeds
        int leftSpeed  = forward + turn;
        int rightSpeed = forward - turn;

        // Spin the motor groups
        LeftDrive.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::pct);
        RightDrive.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct);

        vex::task::sleep(20);
    }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
