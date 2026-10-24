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

competition Competition;
brain Brain;
controller Controller1;

// drivetrain motors
motor frontLeft(PORT21, gearSetting::ratio6_1, true);
motor backLeft(PORT10, gearSetting::ratio6_1, true);
motor frontRight(PORT13, gearSetting::ratio6_1, false);
motor backRight(PORT3, gearSetting::ratio6_1, false);

motor_group LeftDrive(frontLeft, backLeft);
motor_group RightDrive(frontRight, backRight);

// subsystem motors
motor lift(PORT19, gearSetting::ratio36_1, false);
motor arm(PORT4, gearSetting::ratio36_1, false);
intake(PORT20, gearSetting::ratio36_1, false);
motor claw(PORT6, gearSetting::ratio18_1, false);

// pneumatics and button
digital_out piston1(Brain.ThreeWirePort.A);
digital_out piston2(Brain.ThreeWirePort.B);
bumper pneuOffButton(Brain.ThreeWirePort.C);

bool pneuState = false;
bool buttonALatch = false;

// set both pistons
void setPneumatics(bool state) {
  pneuState = state;
  piston1.set(pneuState);
  piston2.set(pneuState);
}

// timed drive helper
void driveTimed(int leftPct, int rightPct, int durationMs) {
  LeftDrive.spin(fwd, leftPct, pct);
  RightDrive.spin(fwd, rightPct, pct);
  wait(durationMs, msec);
  LeftDrive.stop(brake);
  RightDrive.stop(brake);
}

void pre_auton(void) {
  // motor stop modes
  lift.setStopping(hold);
  arm.setStopping(hold);
  claw.setStopping(hold);
  intake.setStopping(coast);

  // limit arm power and lock position
  arm.setMaxTorque(65, pct);
  arm.stop(hold);
  lift.stop(hold);
  
  // start pistons off
  setPneumatics(false);
}

void autonomous(void) {
  LeftDrive.setStopping(brake);
  RightDrive.setStopping(brake);
  arm.setStopping(hold);
  arm.setMaxTorque(65, pct);

  // grab preload and lift arm
  claw.spin(fwd, 80, pct);
  arm.spin(fwd, 30, pct);
  wait(500, msec);
  arm.stop(hold);
  claw.stop(hold);
  wait(300, msec);

  // drive forward and intake
  intake.spin(fwd, 100, pct);
  driveTimed(45, 45, 1500);
  wait(200, msec);

  // release preload
  claw.spin(reverse, 80, pct);
  intake.spin(reverse, 100, pct);
  wait(600, msec);
  claw.stop(hold);
  wait(400, msec);
  intake.stop(coast);

  // back up
  driveTimed(-35, -35, 600);

  LeftDrive.setStopping(coast);
  RightDrive.setStopping(coast);
}

void usercontrol(void) {
  // lock arm and lift on start
  arm.setStopping(hold);
  arm.setMaxTorque(65, pct);
  arm.stop(hold);

  lift.setStopping(hold);
  claw.setStopping(hold);

  const int ARM_SPEED = 35;

  while (true) {
    // split arcade drive
    int forward = Controller1.Axis3.position();
    int turn    = Controller1.Axis1.position();

    // stick deadband
    if (abs(forward) < 5) forward = 0;
    if (abs(turn) < 5) turn = 0;

    int leftSpeed  = forward + turn;
    int rightSpeed = forward - turn;

    LeftDrive.spin(fwd, leftSpeed, pct);
    RightDrive.spin(fwd, rightSpeed, pct);

    // cascade lift
    if (Controller1.ButtonL1.pressing()) {
      lift.spin(fwd, 100, pct);
    } else if (Controller1.ButtonL2.pressing()) {
      lift.spin(reverse, 100, pct);
    } else {
      lift.stop(hold);
    }

    // claw arm
    if (Controller1.ButtonR1.pressing()) {
      arm.spin(fwd, ARM_SPEED, pct);
    } else if (Controller1.ButtonR2.pressing()) {
      arm.spin(reverse, ARM_SPEED, pct);
    } else {
      arm.stop(hold);
    }

    // claw
    if (Controller1.ButtonX.pressing()) {
      claw.spin(fwd, 80, pct);
    } else if (Controller1.ButtonB.pressing()) {
      claw.spin(reverse, 80, pct);
    } else {
      claw.stop(hold);
    }

    // intake
    if (Controller1.ButtonUp.pressing()) {
      intake.spin(fwd, 100, pct);
    } else if (Controller1.ButtonDown.pressing()) {
      intake.spin(reverse, 100, pct);
    } else {
      intake.stop(coast);
    }

    // pneumatics toggle and cutoff switch
    if (pneuOffButton.pressing() || Controller1.ButtonLeft.pressing()) {
      setPneumatics(false);
    } else if (Controller1.ButtonA.pressing()) {
      if (!buttonALatch) {
        setPneumatics(!pneuState);
        buttonALatch = true;
      }
    } else {
      buttonALatch = false;
    }

    wait(20, msec);
  }
}

int main() {
  // competition callbacks
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}