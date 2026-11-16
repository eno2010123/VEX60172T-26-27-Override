#include "main.h"
#include <cmath>

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Drivetrain Motors 
pros::Motor frontLeft(-2, pros::v5::MotorGears::green);
pros::Motor backLeft(-12, pros::v5::MotorGears::green);
pros::Motor frontRight(10, pros::v5::MotorGears::green);
pros::Motor backRight(20, pros::v5::MotorGears::green);

pros::MotorGroup LeftDrive({-2, -12}, pros::v5::MotorGears::green);
pros::MotorGroup RightDrive({10, 20}, pros::v5::MotorGears::green);

// Subsystem Motors
pros::Motor cascadeLeft(3, pros::v5::MotorGears::red);
pros::Motor cascadeRight(4, pros::v5::MotorGears::red);
pros::Motor claw(15, pros::v5::MotorGears::green);

// 3-wire initalization
pros::adi::DigitalOut piston1('A');
pros::adi::DigitalOut piston2('B');
pros::adi::DigitalIn pneuOffButton('C');
pros::adi::DigitalIn limitSwitch('D'); 

// State Variables
bool pneuState = false;

// set both pistons
void setPneumatics(bool state) {
    pneuState = state;
    piston1.set_value(pneuState);
    piston2.set_value(pneuState);
}

// timed drive
void driveTimed(int leftPct, int rightPct, int durationMs) {
    LeftDrive.move(leftPct * 1.27);
    RightDrive.move(rightPct * 1.27);
    pros::delay(durationMs);
    LeftDrive.brake();
    RightDrive.brake();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 * VEX code pre auton converted to PROS
 */
void initialize() {
    // Motor stop modes 
    cascadeLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    cascadeRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // full velocity limit for drivetrain motors
    LeftDrive.set_current_limit_all(2500);
    RightDrive.set_current_limit_all(2500);
    cascadeLeft.set_current_limit(2500);
    cascadeRight.set_current_limit(2500);

    // hold cascade motors
    cascadeLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    cascadeRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}