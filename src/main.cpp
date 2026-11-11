#include "main.h"
#include <cmath>

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Drivetrain Motors (negative port numbers reverse the motor in PROS 4)
pros::Motor frontLeft(-10, pros::v5::MotorGears::blue);
pros::Motor backLeft(-21, pros::v5::MotorGears::blue);
pros::Motor frontRight(3, pros::v5::MotorGears::blue);
pros::Motor backRight(13, pros::v5::MotorGears::blue);

pros::MotorGroup LeftDrive({-10, -21}, pros::v5::MotorGears::blue);
pros::MotorGroup RightDrive({3, 13}, pros::v5::MotorGears::blue);

// Subsystem Motors
pros::Motor lift(19, pros::v5::MotorGears::red);
pros::Motor arm(4, pros::v5::MotorGears::red);
pros::Motor intake(20, pros::v5::MotorGears::red);
pros::Motor claw(6, pros::v5::MotorGears::green);

// Three-Wire ADI Devices (Pneumatics & Bumper)
pros::adi::DigitalOut piston1('A');
pros::adi::DigitalOut piston2('B');
pros::adi::DigitalIn pneuOffButton('C');

// State Variables
bool pneuState = false;
bool intakeRunning = false;

// Helper: Set both pistons
void setPneumatics(bool state) {
    pneuState = state;
    piston1.set_value(pneuState);
    piston2.set_value(pneuState);
}

// Helper: Timed drive (converts % to PROS voltage scale [-127, 127])
void driveTimed(int leftPct, int rightPct, int durationMs) {
    LeftDrive.move(leftPct * 1.27);
    RightDrive.move(rightPct * 1.27);
    pros::delay(durationMs);
    LeftDrive.brake();
    RightDrive.brake();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 * Replaces VEXcode's pre_auton().
 */
void initialize() {
    // Motor stop modes
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // Full current limit (2500mA = 100% torque in V5)
    LeftDrive.set_current_limit_all(2500);
    RightDrive.set_current_limit_all(2500);
    arm.set_current_limit(2500);

    // Engage hold on arm and lift
    arm.brake();
    lift.brake();

    // Start pistons retracted/off
    setPneumatics(false);
}

void disabled() {}

void competition_initialize() {}

/**
 * Runs the user autonomous code.
 */
void autonomous() {
    LeftDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    RightDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    arm.set_current_limit(2500);

    // Grab preload and lift arm (36:1 red gearset max RPM is 100, 18:1 green is 200)
    claw.move_velocity(200 * 0.80); // 80% of 200 RPM
    arm.move_velocity(100 * 0.75);  // 75% of 100 RPM
    pros::delay(500);
    arm.brake();
    claw.brake();
    pros::delay(300);

    // Drive forward and intake
    intake.move_velocity(100);      // 100% of 100 RPM
    driveTimed(45, 45, 1500);
    pros::delay(200);

    // Release preload
    claw.move_velocity(-200 * 0.80);
    intake.move_velocity(-100);
    pros::delay(600);
    claw.brake();
    pros::delay(400);
    intake.brake();

    // Back up
    driveTimed(-35, -35, 600);

    LeftDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    RightDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}

/**
 * Runs the operator control code.
 * Replaces VEXcode's usercontrol().
 */
void opcontrol() {
    LeftDrive.set_current_limit_all(2500);
    RightDrive.set_current_limit_all(2500);
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    arm.set_current_limit(2500);
    arm.brake();

    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    const int ARM_SPEED_PCT = 85;

    while (true) {
        // Split arcade drive (PROS analog sticks return [-127, 127])
        // Scaled to [-100, 100] to preserve your exact deadband and millivolt math
        int forward = -(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 100 / 127);
        int turn    =  (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * 100 / 127);

        // Stick deadband
        if (std::abs(forward) < 5) forward = 0;
        if (std::abs(turn) < 5) turn = 0;

        int leftSpeed  = forward + turn;
        int rightSpeed = forward - turn;

        // Clamp to [-100, 100]
        if (leftSpeed > 100) leftSpeed = 100;
        if (leftSpeed < -100) leftSpeed = -100;
        if (rightSpeed > 100) rightSpeed = 100;
        if (rightSpeed < -100) rightSpeed = -100;

        // move_voltage takes millivolts [-12000, 12000] (100 * 120 = 12000 mV = 12V)
        LeftDrive.move_voltage(leftSpeed * 120);
        RightDrive.move_voltage(rightSpeed * 120);

        // Cascade lift (100 RPM red cartridge)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            lift.move_velocity(100);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            lift.move_velocity(-100);
        } else {
            lift.brake();
        }

        // Claw arm (85% of 100 RPM = 85 RPM)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            arm.move_velocity(ARM_SPEED_PCT);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            arm.move_velocity(-ARM_SPEED_PCT);
        } else {
            arm.brake();
        }

        // Claw (80% of 200 RPM green cartridge = 160 RPM)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            claw.move_velocity(160);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            claw.move_velocity(-160);
        } else {
            claw.brake();
        }

        // Continuous intake toggle on Up (get_digital_new_press handles latching automatically)
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            intakeRunning = !intakeRunning;
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            intake.move_velocity(-100);
        } else if (intakeRunning) {
            intake.move_velocity(100);
        } else {
            intake.brake();
        }

        // Pneumatics toggle and cutoff switch
        if (pneuOffButton.get_value() || master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            setPneumatics(false);
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            setPneumatics(!pneuState);
        }

        pros::delay(20);
    }
}