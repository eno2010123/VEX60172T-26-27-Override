#include "main.h"
#include <string>
#include "lemlib/api.hpp"

#include <cmath>

pros::Controller master(pros::E_CONTROLLER_MASTER);
//Defines all ports and rotation for the entire robot.
pros::MotorGroup left_motors({12, -11}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({19, -20}, pros::MotorGearset::blue);
pros::MotorGroup dr4b_motors({-1, 9}, pros::MotorGearset::red);
pros::Optical color_sensor(7);

pros::Motor color_motor(8);
// The 5.5W motor has no cartridge color/gearset to configure.
pros::Motor claw_motor(10);

// The IMU needs its own smart-port slot; keep it off the optical sensor's port.
pros::Imu imu(5);
pros::adi::Encoder vertical_encoder('C', 'D', true);

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_325, -2.5);

lemlib::OdomSensors sensors(
&vertical_tracking_wheel,
nullptr,
nullptr,
nullptr,
&imu
);

lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

// LemLib chassis PID starting settings: distance control, then heading control.
// Tune these gains on the robot; good values depend on its drivetrain and tracking.
lemlib::ControllerSettings lateral_controller(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller(2, 0, 10, 3, 1, 100, 3, 500, 0);

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 10, lemlib::Omniwheel::NEW_325, 600, 2);

lemlib::Chassis chassis(
drivetrain,
lateral_controller,
angular_controller,
sensors,
&throttle_curve,
&steer_curve
);

// Claw (manual only): positive power = open, negative = close.
constexpr int clawOpenPower = 64;

// ---------------------------------------------------------------------------
// FIELD / ROBOT GEOMETRY (inches).
// Origin = field center, +x right, +y up-field, heading 0 = +x,
// and positive heading is counterclockwise (LemLib's frame).
// Source: Override manual v2.1 -- 140.4 in inside
// wall-to-wall, 6x6 tiles, so one tile = 23.4 in. The Midfield is a 2x2-tile
// diamond whose tape-centerline points are one tile from center.
// ---------------------------------------------------------------------------
namespace geo {
constexpr double FIELD_INSIDE = 140.4;
constexpr double WALL = FIELD_INSIDE / 2.0; // 70.2
constexpr double TILE = FIELD_INSIDE / 6.0; // 23.4 tile pitch

// The Midfield is bounded by the INNER tape edge. Tape width ASSUMED 2.5 in
// (the manual only lists 2.5 in for the Autonomous Line) -- verify on your field.
constexpr double TAPE_WIDTH = 2.5;

constexpr double ROBOT_LEN = 18.0;
constexpr double ROBOT_WID = 18.0;
// Tracking center -> FRONT edge (the claw end). 9.0 assumes the wheels are
// centered on an 18 in robot. MEASURE THIS.
constexpr double FRONT_OFFSET = 9.0;
constexpr double REAR_OFFSET = ROBOT_LEN - FRONT_OFFSET;
} // namespace geo

void drawMiniField(const lemlib::Pose& pose) {
constexpr int fieldLeft = 20;
constexpr int fieldTop = 25;
constexpr int fieldRight = 210;
constexpr int fieldBottom = 215;
constexpr double pixelsPerInch = 190.0 / 144.0;
constexpr double pi = 3.14159265358979323846;

pros::screen::set_eraser(pros::Color::black);
pros::screen::erase();

pros::screen::set_pen(pros::Color::dark_green);
pros::screen::fill_rect(fieldLeft, fieldTop, fieldRight, fieldBottom);
pros::screen::set_pen(pros::Color::white);
pros::screen::draw_rect(fieldLeft, fieldTop, fieldRight, fieldBottom);
pros::screen::draw_line(115, fieldTop, 115, fieldBottom);
pros::screen::draw_line(fieldLeft, 120, fieldRight, 120);
pros::screen::draw_circle(115, 120, 14);

int robotX = static_cast<int>(115 + pose.x * pixelsPerInch);
int robotY = static_cast<int>(120 - pose.y * pixelsPerInch);
if (robotX < fieldLeft + 8) robotX = fieldLeft + 8;
if (robotX > fieldRight - 8) robotX = fieldRight - 8;
if (robotY < fieldTop + 8) robotY = fieldTop + 8;
if (robotY > fieldBottom - 8) robotY = fieldBottom - 8;

pros::screen::set_pen(pros::Color::blue);
pros::screen::fill_rect(robotX - 7, robotY - 7, robotX + 7, robotY + 7);
double heading = pose.theta * pi / 180.0;
pros::screen::set_pen(pros::Color::yellow);
pros::screen::draw_line(
robotX,
robotY,
static_cast<int>(robotX + std::sin(heading) * 13),
static_cast<int>(robotY - std::cos(heading) * 13)
);

pros::screen::set_pen(pros::Color::white);
pros::screen::print(pros::E_TEXT_MEDIUM, 235, 35, "MINI FIELD");
pros::screen::print(pros::E_TEXT_SMALL, 235, 70, "X: %.1f in", pose.x);
pros::screen::print(pros::E_TEXT_SMALL, 235, 95, "Y: %.1f in", pose.y);
pros::screen::print(pros::E_TEXT_SMALL, 235, 120, "H: %.1f deg", pose.theta);
}

void initialize() {
dr4b_motors.tare_position();
dr4b_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
claw_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
color_sensor.set_led_pwm(100);

// Needed so the autonomous can hold a straight heading with the IMU.
// Robot must sit still while it runs. If the IMU isn't plugged in yet,
// comment this line out.
chassis.calibrate();

// Draws a top-down field and follows the robot's odometry pose.
pros::Task screen_task([]() {
while (true) {
lemlib::Pose pose = chassis.getPose();
drawMiniField(pose);
pros::delay(100);
}
});
}

void disabled() {
left_motors.move_velocity(0);
right_motors.move_velocity(0);
dr4b_motors.move_velocity(0);
claw_motor.move(0);
color_motor.move(0);
}

void competition_initialize() {}

// ---------------------------------------------------------------------------
// TOGGLE + PARK IN THE MIDFIELD (straight moves only, no turns)
//
// SETUP ASSUMED: robot starts on the field's RIGHT wall, claw end touching the
// wall, facing it (heading 90), centered on that wall's toggle.
//
// 1. Twice: drive straight back (arm down), then charge straight forward at
// full speed into the toggle (arm up).
// 2. Then drive straight back, all the way, until the rear end is inside the
// Midfield. Claw is never touched.
//
// Distances are open-loop: inches -> time from wheel radius and motor rpm.
// Every straight move holds the IMU heading, so it cannot veer.
// ---------------------------------------------------------------------------
void autonomous() {
// ---- Drivetrain motion settings ----
constexpr int frontRpm = 320; // front motor rpm, blue max is 600
constexpr int backRpm = 320; // motor rpm, blue max is 600
constexpr int chargeRpm = 600; // full speed into the toggle
constexpr int parkRpm = 400; // final run backward into the Midfield
constexpr double headingKp = 4.0; // rpm of correction per degree of heading error

// ---- Toggle cycles ----
// Long run-up so the robot is at full speed when it hits the toggle.
constexpr int cycles = 2;
constexpr double backDistance = 24.0; // inches straight back from the wall
constexpr double chargeDistance = 30.0; // inches straight forward (> backDistance so
// it reaches the wall at full speed)
constexpr int reversePauseMs = 100;

// ---- Arm (used for the toggle only) ----
constexpr int armUpPosition = 2000;
constexpr int armDownPosition = 0;
constexpr int armVelocity = 100; // rpm, max for the red gearset
constexpr int armWaitCapMs = 1000;

// ---- Start and park geometry (inches) ----
constexpr double startHeading = 0.0; // facing the right wall (+x)
constexpr double startOffsetLeft = 0.0; // robot center, toward its LEFT of the toggle
// center. 0 = centered on the toggle.
// NOTE: at 0 the rear end drives straight into
// the cup that starts on the Midfield point
// (23.4, 0) and pushes it. Use +-11.7 to go
// around it (needs |offset| >= 10.6).
constexpr double parkDepth = 5.0; // rear edge ends this far past the tape's inner edge

// Rear edge position when it just touches the Midfield's inner tape edge.
// The part of the rear edge closest to the field's center line enters first.
const double D_INNER = geo::TILE - (geo::TAPE_WIDTH / 2.0) * std::sqrt(2.0); // ~21.63
const double yNearest = std::fmax(0.0, std::fabs(startOffsetLeft) - geo::ROBOT_WID / 2.0);
const double rearAtTape = D_INNER - yNearest; // x of rear edge at the tape
const double centerTargetX = (rearAtTape - parkDepth) + geo::REAR_OFFSET; // tracking center
const double startX = geo::WALL - geo::FRONT_OFFSET; // claw end on the wall
const double parkTravel = std::fmax(0.0, startX - centerTargetX); // ~35.6 in straight back

// Drive 'inches' in a straight line, open loop. Time comes from wheel speed:
// in/sec = (motor rpm * gearRatio / 60) * 2 * pi * wheelRadius
// The IMU heading is held at holdHeading while driving.
auto driveInches = [&](double inches, int rpm, double holdHeading) {
const int dir = inches >= 0 ? 1 : -1;
const double startX = chassis.getPose().x;
const double targetX = startX + inches;
const uint32_t start = pros::millis();
constexpr uint32_t timeoutMs = 5000;
constexpr double positionTolerance = 0.5;

while (pros::millis() - start < timeoutMs &&
       std::fabs(chassis.getPose().x - targetX) > positionTolerance) {
lemlib::Pose pose = chassis.getPose();
double err = holdHeading - pose.theta;
while (err > 180.0) err -= 360.0;
while (err < -180.0) err += 360.0;
const double corr = headingKp * err;
double l = dir * rpm - corr;
double r = dir * rpm + corr;
l = std::fmax(-600.0, std::fmin(600.0, l));
r = std::fmax(-600.0, std::fmin(600.0, r));
left_motors.move_velocity(static_cast<int>(l));
right_motors.move_velocity(static_cast<int>(r));
pros::delay(10);
}
left_motors.move_velocity(0);
right_motors.move_velocity(0);
};

// Arm: start a move, then optionally wait for the time it should take.
uint32_t armMoveStart = 0;
int armEtaMs = 0;
int armPos = armDownPosition;
auto moveArm = [&](int target) {
armEtaMs = static_cast<int>(std::fabs(static_cast<double>(target - armPos)) / (armVelocity * 6.0) * 1000.0);
armMoveStart = pros::millis();
armPos = target;
dr4b_motors.move_absolute(target, armVelocity);
};
auto waitForArm = [&]() {
int remaining = armEtaMs - static_cast<int>(pros::millis() - armMoveStart);
if (remaining > armWaitCapMs) remaining = armWaitCapMs;
if (remaining > 0) pros::delay(remaining);
};

dr4b_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // arm stays where it stops

// Real field position so the mini-field screen matches (right wall, claw at wall).
chassis.setPose(startX, startOffsetLeft, startHeading);

// ---- 1) Toggle: straight back (arm down), straight charge (arm up), twice ----
for (int i = 0; i < cycles; i++) {
moveArm(armDownPosition);
driveInches(-backDistance, backRpm, startHeading);
waitForArm();
pros::delay(reversePauseMs);

moveArm(armUpPosition);
driveInches(chargeDistance, chargeRpm, startHeading);
waitForArm();
}

// ---- 2) Straight back, all the way into the Midfield (arm back down) ----
moveArm(armDownPosition);
driveInches(-parkTravel, parkRpm, startHeading);
waitForArm();
}

void opcontrol() {
while (true) {
int proximity = color_sensor.get_proximity();
double hue = color_sensor.get_hue();
bool yellow_detected = proximity > 100 && hue >= 40 && hue <= 70;

color_motor.move(yellow_detected ? 127 : 0);

int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
chassis.arcade(forward, turn);

if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
dr4b_motors.move(127);
} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
dr4b_motors.move(-127);
} else {
dr4b_motors.move(0);
}

// Manual claw only (no distance sensor, so it never opens by itself).
if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
claw_motor.move(clawOpenPower);
} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
claw_motor.move(-clawOpenPower);
} else {
claw_motor.move(0);
}

pros::delay(20);
}
}
