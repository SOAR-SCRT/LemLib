#include "main.h"
#include "lemlib/api.hpp"

// left motor group
pros::MotorGroup left_motor_group({-1, 2, -3}, pros::MotorGears::blue);

// right motor group
pros::MotorGroup right_motor_group({4, -5, 6}, pros::MotorGears::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(
    &left_motor_group,
    &right_motor_group,
    10, // 10 inch track width
    lemlib::Omniwheel::NEW_4, // new 4 inch omnis
    360,
    2
)

// imu
pros::Imu imu(10);

//horizontal tracking wheel encoder
// port of the horizontal encoder
pros::Rotation horizontal_encoder(20);

// vertical tracking wheel encoder
// optical shaft encoder connected to C and D ports
pros::adi::Encoder vertical_encoder("C", 'D', true);

// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);

// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

//odometry settings
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel,
    nullptr,
    &horizontal_tracking_wheel,
    &imu
    );

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
    10, // kP
    0, // kI
    3, //kD
    3, // anti windup
    1, // small error range (inches)
    100, // small error range timeout(ms)
    3, // large error range (inches)
    500, // large error range timeout (ms)
    20 // max acceleration (slew)
)

// angular PID controller
lemlib::ControllerSettings angular_controller(
    2, //kP
    0, // kI
    10, // kD
    3, // anti windup
    1, // small error range
    100, // small error range timeout (ms)
    3, // large error range (inches)
    500, // large error range timeout (ms)
    0 // max acceleration (slew)
)

lemlib::Chassis chassis(
    drivetrain, // drivetrain settings
    lateral_controller, // lateral PID settings
    angular_controller, // angular PID settings
    sensors // odom sensors
    )

    