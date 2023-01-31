/**
 * @file lemlib/chassis/chassis.hpp
 * @author Liam Teale
 * @brief Chassis class declarations
 * @version 0.1
 * @date 2023-01-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pose.hpp"

namespace lemlib {
    /**
     * @brief Struct containing all the sensors used for odometry
     * 
     * The sensors are stored in a struct so that they can be easily passed to the chassis class
     * The variables are pointers so that they can be set to nullptr if they are not used
     * Otherwise the chassis class would have to have a constructor for each possible combination of sensors
     *
     * @param vertical1 pointer to the first vertical tracking wheel
     * @param vertical2 pointer to the second vertical tracking wheel
     * @param horizontal1 pointer to the first horizontal tracking wheel
     * @param horizontal2 pointer to the second horizontal tracking wheel
     * @param imu pointer to the IMU
     */
    typedef struct {
        TrackingWheel *vertical1;
        TrackingWheel *vertical2;
        TrackingWheel *horizontal1;
        TrackingWheel *horizontal2;
        pros::Imu *imu;
    } OdomSensors_t;

    /**
     * @brief Struct containing constants for a chassis controller
     *
     * The constants are stored in a struct so that they can be easily passed to the chassis class
     * Set a constant to 0 and it will be ignored
     *
     * @param kA maximum acceleration of the chassis motors in/s^2
     * @param kP proportional constant for the chassis controller
     * @param kD derivative constant for the chassis controller
     * @param smallError the error at which the chassis controller will switch to a slower control loop
     * @param smallErrorTimeout the time the chassis controller will wait before switching to a slower control loop
     * @param largeError the error at which the chassis controller will switch to a faster control loop
     * @param largeErrorTimeout the time the chassis controller will wait before switching to a faster control loop
     */
    typedef struct {
        float kA;
        float kP;
        float kD;
        float smallError; 
        float smallErrorTimeout;
        float largeError;
        float largeErrorTimeout;
    } ChassisController_t;

    /**
     * @brief Chassis class
     * 
     */
    class Chassis {
        public:
            /**
             * @brief Construct a new Chassis
             * 
             * @param leftMotors motors on the left side of the drivetrain
             * @param rightMotors motors on the right side of the drivetrain
             * @param lateralSettings settings for the lateral controller
             * @param angularSetting settings for the angular controller
             * @param topSpeed the top speed of the chassis. in/s
             * @param sensors sensors to be used for odometry
             */
            Chassis(pros::Motor_Group *leftMotors, pros::Motor_Group *rightMotors, float topSpeed, ChassisController_t lateralSettings, ChassisController_t angularSetting, OdomSensors_t sensors);
            /**
             * @brief Calibrate the chassis sensors
             * 
             */
            void calibrate();
            /**
             * @brief Set the pose of the chassis
             * 
             * @param x new x value
             * @param y new y value
             * @param theta new theta value
             * @param radians true if theta is in radians, false if not. False by default
             */
            void setPose(double x, double y, double theta, bool radians = false);
            /**
             * @brief Set the pose of the chassis 
             *
             * @param pose the new pose
             * @param radians whether pose theta is in radians (true) or not (false). false by default
             */
            void setPose(Pose pose, bool radians = false);
            /**
             * @brief Get the pose of the chassis
             * 
             * @param radians whether theta should be in radians (true) or degrees (false). false by default
             * @return Pose 
             */
            Pose getPose(bool radians = false);
            /**
             * @brief Move the chassis as close as possible to the target point in a straight line
             * 
             * @param x x location
             * @param y y location
             * @param timeout longest time the robot can spend moving
             * @param reversed whether the robot should turn in the opposite direction. false by default
             */
            void turnTo(float x, float y, int timeout, bool reversed = false);
            /**
             * @brief Turn the chassis so it is facing the target point
             * 
             * @param x x location
             * @param y y location
             * @param timeout longest time the robot can spend moving
             */
            void moveTo(float x, float y, int timeout);
        private:
            float topSpeed;
            ChassisController_t lateralSettings;
            ChassisController_t angularSettings;
            OdomSensors_t odomSensors;
            pros::Motor_Group *leftMotorGroup;
            pros::Motor_Group *rightMotorGroup;
    };
}