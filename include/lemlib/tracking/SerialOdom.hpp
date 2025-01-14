#pragma once

#include "hardware/Encoder/Encoder.hpp"
#include "hardware/Port.hpp"
#include "hardware/IMU/IMU.hpp"
#include "pros/rtos.hpp"
#include "units/Pose.hpp"
#include <vector>

namespace lemlib {
/**
 * @brief class representing a tracking wheel
 *
 */

/**
 * @brief Tracking Wheel Odometry class
 *
 * This class can be used to track the position a differential drive robot,
 * which has any number of tracking wheels and IMUs, including 0
 */

class SerialOdometry {
    public:
        /**
         * @brief Construct a new Serial Odometry object to read odometry data from ESP32
         *
         * @param port Smart Port for serial to ESP32
         *
         */
        SerialOdometry(ReversibleSmartPort port);
        /**
         * @brief Get the estimated Pose of the robot
         *
         * @return units::Pose the estimated pose
         *
         * @b Example:
         * // create TrackingWheelOdom object
         * lemlib::TrackingWheelOdom odom(...);
         *
         * @code {.cpp}
         * void initialize() {
         *   pros::lcd::initialize(); // initialize brain screen
         *   odom.calibrate(); // calibrate odom
         *   while (true) {
         *     // get current pose of the robot
         *     const lemlib::Pose pose = odom.getPose();
         *     // print pros to the brain screen
         *     pros::lcd::print(0, "x: %f", to_in(pose.x));
         *     pros::lcd::print(1, "y: %f", to_in(pose.y));
         *     pros::lcd::print(2, "theta: %f", to_in(pose.theta));
         *     // delay to let other tasks run
         *     pros::delay(10);
         *   }
         * }
         * @endcode
         */
        units::Pose getPose();
        /**
         * @brief Set the estimated pose of the robot
         *
         * @param pose the new pose
         *
         * @b Example:
         * @code {.cpp}
         * // create TrackingWheelOdom object
         * lemlib::TrackingWheelOdom odom(...);
         *
         * void autonomous() {
         *   // set the starting position of the robot
         *   odom.setPose({15_in, -12_in, 90_cDeg});
         * }
         * @endcode
         */
        void setPose(units::Pose pose);
        /**
         * @brief start the tracking task. Sensors need to be calibrated beforehand
         *
         * Starts the tracking task if it has not been started yet.
         * Nothing happens if the task has already been started.
         * Sensors need to be calibrated before this function is called.
         *
         * @param period how long to wait before updating the task again. Defaults to 10 ms
         *
         * @b Example:
         * @code {.cpp}
         * // create TrackingWheelOdom object
         * lemlib::TrackingWheelOdom odom(...);
         *
         * void initialize() {
         *   // calibrate sensors
         *   imu.calibrate();
         *   // start the tracking task
         *   odom.startTask();
         *   // now we can get position data
         * }
         * @endcode
         *
         * @b Example:
         * @code {.cpp}
         * // create TrackingWheelOdom object
         * lemlib::TrackingWheelOdom odom(...);
         *
         * void initialize() {
         *   // calibrate sensors
         *   imu.calibrate();
         *   // start the tracking task
         *   // and have it update every 20 milliseconds
         *   // or, in other words, 50 times per second
         *   odom.startTask(20_msec);
         *   // now we can get position data
         * }
         * @endcode
         */
        void startTask(Time period = 10_msec);
        /**
         * @brief Destroy the Tracking Wheel Odometry object. Stops the tracking task
         *
         * De-allocation of IMU pointers is up to the caller.
         */
        ~SerialOdometry();
    private:
        /**
         * @brief update the estimated pose
         *
         * This function should have its own dedicated task
         */
        void update(Time period);
        units::Pose m_pose = {0_m, 0_m, 0_cDeg};
        Angle m_offset = 0_stDeg;
        std::optional<pros::Task> m_task = std::nullopt;
};
}