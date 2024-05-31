#pragma once

#include "units/Pose.hpp"
#include "units/units.hpp"

namespace lemlib {

/**
 * @struct DifferentialChassisSpeeds
 *
 * @brief represents the linear and velocity of the differential drivetrain
 */
struct DifferentialChassisSpeeds {
        LinearVelocity linearVelocity; /** linear velocity */
        AngularVelocity angularVelocity; /** angular velocity */
};

/**
 * @struct HolonomicChassisSpeeds
 *
 * @brief represents the field-relative x, y, and angular velocity of a holonomic drivetrain
 */
struct HolonomicChassisSpeeds {
        LinearVelocity xVelocity; /** x velocity */
        LinearVelocity yVelocity; /** y velocity */
        AngularVelocity angularVelocity; /** angular velocity */
};

/**
 * @class DifferentialMotion
 *
 * @brief Abstract class which represents a motion algorithm for a differential drive robot
 *
 * We use this abstraction to allow for different motion algorithms to be used with the same interface.
 * This allows us to easily create and maintain motion algorithms while also allowing the user to
 * make custom motion algorithms without having to recompile the library.
 */
class DifferentialMotion {
    public:
        DifferentialMotion(const Length trackWidth, const LinearVelocity maxVelocity, const float desaturateBias = 0.5);
        /**
         * @brief Calculates the speed of the left and right wheels of a differential drive robot
         *
         * @param pose the current pose of the robot
         * @return DifferentialChassisSpeeds the speed of the left and right wheels
         */
        virtual DifferentialChassisSpeeds calculate(units::Pose pose) = 0;
        /**
         * @brief Get whether the motion is running
         *
         * @return true the motion is running
         * @return false the motion is not running
         */
        bool isRunning() const;
        /**
         * @brief Destroy the Differential Motion object
         *
         */
        virtual ~DifferentialMotion();
    protected:
        bool running = true; /** whether the motion is running or not */
        const Length trackWidth; /** the track width of the robot */
        const LinearVelocity maxVelocity; /** the maximum velocity of the robot */
        const float desaturateBias; /** weight to determine whether to prioritize linear or angular velocity when
                               desaturating the motors */
        /**
         * @brief Desaturates the motor speeds to prevent the motors from saturating
         *
         * @param speeds the speeds to desaturate
         * @return DifferentialChassisSpeeds
         */
        DifferentialChassisSpeeds desaturate(DifferentialChassisSpeeds speeds) const;
};

/**
 * @class HolonomicMotion
 *
 * @brief Abstract class which represents a motion algorithm for a holonomic drive robot
 *
 * We use this abstraction to allow for different motion algorithms to be used with the same interface.
 * This allows us to easily create and maintain motion algorithms while also allowing the user to
 * make custom motion algorithms without having to recompile the library.
 */
class HolonomicMotion {
    public:
        /**
         * @brief Calculates the field-relative speed of a holonomic drive robot
         *
         * @param pose the current pose of the robot
         * @return HolonomicChassisSpeeds the field-relative speed of the robot
         */
        virtual HolonomicChassisSpeeds calculate(units::Pose pose) = 0;
        /**
         * @brief Get whether the motion is running
         *
         * @return true the motion is running
         * @return false the motion is not running
         */
        bool isRunning() const;
        /**
         * @brief Destroy the Holonomic Motion object
         *
         */
        virtual ~HolonomicMotion();
    protected:
        bool running = true; /** whether the motion is running or not */
};

} // namespace lemlib