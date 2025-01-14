#include "lemlib/tracking/SerialOdom.hpp"
#include "hardware/Encoder/V5RotationSensor.hpp"
#include "hardware/Encoder/ADIEncoder.hpp"
#include "LemLog/logger/Helper.hpp"
#include "units/Vector2D.hpp"

static logger::Helper helper("lemlib/odom/tracking_wheel_odom");

namespace lemlib {

SerialOdometry::SerialOdometry(ReversibleSmartPort port){}

units::Pose SerialOdometry::getPose() { return m_pose; }

void SerialOdometry::setPose(units::Pose pose) {
    m_offset += pose.orientation - m_pose.orientation;
    m_pose = pose;
}

void SerialOdometry::startTask(Time period) {
    // check if the task has been started yet
    if (m_task == std::nullopt) { // start the task
        m_task = pros::Task([this, period] { this->update(period); });
        helper.log(logger::Level::INFO, "Tracking task started!");
    } else {
        helper.log(logger::Level::WARN, "Tried to start tracking task, but it has already been started!");
    }
}


/**
 * @brief struct representing data from a tracking wheel
 *
 */
struct OdometryData {
        Length distance; /** the distance delta reported by the tracking wheel */
        Length offset; /** the offset of the tracking wheel used to measure the distance */
};
/**
 * @brief struct representing data from a tracking wheel
 *
 */
struct TrackingWheelData {
        Length distance; /** the distance delta reported by the tracking wheel */
        Length offset; /** the offset of the tracking wheel used to measure the distance */
};

/*
 * The implementation below is based off of
 * the document written by 5225A (Pilons)
 * http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
 */
void SerialOdometry::update(Time period) {
    // record the previous time, used for consistent loop timings
    Time prevTime = from_msec(pros::millis());
    // run until the task has been notified, which will probably never happen
    while (pros::Task::notify_take(true, 0) == 0) {
        const Time now = from_msec(pros::millis());
        const Time deltaTime = now - prevTime;

        // step 1: get tracking wheel deltas
        const TrackingWheelData horizontalData = findLateralDelta(m_horizontalWheels);
        const TrackingWheelData verticalData = findLateralDelta(m_verticalWheels);

        // step 2: calculate heading
        const std::optional<Angle> thetaOpt = calculateIMUHeading(m_Imus)
                                                  .or_else(std::bind(&calculateWheelHeading, m_horizontalWheels))
                                                  .or_else(std::bind(&calculateWheelHeading, m_verticalWheels));
        if (thetaOpt == std::nullopt) { // error checking
            helper.log(logger::Level::ERROR, "Not enough sensors available!");
            break;
        }
        const Angle theta = m_offset + *thetaOpt;

        // step 3: calculate change in local coordinates
        const Angle deltaTheta = theta - m_pose.orientation;
        const units::V2Position localPosition = [&] {
            const units::V2Position lateralDeltas = {verticalData.distance, horizontalData.distance};
            const units::V2Position lateralOffsets = {verticalData.offset, horizontalData.offset};
            if (deltaTheta == 0_stRad) return lateralDeltas; // prevent divide by 0
            return 2 * units::sin(deltaTheta / 2) * (lateralDeltas / to_stRad(deltaTheta) + lateralOffsets);
        }();

        // step 4: set global position
        m_pose += localPosition.rotatedBy(m_pose.orientation + deltaTheta / 2);
        m_pose.orientation = theta;

        // if current time - previous time > timeout
        // then set previous time to current time
        // this is to prevent the tracking task updating multiple times
        // with no delay in between
        if (deltaTime > period) prevTime = now;
        uint32_t dummyPrevTime = to_msec(prevTime);
        pros::Task::delay_until(&dummyPrevTime, to_msec(period));
        prevTime = from_msec(dummyPrevTime);
    }

    helper.log(logger::Level::INFO, "Tracking task stopped!");
}

SerialOdometry::~SerialOdometry() { m_task->notify(); }
}; // namespace lemlib