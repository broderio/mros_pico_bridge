#include "utils.hpp"
#include "mros/node.hpp"
#include "mros/publisher.hpp"
#include "mros/subscriber.hpp"
#include "mros/console.hpp"

#include "messages/mbot_msgs/encoders.hpp"
#include "messages/mbot_msgs/mbotImu.hpp"
#include "messages/mbot_msgs/motorPwm.hpp"
#include "messages/mbot_msgs/motorVel.hpp"
#include "messages/mbot_msgs/pose2D.hpp"
#include "messages/mbot_msgs/timestamp.hpp"
#include "messages/mbot_msgs/twist2D.hpp"

void odometryCb(const mbot_msgs::Pose2D &msg)
{
    mros::Console::log(mros::LogLevel::DEBUG, "Odometry: x: " + msg.x.toString() + ", y: " + msg.y.toString() + ", theta: " + msg.theta.toString());
}

void imuCb(const mbot_msgs::MbotImu &msg)
{
    mros::Console::log(mros::LogLevel::DEBUG, "IMU: ax=" + msg.accel[0].toString() + ", ay=" + msg.accel[1].toString() + ", az=" + msg.accel[2].toString());
}

void encodersCb(const mbot_msgs::Encoders &msg)
{
    mros::Console::log(mros::LogLevel::DEBUG, "Encoders: A: " + msg.ticks[0].toString() + ", B: " + msg.ticks[1].toString() + ", C: " + msg.ticks[2].toString());
}

void mbotVelCb(const mbot_msgs::Twist2D &msg)
{
    mros::Console::log(mros::LogLevel::DEBUG, "Mbot Vel: vx: " + msg.vx.toString() + ", vy: " + msg.vy.toString() + ", wz: " + msg.wz.toString());
}

void motorVelCb(const mbot_msgs::MotorVel &msg)
{
    mros::Console::log(mros::LogLevel::DEBUG, "Motor Vel: A: " + msg.velocity[0].toString() + ", B: " + msg.velocity[1].toString() + ", C: " + msg.velocity[2].toString());
}

void motorPwmCb(const mbot_msgs::MotorPwm &msg)
{
    mros::Console::log(mros::LogLevel::DEBUG, "Motor PWM: A: " + msg.pwm[0].toString() + ", B: " + msg.pwm[1].toString() + ", C: " + msg.pwm[2].toString());
}

int main(int argc, char **argv)
{
    mros::Console::init("serial_listener");
    
    // Get topic from input args
    if (argc < 2)
    {
        mros::Console::log(mros::LogLevel::ERROR, "Usage: listenerTest <topic>");
        return 1;
    }

    std::string topic = argv[1];
    mros::Console::log(mros::LogLevel::DEBUG, "Listening to topic: " + topic);

    // Create a node
    mros::Node node("serial_listener", URI("0.0.0.0", MEDIATOR_PORT_NUM));

    // Create a subscriber
    std::shared_ptr<mros::Subscriber> sub;

    if (topic == "odometry")
    {
        sub = node.subscribe<mbot_msgs::Pose2D>(topic, 1, odometryCb);
    }
    else if (topic == "imu")
    {
        sub = node.subscribe<mbot_msgs::MbotImu>(topic, 1, imuCb);
    }
    else if (topic == "encoders")
    {
        sub = node.subscribe<mbot_msgs::Encoders>(topic, 1, encodersCb);
    }
    else if (topic == "mbot_vel")
    {
        sub = node.subscribe<mbot_msgs::Twist2D>(topic, 1, mbotVelCb);
    }
    else if (topic == "motor_vel")
    {
        sub = node.subscribe<mbot_msgs::MotorVel>(topic, 1, motorVelCb);
    }
    else if (topic == "motor_pwm")
    {
        sub = node.subscribe<mbot_msgs::MotorPwm>(topic, 1, motorPwmCb);
    }

    // Spin
    node.spin();
}