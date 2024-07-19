#include <iostream>
#include <memory>

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


class MbotSerialBridge
{
    public:
        MbotSerialBridge();
        ~MbotSerialBridge();
        void run();

    private:
        mros::Node node;

        // Publishers
        std::shared_ptr<mros::Publisher> odometryPub;
        std::shared_ptr<mros::Publisher> imuPub;
        std::shared_ptr<mros::Publisher> encodersPub;
        std::shared_ptr<mros::Publisher> mbotVelPub;
        std::shared_ptr<mros::Publisher> motorVelPub;
        std::shared_ptr<mros::Publisher> motorPwmPub;

        // Subscribers
        std::shared_ptr<mros::Subscriber> timesyncSub;
        std::shared_ptr<mros::Subscriber> odometryResetSub;
        std::shared_ptr<mros::Subscriber> encodersResetSub;
        std::shared_ptr<mros::Subscriber> motorPwmCmdSub;
        std::shared_ptr<mros::Subscriber> motorVelCmdSub;
        std::shared_ptr<mros::Subscriber> mbotVelCmdSub;

        void timesyncCallback(const mbot_msgs::Timestamp &msg);
        void odometryResetCallback(const mbot_msgs::Pose2D &msg);
        void encodersResetCallback(const mbot_msgs::Encoders &msg);
        void motorPwmCmdCallback(const mbot_msgs::MotorPwm &msg);
        void motorVelCmdCallback(const mbot_msgs::MotorVel &msg);
        void mbotVelCmdCallback(const mbot_msgs::Twist2D &msg);

        int serialDevice;
};

void MbotSerialBridge::timesyncCallback(const mbot_msgs::Timestamp &msg)
{
    write(serialDevice, &msg, sizeof(msg));
}

void MbotSerialBridge::odometryResetCallback(const mbot_msgs::Pose2D &msg)
{
    write(serialDevice, &msg, sizeof(msg));
}

void MbotSerialBridge::encodersResetCallback(const mbot_msgs::Encoders &msg)
{
    write(serialDevice, &msg, sizeof(msg));
}

void MbotSerialBridge::motorPwmCmdCallback(const mbot_msgs::MotorPwm &msg)
{
    write(serialDevice, &msg, sizeof(msg));
}

void MbotSerialBridge::motorVelCmdCallback(const mbot_msgs::MotorVel &msg)
{
    write(serialDevice, &msg, sizeof(msg));
}

void MbotSerialBridge::mbotVelCmdCallback(const mbot_msgs::Twist2D &msg)
{
    write(serialDevice, &msg, sizeof(msg));
}

MbotSerialBridge::MbotSerialBridge() : node("serial_bridge", URI("0.0.0.0", MEDIATOR_PORT_NUM))
{
    serialDevice = open("/dev/mbot_lcm", O_RDWR);

    odometryPub = node.advertise<mbot_msgs::Pose2D>("odometry", 10);
    imuPub = node.advertise<mbot_msgs::MbotImu>("imu", 10);
    encodersPub = node.advertise<mbot_msgs::Encoders>("encoders", 10);
    mbotVelPub = node.advertise<mbot_msgs::Twist2D>("mbot_vel", 10);
    motorVelPub = node.advertise<mbot_msgs::MotorVel>("motor_vel", 10);
    motorPwmPub = node.advertise<mbot_msgs::MotorPwm>("motor_pwm", 10);

    timesyncSub = node.subscribe<mbot_msgs::Timestamp>("timesync", 10, std::bind(&MbotSerialBridge::timesyncCallback, this, std::placeholders::_1));
    odometryResetSub = node.subscribe<mbot_msgs::Pose2D>("odometry_reset", 10, std::bind(&MbotSerialBridge::odometryResetCallback, this, std::placeholders::_1));
    encodersResetSub = node.subscribe<mbot_msgs::Encoders>("encoders_reset", 10, std::bind(&MbotSerialBridge::encodersResetCallback, this, std::placeholders::_1));
    motorPwmCmdSub = node.subscribe<mbot_msgs::MotorPwm>("motor_pwm_cmd", 10, std::bind(&MbotSerialBridge::motorPwmCmdCallback, this, std::placeholders::_1));
    motorVelCmdSub = node.subscribe<mbot_msgs::MotorVel>("motor_vel_cmd", 10, std::bind(&MbotSerialBridge::motorVelCmdCallback, this, std::placeholders::_1));
    mbotVelCmdSub = node.subscribe<mbot_msgs::Twist2D>("mbot_vel_cmd", 10, std::bind(&MbotSerialBridge::mbotVelCmdCallback, this, std::placeholders::_1));
}

MbotSerialBridge::~MbotSerialBridge()
{
    close(serialDevice);
}

void MbotSerialBridge::run()
{
    node.spin(true);

    while (node.ok())
    {
        mros::Console::log(mros::LogLevel::DEBUG, "Running...");
        sleep(500);
    }
}

int main()
{  
    MbotSerialBridge bridge;
    bridge.run();
    return 0;
}