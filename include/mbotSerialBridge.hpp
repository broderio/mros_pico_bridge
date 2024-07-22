#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include <stdio.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/time.h>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions

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

#define ROS_HEADER_LENGTH 7
#define ROS_FOOTER_LENGTH 1
#define ROS_PKG_LENGTH  (ROS_HEADER_LENGTH + ROS_FOOTER_LENGTH) //length (in bytes) of ros packaging (header and footer)

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

        // Callbacks
        void timesyncCallback(const mbot_msgs::Timestamp &msg);
        void odometryResetCallback(const mbot_msgs::Pose2D &msg);
        void encodersResetCallback(const mbot_msgs::Encoders &msg);
        void motorPwmCmdCallback(const mbot_msgs::MotorPwm &msg);
        void motorVelCmdCallback(const mbot_msgs::MotorVel &msg);
        void mbotVelCmdCallback(const mbot_msgs::Twist2D &msg);

        // Handlers for serial message parsing
        bool readHeader(uint8_t* headerData);
        bool validateHeader(uint8_t* headerData);
        bool readMessage(uint8_t* msgDataSerialized, uint16_t msgLen, char* topicMsgDataChecksum);
        bool validateMessage(uint8_t* headerData, uint8_t* msgDataSerialized, uint16_t msgLen, char topicMsgDataChecksum);
        void handleMessage(uint8_t* msgDataSerialized, uint16_t msgLen, uint16_t topicId);
        uint8_t checksum(uint8_t* addends, int len);

        // Thread for sending timesync messages over serial
        void timesyncThread();

        int serialDevice;
        std::mutex serialMutex;
};