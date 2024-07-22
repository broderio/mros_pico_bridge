#include "mbotSerialBridge.hpp"

MbotSerialBridge::MbotSerialBridge() : node("serial_bridge", URI("0.0.0.0", MEDIATOR_PORT_NUM))
{
    mros::Console::log(mros::LogLevel::DEBUG, "Opening serial port ... ");
    serialDevice = open("/dev/mbot_lcm", O_RDWR);
    if (serialDevice < 0)
    {
        mros::Console::log(mros::LogLevel::ERROR, "Unable to open serial port.");
        throw std::runtime_error("Unable to open serial port.");
    }
    mros::Console::log(mros::LogLevel::DEBUG, "Serial port opened!");

    struct termios options;
    tcgetattr(serialDevice, &options);
    cfsetspeed(&options, B115200);
    options.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    options.c_cflag |= CS8 | CREAD | CLOCAL;
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ISIG | ECHO | IEXTEN); /* Set non-canonical mode */
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;
    cfmakeraw(&options);
    tcflush(serialDevice, TCIFLUSH);
    tcsetattr(serialDevice, TCSANOW, &options);
    if(tcgetattr(serialDevice, &options) != 0)
    {
        mros::Console::log(mros::LogLevel::ERROR, "Unable to set serial port attributes.");
        throw std::runtime_error("Unable to set serial port attributes.");
    }

    int flags = fcntl(serialDevice, F_GETFL, 0); 
    if (fcntl(serialDevice, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        mros::Console::log(mros::LogLevel::ERROR, "Unable to set serial port to non-blocking mode.");
        throw std::runtime_error("Unable to set serial port to non-blocking mode.");
    }

    mros::Console::log(mros::LogLevel::DEBUG, "Creating publishers ... ");
    odometryPub = node.advertise<mbot_msgs::Pose2D>("odometry", 10);
    imuPub = node.advertise<mbot_msgs::MbotImu>("imu", 10);
    encodersPub = node.advertise<mbot_msgs::Encoders>("encoders", 10);
    mbotVelPub = node.advertise<mbot_msgs::Twist2D>("mbot_vel", 10);
    motorVelPub = node.advertise<mbot_msgs::MotorVel>("motor_vel", 10);
    motorPwmPub = node.advertise<mbot_msgs::MotorPwm>("motor_pwm", 10);

    mros::Console::log(mros::LogLevel::DEBUG, "Creating subscribers ... ");
    timesyncSub = node.subscribe<mbot_msgs::Timestamp>("timesync", 10, std::bind(&MbotSerialBridge::timesyncCallback, this, std::placeholders::_1));
    odometryResetSub = node.subscribe<mbot_msgs::Pose2D>("odometry_reset", 10, std::bind(&MbotSerialBridge::odometryResetCallback, this, std::placeholders::_1));
    encodersResetSub = node.subscribe<mbot_msgs::Encoders>("encoders_reset", 10, std::bind(&MbotSerialBridge::encodersResetCallback, this, std::placeholders::_1));
    motorPwmCmdSub = node.subscribe<mbot_msgs::MotorPwm>("motor_pwm_cmd", 10, std::bind(&MbotSerialBridge::motorPwmCmdCallback, this, std::placeholders::_1));
    motorVelCmdSub = node.subscribe<mbot_msgs::MotorVel>("motor_vel_cmd", 10, std::bind(&MbotSerialBridge::motorVelCmdCallback, this, std::placeholders::_1));
    mbotVelCmdSub = node.subscribe<mbot_msgs::Twist2D>("mbot_vel_cmd", 10, std::bind(&MbotSerialBridge::mbotVelCmdCallback, this, std::placeholders::_1));
}

MbotSerialBridge::~MbotSerialBridge()
{
    std::lock_guard<std::mutex> lock(serialMutex);
    close(serialDevice);
}

void MbotSerialBridge::timesyncCallback(const mbot_msgs::Timestamp &msg)
{
    std::string msgStr = Parser::encode(msg, MBOT_TIMESYNC);
    std::lock_guard<std::mutex> lock(serialMutex);
    write(serialDevice, msgStr.c_str(), sizeof(msgStr));
}

void MbotSerialBridge::odometryResetCallback(const mbot_msgs::Pose2D &msg)
{
    std::string msgStr = Parser::encode(msg, MBOT_ODOMETRY_RESET);
    std::lock_guard<std::mutex> lock(serialMutex);
    write(serialDevice, msgStr.c_str(), sizeof(msgStr));
}

void MbotSerialBridge::encodersResetCallback(const mbot_msgs::Encoders &msg)
{
    std::string msgStr = Parser::encode(msg, MBOT_ENCODERS_RESET);
    std::lock_guard<std::mutex> lock(serialMutex);
    write(serialDevice, msgStr.c_str(), sizeof(msgStr));
}

void MbotSerialBridge::motorPwmCmdCallback(const mbot_msgs::MotorPwm &msg)
{
    std::string msgStr = Parser::encode(msg, MBOT_MOTOR_PWM);
    std::lock_guard<std::mutex> lock(serialMutex);
    write(serialDevice, msgStr.c_str(), sizeof(msgStr));
}

void MbotSerialBridge::motorVelCmdCallback(const mbot_msgs::MotorVel &msg)
{
    std::string msgStr = Parser::encode(msg, MBOT_MOTOR_VEL);
    std::lock_guard<std::mutex> lock(serialMutex);
    write(serialDevice, msgStr.c_str(), sizeof(msgStr));
}

void MbotSerialBridge::mbotVelCmdCallback(const mbot_msgs::Twist2D &msg)
{
    std::string msgStr = Parser::encode(msg, MBOT_VEL_CMD);
    std::lock_guard<std::mutex> lock(serialMutex);
    write(serialDevice, msgStr.c_str(), sizeof(msgStr));
}

bool MbotSerialBridge::readHeader(uint8_t *headerData)
{
    unsigned char triggerVal = 0x00;
    int rc = 0x00;
    while (triggerVal != 0xff && node.ok() && rc != 1)
    {
        rc = read(serialDevice, &triggerVal, 1);
        if (rc < 0)
        {
            return -1;
        }
    }
    if (!node.ok())
    {
        return -1;
    }
    headerData[0] = triggerVal;

    rc = read(serialDevice, &headerData[1], ROS_HEADER_LENGTH - 1);
    if (rc < 0)
    {
        return -1;
    }

    return (rc == ROS_HEADER_LENGTH - 1);
}

bool MbotSerialBridge::validateHeader(uint8_t *headerData)
{
    bool validHeader = (headerData[1] == 0xfe);
    uint8_t cs1Addends[2] = {headerData[2], headerData[3]};
    uint8_t csMsgLen = checksum(cs1Addends, 2);
    validHeader = validHeader && (csMsgLen == headerData[4]);

    return validHeader;
}

bool MbotSerialBridge::readMessage(uint8_t *msgDataSerialized, uint16_t msgLen, char *topicMsgDataChecksum)
{
    int rc = read(serialDevice, msgDataSerialized, msgLen);
    if (rc < 0)
    {
        return -1;
    }
    bool validMsg = (rc == msgLen);

    rc = read(serialDevice, topicMsgDataChecksum, 1);
    if (rc < 0)
    {
        return -1;
    }
    validMsg = validMsg && (rc == 1);

    return validMsg;
}

bool MbotSerialBridge::validateMessage(uint8_t *headerData, uint8_t *msgDataSerialized, uint16_t msgLen, char topicMsgDataChecksum)
{
    uint8_t *cs2Addends = new uint8_t[msgLen + 2];
    cs2Addends[0] = headerData[5];
    cs2Addends[1] = headerData[6];
    std::memcpy(&cs2Addends[2], msgDataSerialized, msgLen);

    uint8_t csTopicMsgData = checksum(cs2Addends, msgLen + 2);
    bool validMsg = (csTopicMsgData == topicMsgDataChecksum);

    delete[] cs2Addends;

    return validMsg;
}

void MbotSerialBridge::handleMessage(uint8_t *msgDataSerialized, uint16_t msgLen, uint16_t topicId)
{
    std::string msgStr;
    msgStr.assign((char *)msgDataSerialized, msgLen);

    switch (topicId)
    {
    case MBOT_ODOMETRY:
    {
        mbot_msgs::Pose2D msg;
        msg.decode(msgStr);
        odometryPub->publish(msg);
        break;
    }
    case MBOT_IMU:
    {
        mbot_msgs::MbotImu msg;
        msg.decode(msgStr);
        imuPub->publish(msg);
        break;
    }
    case MBOT_ENCODERS:
    {
        mbot_msgs::Encoders msg;
        msg.decode(msgStr);
        encodersPub->publish(msg);
        break;
    }
    case MBOT_VEL:
    {
        mbot_msgs::Twist2D msg;
        msg.decode(msgStr);
        mbotVelPub->publish(msg);
        break;
    }
    case MBOT_MOTOR_VEL:
    {
        mbot_msgs::MotorVel msg;
        msg.decode(msgStr);
        motorVelPub->publish(msg);
        break;
    }
    case MBOT_MOTOR_PWM:
    {
        mbot_msgs::MotorPwm msg;
        msg.decode(msgStr);
        motorPwmPub->publish(msg);
        break;
    }
    default:
        break;
    }
}

uint8_t MbotSerialBridge::checksum(uint8_t* addends, int len) {
    //takes in an array and sums the contents then checksums the array
    int sum = 0;
    for (int i = 0; i < len; i++) {
        sum += addends[i];
    }
    return 255 - ( ( sum ) % 256 );
}

void MbotSerialBridge::timesyncThread()
{
    while (node.ok())
    {
        mbot_msgs::Timestamp msg;
        struct timeval tv;
        gettimeofday (&tv, NULL);
        msg.utime = (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
        timesyncCallback(msg);
        sleep(1000);
    }

}

void MbotSerialBridge::run()
{
    mros::Console::log(mros::LogLevel::DEBUG, "Spinning node ...");
    node.spin(true);

    mros::Console::log(mros::LogLevel::DEBUG, "Starting timesync thread ...");
    std::thread timesyncThreadHandler(&MbotSerialBridge::timesyncThread, this);
    timesyncThreadHandler.detach();

    uint8_t headerData[ROS_HEADER_LENGTH];
    headerData[0] = 0x00;
    while (node.ok())
    {   
        // Read the header and check if we lost serial connection
        int headerStatus = readHeader(headerData);
        if (headerStatus < 0)
        {
            // fprintf(stderr, "[ERROR] Serial device is not available, exiting thread to attempt reconnect...\n");
            mros::Console::log(mros::LogLevel::ERROR, "Serial device is not available, exiting thread to attempt reconnect...");
            continue; // Break the loop if the device is not available
        }

        bool validHeader = (headerStatus == 1);
        if (validHeader)
        {
            validHeader = validateHeader(headerData);
        }

        if (validHeader)
        {
            uint16_t msgLen = ((uint16_t)headerData[3] << 8) + (uint16_t)headerData[2];
            uint16_t topicId = ((uint16_t)headerData[6] << 8) + (uint16_t)headerData[5];
            uint8_t msgDataSerialized[msgLen];

            int avail = 0;
            ioctl(serialDevice, FIONREAD, &avail);
            while (avail < (msgLen + 1) && node.ok())
            {
                usleep(1000);
                ioctl(serialDevice, FIONREAD, &avail);
            }
            if (!node.ok())
            {
                break;
            }

            // Read the message and check if we lost serial connection
            char topicMsgDataChecksum = 0;
            int msgStatus = readMessage(msgDataSerialized, msgLen, &topicMsgDataChecksum);
            if (msgStatus < 0)
            {
                // fprintf(stderr, "[ERROR] Serial device is not available, exiting thread to attempt reconnect...\n");
                mros::Console::log(mros::LogLevel::ERROR, "Serial device is not available, exiting thread to attempt reconnect...");
                continue; // Break the loop if the device is not available
            }

            bool validMsg = (msgStatus == 1);
            if (validMsg)
            {
                validMsg = validateMessage(headerData, msgDataSerialized, msgLen, topicMsgDataChecksum);
                if (validMsg)
                {
                    handleMessage(msgDataSerialized, msgLen, topicId);
                }
            }
        }
        headerData[0] = 0x00;
    }
}