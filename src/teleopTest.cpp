#include <iostream>
#include <memory>
#include <curses.h>

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

class Teleop {
public:
    Teleop();
    ~Teleop();
    void run();

private:
    mros::Node node;
    std::shared_ptr<mros::Publisher> mbotVelPub;
};  

Teleop::Teleop()
: node("teleop", URI("0.0.0.0", MEDIATOR_PORT_NUM))
{
    mbotVelPub = node.advertise<mbot_msgs::Twist2D>("mbot_vel_cmd", 1);
}

Teleop::~Teleop()
{
}

void Teleop::run()
{
    mros::Console::log(mros::LogLevel::DEBUG, "Teleop running...");
    mbot_msgs::Twist2D msg;

    node.spin(true);

    initscr();
    char ch;
    cbreak();
    while (node.ok())
    {
        msg.vx = 0.0;
        msg.vy = 0.0;
        msg.wz = 0.0;

        // Stores the pressed key in ch
        ch = getch();

        // Terminates the loop
        // when escape is pressed
        if (int(ch) == 27)
            break;

        switch (ch)
        {
        case 'w':
            msg.vx = 0.5;
            break;
        case 's':
            msg.vx = -0.5;
            break;
        case 'a':
            msg.wz = 1.5;
            break;
        case 'd':
            msg.wz = -1.5;
            break;
        case ' ':
            msg.vx = 0.0;
            msg.vy = 0.0;
            msg.wz = 0.0;
            break;
        }

        mbotVelPub->publish(msg);
    }
    endwin();
}

int main()
{
    Teleop teleop;
    teleop.run();
    return 0;
}