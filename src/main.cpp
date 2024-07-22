#include "mbotSerialBridge.hpp"

int main()
{  
    MbotSerialBridge bridge;
    mros::Console::log(mros::LogLevel::DEBUG, "Starting serial bridge...");
    bridge.run();
    return 0;
}