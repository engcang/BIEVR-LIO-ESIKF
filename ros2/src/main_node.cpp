#include <csignal>
#include <iostream>

#include "main.hpp"

void signalHandler(int _signal)
{
    (void)_signal;
    std::cout << "You pressed Ctrl+C, exiting BIEVR-LIO-ESIKF!" << std::endl;
    BievrLioApplication::requestExit();
}

int main(int _argc,
         char **_argv)
{
    rclcpp::init(_argc, _argv);
    std::signal(SIGINT, signalHandler);
    static BievrLioApplication application;
    return application.mainFunction();
}
