/**
 * @file Robot.cpp
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */
#include <iostream>
#include "../../include/robotarm/app/Robotarm.hpp"
#include "../../include/robotarm/dll/SSC32.hpp"
#include "../../include/robotarm/app/Logger.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto robot = std::make_shared<Robotarm>(std::make_unique<SSC32>("/dev/ttyUSB0", 9600, 8));
    auto logger = std::make_shared<Logger>();

    executor.add_node(robot);
    executor.add_node(logger);
    executor.spin();

    rclcpp::shutdown();
}
