#include "wheeltec_robot.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurnOnRobot>();
    node->control();
    return 0;
}