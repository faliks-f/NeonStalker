#ifndef ZHIBO_ROBOT_TOTALCONTROLNODE_H
#define ZHIBO_ROBOT_TOTALCONTROLNODE_H

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <vector>


class ArmControl;

class CarControl;

class TotalControlNode : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_jointSub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_poseSub;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr m_trackResultSub;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr m_cmdSub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_bottomArmAngleSub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan >::SharedPtr m_laserScanSub;


    std::shared_ptr<ArmControl> m_armControl;
    std::shared_ptr<CarControl> m_carControl;


    rclcpp::TimerBase::SharedPtr m_timer;

    bool m_start = false;

public:
    explicit TotalControlNode(const std::string &name);

    ~TotalControlNode() override = default;

    void control();

    void trackResultCallback(const std_msgs::msg::Int16MultiArray& msg);

    void cmdCallback(std_msgs::msg::Int8 msg);
};


#endif //ZHIBO_ROBOT_TOTALCONTROLNODE_H
