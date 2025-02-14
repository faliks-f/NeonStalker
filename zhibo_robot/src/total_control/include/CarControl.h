#ifndef ZHIBO_ROBOT_CARCONTROL_H
#define ZHIBO_ROBOT_CARCONTROL_H

#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <queue>

class CarControl : public rclcpp::Node {
private:
    double m_rotateSpeed = 0.0;
    double m_forwardSpeed = 0.0;

    std::atomic<double> m_armBottomAngle = 0.0;
    std::atomic<double> m_armHeightAngle = 0.0;
    std::atomic<bool> m_blocked = false;

    std::queue<int16_t> m_trackResult;
    int16_t m_distance = 0;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelPub;
public:
    explicit CarControl(const std::string &name);

    ~CarControl() override = default;

    void move();

    void jointStateCallback(const sensor_msgs::msg::JointState &jointState);

    void setTrackResult(const std::vector<int16_t> &result);

    void laserScanCallback(const sensor_msgs::msg::LaserScan &scan);

    void stop();
};


#endif //ZHIBO_ROBOT_CARCONTROL_H
