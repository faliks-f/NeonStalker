#ifndef ZHIBO_ROBOT_ARMCONTROL_H
#define ZHIBO_ROBOT_ARMCONTROL_H

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <piper_msgs/msg/pos_cmd.hpp>
#include <std_msgs/msg/float64.hpp>

#include <vector>
#include <mutex>
#include <atomic>

class CarControl;

class ArmControl : public rclcpp::Node {
private:

//    enum class MoveMode {
//        LEFT_TO_LEFT_LIMIT_POSE = 0,
//        LEFT_LIMIT_POSE = 1,
//        LEFT_LIMIT_POSE_TO_RIGHT_LIMIT_POSE = 2,
//        RIGHT_LIMIT_POSE = 3,
//        RIGHT_LIMIT_POSE_TO_RIGHT = 4
//    };
//
//    enum class WaitMode {
//        JOINT_MOVE = 1,
//        POSE_MOVE = 2
//    };


    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_jointPub;
    rclcpp::Publisher<piper_msgs::msg::PosCmd>::SharedPtr m_posCmdPub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_gripperCtrlPub;

    rclcpp::TimerBase::SharedPtr m_timer;

    std::mutex m_targetJointStatusMutex;
    std::vector<double> m_targetJointStatus;

    std::atomic<double> m_targetYStatus{};

    std::mutex m_curJointStatusMutex;
    std::vector<double> m_curJointStatus;

    std::mutex m_curPoseStatusMutex;
    std::vector<double> m_curPoseStatus;

    // 1 for joint move; 2 for pose move
//    std::atomic<WaitMode> m_waitMode = WaitMode::JOINT_MOVE;

    // 0: left to left pose
    // 1: left pose
    // 2: left pose to right pose
    // 3: right pose
    // 4: right pose to right
//    std::atomic<MoveMode> m_moveMode = MoveMode::LEFT_LIMIT_POSE_TO_RIGHT_LIMIT_POSE;

    // -1: left, 1: right, 0: stop
    std::atomic<int> m_moveHorizontalDirection = 0;
    // -1: down, 1: up, 0: stop
    std::atomic<int> m_moveVerticalDirection = -1;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_bottomArmAnglePub;

public:
    explicit ArmControl(const std::string &name);

    ~ArmControl() override = default;

    void jointStateCallback(const sensor_msgs::msg::JointState &jointState);

    void poseCallback(const geometry_msgs::msg::Pose &pose);

    void publishJointStateOnce();

    void publishPosCmdOnce(double targetY, double targetZ, double yaw);

    void setTrackResult2(const std::vector<int16_t> &target);

    void initPos();

    void resetPos();

    void rotatePos(int rotate, double rotateZ, double targetAngle);

//    bool waitMoveComplete();

    void move();


    void clawGripper();

    void releaseGripper();
};


#endif //ZHIBO_ROBOT_ARMCONTROL_H
