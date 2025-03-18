#include "TotalControlNode.h"
#include "ArmControl.h"
#include "CarControl.h"


using namespace std;

TotalControlNode::TotalControlNode(const std::string &name) : rclcpp::Node(name) {

    std::cout << "TotalControlNode constructor\n";

    m_armControl = std::make_shared<ArmControl>("arm_control");
    m_carControl = std::make_shared<CarControl>("car_control");

    m_timer1 = nullptr;
    m_jointSub = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states_single", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                m_carControl->jointStateCallback(*msg);
            });
    m_poseSub = this->create_subscription<geometry_msgs::msg::Pose>(
            "end_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
                m_armControl->poseCallback(*msg);
            });
    m_trackResultSub = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "track_result", 10,
            [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
                this->trackResultCallback(*msg);
            });
    m_cmdSub = this->create_subscription<std_msgs::msg::Int8>(
            "total_cmd", 10,
            [this](const std_msgs::msg::Int8::SharedPtr msg) {
                this->cmdCallback(*msg);
            });
    m_bottomArmAngleSub = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states_single", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                m_armControl->jointStateCallback(*msg);
            });
    m_laserScanSub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                m_carControl->laserScanCallback(*msg);
            });
}

void TotalControlNode::armControl() {
    m_armControl->move2();
    m_carControl->move();
//    m_armControl->publishJointStateOnce();
//    static int i = 0;
//    std::cout << "set result\n";
//    m_armControl->setTrackResult({360, 0, 380, 0});
//    m_armControl->setTrackResult({200, 0, 220, 0});
//    if (i < 70) {
//        m_armControl->setTrackResult({280, 0, 300, 0});
//    } else {
//        m_armControl->setTrackResult({340, 0, 360, 0});
//    }
//    if (i == 140) {
//        i = 0;
//    }
//    i++;
}

void TotalControlNode::trackResultCallback(const std_msgs::msg::Int16MultiArray &msg) {
    if (!m_start || msg.data.empty()) {
        return;
    }
    vector<int16_t> result(5, 0);
    for (int j = 0; j < 5; j++) {
        result[j] = msg.data[j];
    }
//    std::cout << "set result: " << result[0] << " " << result[1] << " " << result[2] << " " << result[3] << std::endl;
    m_armControl->setTrackResult2(result);
    m_carControl->setTrackResult(result);
}

void TotalControlNode::cmdCallback(std_msgs::msg::Int8 msg) {

    switch (msg.data) {
        case 1:
            m_armControl->initPos();
            cout << "init pos\n";
            break;
        case 2:
            m_armControl->clawGripper();
            cout << "claw gripper\n";
            break;
        case 3:
            if (m_timer1 == nullptr) {
                m_timer1 = this->create_wall_timer(std::chrono::milliseconds(50),
                                                   std::bind(&TotalControlNode::armControl, this));
            }
            m_start = true;
            break;
        case 4:
            m_carControl->rotate2Forward();
            cout << "rotate2Forward\n";
            break;
        case 8:
            m_armControl->releaseGripper();
            cout << "release gripper\n";
            break;
        case 9:
            if (m_timer1 != nullptr) {
                m_timer1->reset();
                m_timer1 = nullptr;
            }
            m_armControl->resetPos();
            m_carControl->stop();
            m_start = false;
            break;
    }
}






