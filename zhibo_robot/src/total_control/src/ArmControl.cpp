#include "ArmControl.h"
#include "Constant.h"

#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

double convertToRad(double degree) {
    return degree * M_PI / 180.0;
}


ArmControl::ArmControl(const std::string &name) : rclcpp::Node(name) {
    m_targetJointStatus = std::vector<double>(7, 0.0);
    m_curJointStatus = std::vector<double>(7, 0.0);
    m_curPoseStatus = std::vector<double>(6, 0.0);
    m_initPos = std::vector<double>{0.0, 56.0, -70.0, 0.0, 14.0, 0.0, 0.0};

    m_jointPub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 3);
    m_posCmdPub = this->create_publisher<piper_msgs::msg::PosCmd>("pos_cmd", 3);
    m_gripperCtrlPub = this->create_publisher<std_msgs::msg::Float64>("gripper_ctrl", 1);

    m_bottomArmAnglePub = this->create_publisher<std_msgs::msg::Float64>("bottom_arm_angle", 10);
}


void ArmControl::jointStateCallback(const sensor_msgs::msg::JointState &jointState) {
    {
        std::scoped_lock lock(m_curJointStatusMutex);
        m_curJointStatus[0] = jointState.position[0] * 180.0 / M_PI;
        m_curJointStatus[1] = jointState.position[1] * 180.0 / M_PI;
        m_curJointStatus[2] = jointState.position[2] * 180.0 / M_PI;
        m_curJointStatus[3] = jointState.position[3] * 180.0 / M_PI;
        m_curJointStatus[4] = jointState.position[4] * 180.0 / M_PI;
        m_curJointStatus[5] = jointState.position[5] * 180.0 / M_PI;
        m_curJointStatus[6] = jointState.position[6] * 180.0 / M_PI;
    }
//    std::cout << "cur joint: " << m_curJointStatus[0] << " " << m_curJointStatus[1] << " " << m_curJointStatus[2]
//              << " " << m_curJointStatus[3] << " " << m_curJointStatus[4] << " " << m_curJointStatus[5] << " "
//              << m_curJointStatus[6] << std::endl;
}

void ArmControl::publishJointStateOnce() {
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
        msg.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 60.0};
        msg.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::scoped_lock lock(m_targetJointStatusMutex);
        msg.position = {convertToRad(m_targetJointStatus[0]), convertToRad(m_targetJointStatus[1]),
                        convertToRad(m_targetJointStatus[2]),
                        convertToRad(m_targetJointStatus[3]), convertToRad(m_targetJointStatus[4]),
                        convertToRad(m_targetJointStatus[5]),
                        convertToRad(m_targetJointStatus[6])};

        m_jointPub->publish(msg);
    }
}


void ArmControl::initPos() {
//    m_waitMode = WaitMode::JOINT_MOVE;
    const static std::vector<double> initPos = {0.0, 56.0, -70.0, 0.0, 14.0, 0.0, 0.0};
    {
        std::scoped_lock lock(m_targetJointStatusMutex);
        for (int i = 0; i < 7; i++) {
            m_targetJointStatus[i] = initPos[i];
        }
    }

    publishJointStateOnce();
}

void ArmControl::poseCallback(const geometry_msgs::msg::Pose &pose) {
    std::scoped_lock lock(m_curPoseStatusMutex);
    m_curPoseStatus[0] = pose.position.x * 1000;
    m_curPoseStatus[1] = pose.position.y * 1000;
    m_curPoseStatus[2] = pose.position.z * 1000;
    tf2::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    constexpr double RAD_TO_DEG = 180.0 / M_PI;
    m_curPoseStatus[3] = roll * RAD_TO_DEG;
    m_curPoseStatus[4] = pitch * RAD_TO_DEG;
    m_curPoseStatus[5] = yaw * RAD_TO_DEG;
//    std::cout << "cur pose: " << m_curPoseStatus[0] << " " << m_curPoseStatus[1] << " " << m_curPoseStatus[2]
//              << " " << m_curPoseStatus[3] << " " << m_curPoseStatus[4] << " " << m_curPoseStatus[5] << std::endl;
}

void ArmControl::publishPosCmdOnce(double targetY, double targetZ, double yaw) {
//    std::cout << "targetY: " << targetY << std::endl;
    piper_msgs::msg::PosCmd msg;
    m_targetYStatus = targetY;
    msg.x = 0.185878 * 1000;
    msg.y = targetY;
//    msg.z = 0.484457 * 1000;
    msg.z = targetZ;
    msg.roll = 0.0;
    msg.pitch = 85.3;
    msg.yaw = yaw;
    msg.gripper = 0.0;
    m_posCmdPub->publish(msg);
}

void ArmControl::resetPos() {
    const static std::vector<double> resetPos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    {
        std::scoped_lock lock(m_targetJointStatusMutex);
        for (int i = 0; i < 7; i++) {
            m_targetJointStatus[i] = resetPos[i];
        }
    }

    publishJointStateOnce();
}

void ArmControl::rotatePos(int rotate, double rotateZ, double targetAngle) {
    const static std::vector<double> rightPos = {62.0979, 76.6262, -83.0408, -89.1505, 61.8141, 79.5206, 0};
    const static std::vector<double> leftPos = {-62.0979, 76.6262, -83.0408, 89.1505, 61.8141, -79.5206, 0};
    if (rotate == 1) {
        std::scoped_lock lock(m_targetJointStatusMutex);
        m_targetJointStatus[0] = targetAngle;
        for (int i = 1; i < 7; i++) {
            m_targetJointStatus[i] = leftPos[i];
        }
        m_targetJointStatus[2] = rotateZ;
        m_targetJointStatus[5] -= rotateZ - leftPos[2];
    } else if (rotate == 2) {
        std::scoped_lock lock(m_targetJointStatusMutex);
        m_targetJointStatus[0] = targetAngle;
        for (int i = 1; i < 7; i++) {
            m_targetJointStatus[i] = rightPos[i];
        }
        m_targetJointStatus[2] = rotateZ;
        m_targetJointStatus[5] += rotateZ - leftPos[2];
    }
    publishJointStateOnce();
}

//bool ArmControl::waitMoveComplete() {
//    if (m_waitMode == WaitMode::JOINT_MOVE) {
//        // joint move
//        {
//            std::scoped_lock lock(m_curJointStatusMutex);
//            if (std::abs(m_curJointStatus[0] - m_targetJointStatus[0]) < 0.5 &&
//                std::abs(m_curJointStatus[1] - m_targetJointStatus[1]) < 0.5 &&
//                std::abs(m_curJointStatus[2] - m_targetJointStatus[2]) < 0.5 &&
//                std::abs(m_curJointStatus[3] - m_targetJointStatus[3]) < 0.5 &&
//                std::abs(m_curJointStatus[4] - m_targetJointStatus[4]) < 0.5 &&
//                std::abs(m_curJointStatus[5] - m_targetJointStatus[5]) < 0.5 &&
//                std::abs(m_curJointStatus[6] - m_targetJointStatus[6]) < 0.5) {
//                return true;
//            } else {
//                return false;
//            }
//        }
//    } else if (m_waitMode == WaitMode::POSE_MOVE) {
//        // pose move
//        {
//            std::scoped_lock lock(m_curPoseStatusMutex);
//            if (std::abs(m_curPoseStatus[0] - 185.878) < 1 &&
//                std::abs(m_targetYStatus - m_curPoseStatus[1]) < 1 &&
//                std::abs(m_curPoseStatus[2] - 484.457) < 1
////                std::abs(m_curJointStatus[3] - 0) < 1 &&
////                std::abs(m_curJointStatus[4] - 85.3) < 1 &&
////                std::abs(m_curJointStatus[5] - 0) < 1
//                    ) {
//                return true;
//            } else {
//                return false;
//            }
//        }
//    }
//    return true;
//}

void ArmControl::move() {
//    static double angle = 0.0, pose = 0.0, poseZ = 0.484457 * 1000, angleZ = -83.0408;
//    if (m_moveHorizontalDirection == 0 && m_moveVerticalDirection == 0) {
//        return;
//    }
//    if (-POSE_MOVE_LIMIT <= pose && pose <= POSE_MOVE_LIMIT) {
//        pose += m_moveHorizontalDirection * POSE_MOVE_REFACTOR;
//        pose = std::min(m_curPoseStatus[1] + 80, pose);
//        pose = std::max(m_curPoseStatus[1] - 80, pose);
//        double yaw = 0.0;
//        poseZ += m_moveVerticalDirection * POSE_MOVE_REFACTOR;
//        if (-POSE_MOVE_LIMIT <= pose && pose <= POSE_MOVE_LIMIT) {
//            publishPosCmdOnce(pose, poseZ, yaw);
//        }
//        poseZ = std::max(400.0, poseZ);
//        poseZ = std::min(0.484457 * 1000, poseZ);
//        angle = 0;
//        return;
//    }
//    if (m_moveHorizontalDirection > 0) {
//        m_moveHorizontalDirection = 1;
//    }
//    if (m_moveHorizontalDirection < 0) {
//        m_moveHorizontalDirection = -1;
//    }
//    if (angle == 0) {
//        if (pose > POSE_MOVE_LIMIT) {
//            angle += ANGLE_MOVE_REFACTOR;
//        } else {
//            angle -= ANGLE_MOVE_REFACTOR;
//        }
//        angleZ = m_curJointStatus[2];
//        return;
//    }
//
//    if (angle > 0) {
//        // in right pose
//        angle += m_moveHorizontalDirection * ANGLE_MOVE_REFACTOR;
//        angle = std::min(angle, ANGLE_MOVE_LIMIT);
//        if (angle <= 0) {
//            angle = 0;
//            pose = std::min(pose, POSE_MOVE_LIMIT);
//            poseZ = m_curPoseStatus[2];
//        }
//
//        angleZ -= m_moveVerticalDirection * ANGLE_MOVE_REFACTOR;
//        angleZ = std::max(angleZ, -83.0408);
//        angleZ = std::min(angleZ, ARM_HEIGHT_MAX_ANGLE);
//        rotatePos(2, angleZ, RIGHT_LIMIT_ANGLE + angle);
//    } else {
//        // in left pose
//        angle += m_moveHorizontalDirection * ANGLE_MOVE_REFACTOR;
//        angle = std::max(angle, -ANGLE_MOVE_LIMIT);
//        if (angle >= 0) {
//            angle = 0;
//            pose = std::max(pose, -POSE_MOVE_LIMIT);
//            poseZ = m_curPoseStatus[2];
//        }
//        angleZ -= m_moveVerticalDirection * ANGLE_MOVE_REFACTOR;
//        angleZ = std::max(angleZ, -83.0408);
//        angleZ = std::min(angleZ, ARM_HEIGHT_MAX_ANGLE);
//        rotatePos(1, angleZ, LEFT_LIMIT_ANGLE + angle);
//    }
}

void ArmControl::setTrackResult2(const std::vector<int16_t> &target) {
    double x1 = target[0], x2 = target[2], y1 = target[1], y2 = target[3];
    double centerX = (x1 + x2) / 2;
    if (-50 < centerX - 320 && centerX - 320 < 50) {
        m_moveHorizontalDirection = 0;
    } else if (centerX < 320) {
//        m_moveHorizontalDirection = -1 * int((320 - centerX) / 20);
        m_moveHorizontalDirection = -1;
    } else {
//        m_moveHorizontalDirection = 1 * int((centerX - 320) / 20);
        m_moveHorizontalDirection = 1;
    }
    double maxy = 480 - std::min(y1, y2);
//    std::cout << "maxy: " << maxy << std::endl;
    if (maxy > 320) {
        m_moveVerticalDirection = 1;
    } else if (maxy < 260) {
        m_moveVerticalDirection = -1;
    } else {
        m_moveVerticalDirection = 0;
    }
}

void ArmControl::clawGripper() {
    auto msg = std_msgs::msg::Float64();
    msg.data = 60;
    m_gripperCtrlPub->publish(msg);
}

void ArmControl::releaseGripper() {
    auto msg = std_msgs::msg::Float64();
    msg.data = 100;
    m_gripperCtrlPub->publish(msg);
}

void ArmControl::move2() {
    if (m_moveHorizontalDirection == 0 && m_moveVerticalDirection == 0) {
        return;
    }
//    m_verticalDiff = m_verticalDiff + m_moveVerticalDirection * ANGLE_MOVE_VERTICAL_REFACTOR;
//    m_verticalDiff = std::max(m_verticalDiff, -ANGLE_MOVE_VERTICAL_LIMIT);
//    m_verticalDiff = std::min(m_verticalDiff, ANGLE_MOVE_VERTICAL_LIMIT);
//    if (m_verticalDiff > m_curJointStatus[0] + 2 * ANGLE_MOVE_VERTICAL_REFACTOR) {
//        m_verticalDiff = m_curJointStatus[0] + 2 * ANGLE_MOVE_VERTICAL_REFACTOR;
//    }
//    if (m_verticalDiff < m_curJointStatus[0] - 2 * ANGLE_MOVE_VERTICAL_REFACTOR) {
//        m_verticalDiff = m_curJointStatus[0] - 2 * ANGLE_MOVE_VERTICAL_REFACTOR;
//    }
    if (m_moveHorizontalDirection != 0) {
        m_horizontalDiff = m_horizontalDiff + m_moveHorizontalDirection * ANGLE_MOVE_HORIZONTAL_REFACTOR;
        m_horizontalDiff = std::max(m_horizontalDiff, -ANGLE_MOVE_HORIZONTAL_LIMIT);
        m_horizontalDiff = std::min(m_horizontalDiff, ANGLE_MOVE_HORIZONTAL_LIMIT);
        {
            std::scoped_lock lock(m_targetJointStatusMutex);
            m_targetJointStatus[0] = m_initPos[0] + m_horizontalDiff;
            if (m_targetJointStatus[0] > m_curJointStatus[0] + 2 * ANGLE_MOVE_HORIZONTAL_REFACTOR) {
                m_targetJointStatus[0] = m_curJointStatus[0] + 2 * ANGLE_MOVE_HORIZONTAL_REFACTOR;
            }
            if (m_targetJointStatus[0] < m_curJointStatus[0] - 2 * ANGLE_MOVE_HORIZONTAL_REFACTOR) {
                m_targetJointStatus[0] = m_curJointStatus[0] - 2 * ANGLE_MOVE_HORIZONTAL_REFACTOR;
            }
        }
    }

    if (m_moveVerticalDirection != 0) {
        m_verticalDiff = m_verticalDiff + m_moveVerticalDirection * ANGLE_MOVE_VERTICAL_REFACTOR;
        m_verticalDiff = std::max(m_verticalDiff, -ANGLE_MOVE_VERTICAL_LIMIT);
        m_verticalDiff = std::min(m_verticalDiff, ANGLE_MOVE_VERTICAL_LIMIT);
        {
            std::scoped_lock lock(m_targetJointStatusMutex);
            m_targetJointStatus[2] = m_initPos[2] - m_verticalDiff;

            if (m_targetJointStatus[2] > m_curJointStatus[2] + 2 * ANGLE_MOVE_VERTICAL_REFACTOR) {
                m_targetJointStatus[2] = m_curJointStatus[2] + 2 * ANGLE_MOVE_VERTICAL_REFACTOR;
            }
            if (m_targetJointStatus[2] < m_curJointStatus[2] - 2 * ANGLE_MOVE_VERTICAL_REFACTOR) {
                m_targetJointStatus[2] = m_curJointStatus[2] - 2 * ANGLE_MOVE_VERTICAL_REFACTOR;
            }

            m_targetJointStatus[4] = m_initPos[4] + m_verticalDiff;
            if (m_targetJointStatus[4] > m_curJointStatus[4] + 2 * ANGLE_MOVE_VERTICAL_REFACTOR) {
                m_targetJointStatus[4] = m_curJointStatus[4] + 2 * ANGLE_MOVE_VERTICAL_REFACTOR;
            }
            if (m_targetJointStatus[4] < m_curJointStatus[4] - 2 * ANGLE_MOVE_VERTICAL_REFACTOR) {
                m_targetJointStatus[4] = m_curJointStatus[4] - 2 * ANGLE_MOVE_VERTICAL_REFACTOR;
            }
        }


    }

    publishJointStateOnce();


}

