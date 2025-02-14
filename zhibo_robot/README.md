# zhibo_robot

## 启动命令
```bash
source {python_venv}/bin/activate

colcon build
source ./install/setup.zsh

cd OSTrack
python track.py

ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.py
ros2 launch piper start_single_piper.launch.py
ros2 launch lslidar_driver_n10p lslidar_launch.py
ros2 launch lidar_control lidar_control.py
ros2 launch total_control total_control_launch.py
```

## 功能包
### src/lidar_control
- `lidar_control` 是根据雷达数据进行控制的功能包

### src/lslidar_driver_n10p
- `lslidar_driver_n10p` 是官方提供的雷达驱动功能包

### src/lslidar_msgs
- `lslidar_msgs` 是官方提供的雷达消息功能包

### src/piper
- `piper` 是github上松灵科技提供的机械臂控制demo

### src/piper_arm_control
- `piper_arm_control` 是根据机械臂数据进行控制的功能包

### src/piper_msgs
- `piper_msgs` 是github上松灵科技提供根据机械臂消息功能包

### src/serial
- `serial` 是github上提供的与ros1接口一致的ros2串口通信功能包

### src/turn_on_wheeltec_robot
- `turn_on_wheeltec_robot` 是wheeltec提供的启动机器人的功能包

## 控制程序需要订阅的话题
### /scan
- `msg: sensor_msgs/msg/LaserScan` 
- 由lslidar_driver_n10p发布, 包含雷达数据

### /joint_states_single
- `msg: sensor_msgs/msg/JointState`
- 由piper/piper/piper_ctrl_single_node发布, 包含机械臂关节数据

### /arm_status
- `msg: piper_msgs/msg/ArmStatus`
- 由piper/piper/piper_ctrl_single_node发布, 包含机械臂通讯状态,角度限制数据

### /end_pose
- `msg: geometry_msgs/msg/EndPose`
- 由piper/piper/piper_ctrl_single_node发布, 包含机械臂末端位姿数据

### /PowerVoltage
- `msg: std_msgs/msg/Float32`
- 由turn_on_wheeltec_robot发布, 包含机械臂电压数据

### /odom
- `msg: nav_msgs/msg/Odometry`
- 由turn_on_wheeltec_robot发布, 包含机器人里程计数据, 包含位置、姿态、三轴速度、绕三轴角速度、TF父子坐标、协方差矩阵

### /imu
- `msg: sensor_msgs/msg/Imu`
- 由turn_on_wheeltec_robot发布, 包含机器人IMU数据

### /track_result
- `msg: std_msgs/msg/Float64MultiArray`
- 由OSTrack发布, 包含跟踪结果数据

### /bottom_arm_angle
- `msg: std_msgs/msg/Float64`
- 由total_control发布, 包含底部机械臂角度数据

### /total_cmd
- `msg: std_msgs/msg/Int8`
- 由用户发布, 包含整个机器人的启动和停止指令

## 控制程序需要发布的话题
### /pos_cmd
- `msg: piper_msgs/msg/PosCmd`
- 由piper/piper/piper_ctrl_single_node订阅

### /joint_states
- `msg: sensor_msgs/msg/JointState`
- 由piper/piper/piper_ctrl_single_node订阅

### /enable_flag
- `msg: std_msgs/msg/Bool`
- 由piper/piper/piper_ctrl_single_node订阅

### /gripper_ctrl
- `msg: std_msgs/msg/Float64`
- 由piper/piper/piper_ctrl_single_node订阅

### /cmd_vel
- `msg: geometry_msgs/msg/Twist`
- 由turn_on_wheeltec_robot订阅

### /bottom_arm_angle
- `msg: std_msgs/msg/Float64`
- 由total_control中底盘控制程序订阅

## 用户手机发送的流程控制指令
### /total_cmd
- `msg: std_msgs/msg/Int8`
- 由total_control订阅, 用于控制整个机器人的启动和停止
- 1: 机械臂位姿初始化
- 2: 机械臂加爪闭合
- 3: 程序启动
- 8: 机械臂加爪张开
- 9: 程序停止
