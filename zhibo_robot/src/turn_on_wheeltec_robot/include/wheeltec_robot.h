#ifndef WHEELTEC_ROBOT_H_
#define WHEELTEC_ROBOT_H_

#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <cstring>
#include <string>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>
#include <cstdbool>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

//Macro definition
//宏定义
constexpr uint8_t SEND_DATA_CHECK = 1;          //Send data check flag bits //发送数据校验标志位
constexpr uint8_t READ_DATA_CHECK = 0;          //Receive data to check flag bits //接收数据校验标志位
constexpr uint8_t FRAME_HEADER = 0X7B;       //Frame head //帧头
constexpr uint8_t FRAME_TAIL = 0X7D;       //Frame tail //帧尾
constexpr int RECEIVE_DATA_SIZE = 24;         //The length of the data sent by the lower computer //下位机发送过来的数据的长度
constexpr int SEND_DATA_SIZE = 11;         //The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define PI                  3.1415926f //PI //圆周率

//Relative to the range set by the IMU gyroscope, the range is ±500°, corresponding data range is ±32768
//The gyroscope raw data is converted in radian (rad) units, 1/65.5/57.30=0.00026644
//与IMU陀螺仪设置的量程有关，量程±500°，对应数据范围±32768
//陀螺仪原始数据转换位弧度(rad)单位，1/65.5/57.30=0.00026644
constexpr double GYROSCOPE_RATIO = 0.00026644f;
//Relates to the range set by the IMU accelerometer, range is ±2g, corresponding data range is ±32768
//Accelerometer original data conversion bit m/s^2 units, 32768/2g=32768/19.6=1671.84	
//与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
//加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
constexpr double ACCEl_RATIO = 1671.84f;

//Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
//协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
constexpr double ODOM_POSE_COVARIANCE[36] = {1e-3, 0, 0, 0, 0, 0,
                                             0, 1e-3, 0, 0, 0, 0,
                                             0, 0, 1e6, 0, 0, 0,
                                             0, 0, 0, 1e6, 0, 0,
                                             0, 0, 0, 0, 1e6, 0,
                                             0, 0, 0, 0, 0, 1e3};

constexpr double ODOM_POSE_COVARIANCE2[36] = {1e-9, 0, 0, 0, 0, 0,
                                              0, 1e-3, 1e-9, 0, 0, 0,
                                              0, 0, 1e6, 0, 0, 0,
                                              0, 0, 0, 1e6, 0, 0,
                                              0, 0, 0, 0, 1e6, 0,
                                              0, 0, 0, 0, 0, 1e-9};

constexpr double ODOM_TWIST_COVARIANCE[36] = {1e-3, 0, 0, 0, 0, 0,
                                              0, 1e-3, 0, 0, 0, 0,
                                              0, 0, 1e6, 0, 0, 0,
                                              0, 0, 0, 1e6, 0, 0,
                                              0, 0, 0, 0, 1e6, 0,
                                              0, 0, 0, 0, 0, 1e3};

constexpr double ODOM_TWIST_COVARIANCE2[36] = {1e-9, 0, 0, 0, 0, 0,
                                               0, 1e-3, 1e-9, 0, 0, 0,
                                               0, 0, 1e6, 0, 0, 0,
                                               0, 0, 0, 1e6, 0, 0,
                                               0, 0, 0, 0, 1e6, 0,
                                               0, 0, 0, 0, 0, 1e-9};

//Data structure for speed and position
//速度、位置数据结构体
typedef struct VelPosData_ {
    float x;
    float y;
    float z;
} VelPosData;

//IMU data structure
//IMU数据结构体
typedef struct Mpu6050Data_ {
    int16_t accele_xData;
    int16_t accele_yData;
    int16_t accele_zData;
    int16_t gyros_xData;
    int16_t gyros_yData;
    int16_t gyros_zData;
} Mpu6050Data;

//The structure of the ROS to send data to the down machine
//ROS向下位机发送数据的结构体
typedef struct SendData_ {
    uint8_t tx[SEND_DATA_SIZE];
    float x_speed;
    float y_speed;
    float z_speed;
    uint8_t frameTail;
} SendData;

//The structure in which the lower computer sends data to the ROS
//下位机向ROS发送数据的结构体
typedef struct ReceiveData_ {
    uint8_t rx[RECEIVE_DATA_SIZE];
    uint8_t flagStop;
    uint8_t frameHeader;
    float x_speed;
    float y_speed;
    float z_speed;
    float powerVoltage;
    unsigned char frameTail;
} ReceiveData;

//The robot chassis class uses constructors to initialize data, publish topics, etc
//机器人底盘类，使用构造函数初始化数据和发布话题等
class TurnOnRobot {
public:
    TurnOnRobot();  //Constructor //构造函数
    ~TurnOnRobot(); //Destructor //析构函数
    void control();   //Loop control code //循环控制代码

private:
    rclcpp::Node::SharedPtr m_node; //Create a ROS node handle //创建ROS节点句柄
    rclcpp::Time m_now, m_lastTime;  //Time dependent, used for integration to find displacement (mileage) //时间相关，用于积分求位移(里程)
    float m_samplingTime;         //Sampling time, used for integration to find displacement (mileage) //采样时间，用于积分求位移(里程)

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelSub; //Initialize the topic subscriber //初始化话题订阅者
    //The speed topic subscribes to the callback function
    //速度话题订阅回调函数
    void cmdVelCallback(const geometry_msgs::msg::Twist &twistAux);

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odomPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_voltagePublisher; //Initialize the topic publisher //初始化话题发布者
    void publishOdom();      //Pub the speedometer topic //发布里程计话题
    void publishImuSensor(); //Pub the IMU sensor topic //发布IMU传感器话题
    void publishVoltage();   //Pub the power supply voltage topic //发布电源电压话题

    //从串口(ttyUSB)读取运动底盘速度、IMU、电源电压数据
    //Read motion chassis speed, IMU, power supply voltage data from serial port (ttyUSB)
    [[maybe_unused]] bool getSensorData();

    bool getSensorDataNew();

    unsigned char checkSum(uint8_t countNumber, uint8_t mode); //BBC check function //BBC校验函数
    static int16_t imuTrans(uint8_t dataHigh, uint8_t dataLow);  //IMU data conversion read //IMU数据转化读取
    static float odomTrans(uint8_t dataHigh, uint8_t dataLow); //Odometer data is converted to read //里程计数据转化读取

    std::string m_uartPortName, m_robotFrameId, m_gyroFrameId, m_odomFrameId; //Define the related variables //定义相关变量
    uint32_t m_serialBaudRate;      //Serial communication baud rate //串口通信波特率
    ReceiveData m_receiveData; //The serial port receives the data structure //串口接收数据结构体
    SendData m_sendData;       //The serial port sends the data structure //串口发送数据结构体

    VelPosData m_robotPos;    //The position of the robot //机器人的位置
    VelPosData m_robotVel;    //The speed of the robot //机器人的速度
    Mpu6050Data m_mpu6050Data; //IMU data //IMU数据
    float m_powerVoltage;       //Power supply voltage //电源电压
    float m_odom_xScale, m_odom_yScale, m_odom_zScalePositive, m_odom_zScaleNegative;       //Odometer correction parameters //里程计修正参数
    serial::Serial m_stm32Serial; //Declare a serial object //声明串口对象
};

#endif
