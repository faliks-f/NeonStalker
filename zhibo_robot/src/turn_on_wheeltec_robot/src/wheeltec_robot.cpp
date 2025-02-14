#include "quaternion_solution.h"
#include "wheeltec_robot.h"

sensor_msgs::msg::Imu Mpu6050;

/**************************************
Function: Data conversion function
功能: 数据转换函数
***************************************/
int16_t TurnOnRobot::imuTrans(uint8_t dataHigh, uint8_t dataLow) {
    return static_cast<int16_t>((dataHigh << 8) + dataLow);
}

float TurnOnRobot::odomTrans(uint8_t dataHigh, uint8_t dataLow) {
    int16_t trans = imuTrans(dataHigh, dataLow);
    return static_cast<float>(static_cast<int16_t>(trans / 1000) + (trans % 1000) * 0.001);
}

/**************************************
Function: The speed topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
功能: 速度话题订阅回调函数Callback，根据订阅的指令通过串口发指令控制下位机
***************************************/
void TurnOnRobot::cmdVelCallback(const geometry_msgs::msg::Twist &twistAux) {
//    std::cout << "x: " << twistAux.linear.x << " y: " << twistAux.linear.y << " z: " << twistAux.angular.z << std::endl;
    int16_t transition = 0;
    m_sendData.tx[0] = FRAME_HEADER;    // frame head 0x7B // 帧头0X7B
    m_sendData.tx[1] = 0;
    m_sendData.tx[2] = 0;

    //The target velocity of the X-axis of the robot
    //机器人x轴的目标线速度
    transition = static_cast<int16_t>(twistAux.linear.x * 1000); //将浮点数放大一千倍，简化传输

    m_sendData.tx[4] = transition & 0xff;     //取数据的低8位
    m_sendData.tx[3] = transition >> 8;  //取数据的高8位

    //The target velocity of the Y-axis of the robot
    //机器人y轴的目标线速度
    transition = static_cast<int16_t>(twistAux.linear.y * 1000);
    if (transition < 0) {
        transition += 65536; //如果是负数，转换成无符号数
    }
    m_sendData.tx[6] = transition & 0xff;
    m_sendData.tx[5] = transition >> 8;

    //The target angular velocity of the robot's Z axis
    //机器人z轴的目标角速度
    transition = static_cast<int16_t>(twistAux.angular.z * 1000);
    if (transition < 0) {
        transition += 65536; //如果是负数，转换成无符号数
    }
    m_sendData.tx[8] = transition & 0xff;
    m_sendData.tx[7] = transition >> 8;

    //For the BCC check bits, see the Check_Sum function //BCC校验位，规则参见Check_Sum函数
    m_sendData.tx[9] = checkSum(9, SEND_DATA_CHECK);
    m_sendData.tx[10] = FRAME_TAIL; //frame tail 0x7D //帧尾0X7D

    try {
        m_stm32Serial.write(m_sendData.tx, SEND_DATA_SIZE);
    } catch (serial::IOException &e) {
        //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Unable to send data through serial port");
    }
}

/**************************************
Function: Publish the IMU data topic
功能: 发布IMU数据话题
***************************************/
void TurnOnRobot::publishImuSensor() {
    sensor_msgs::msg::Imu imuDataPub; // Instantiate IMU topic data //实例化IMU话题数据
    imuDataPub.header.stamp = m_node->get_clock()->now();
    //IMU corresponds to TF coordinates, which is required to use the robot_pose_ekf feature pack
    //IMU对应TF坐标，使用robot_pose_ekf功能包需要设置此项
    imuDataPub.header.frame_id = m_gyroFrameId;

    imuDataPub.orientation.x = Mpu6050.orientation.x; // A quaternion represents a three-axis attitude //四元数表达三轴姿态
    imuDataPub.orientation.y = Mpu6050.orientation.y;
    imuDataPub.orientation.z = Mpu6050.orientation.z;
    imuDataPub.orientation.w = Mpu6050.orientation.w;
    // Three-axis attitude covariance matrix //三轴姿态协方差矩阵
    imuDataPub.orientation_covariance[0] = 1e6;
    imuDataPub.orientation_covariance[4] = 1e6;
    imuDataPub.orientation_covariance[8] = 1e-6;

    imuDataPub.angular_velocity.x = Mpu6050.angular_velocity.x; // Triaxial angular velocity //三轴角速度
    imuDataPub.angular_velocity.y = Mpu6050.angular_velocity.y;
    imuDataPub.angular_velocity.z = Mpu6050.angular_velocity.z;
    // Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
    imuDataPub.angular_velocity_covariance[0] = 1e6;
    imuDataPub.angular_velocity_covariance[4] = 1e6;

    imuDataPub.linear_acceleration.x = Mpu6050.linear_acceleration.x; // Triaxial acceleration //三轴线性加速度
    imuDataPub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
    imuDataPub.linear_acceleration.z = Mpu6050.linear_acceleration.z;

    m_imuPublisher->publish(imuDataPub);
}

/**************************************
Function: Publish the odometer topic, Contains position, attitude, triaxial velocity, angular velocity about triaxial, TF parent-child coordinates, and covariance matrix
功能: 发布里程计话题，包含位置、姿态、三轴速度、绕三轴角速度、TF父子坐标、协方差矩阵
***************************************/
void TurnOnRobot::publishOdom() {
    tf2::Quaternion odomQuat;
    odomQuat.setRPY(0, 0, m_robotPos.z);
    geometry_msgs::msg::Quaternion odomQuatMsg = tf2::toMsg(odomQuat);

    nav_msgs::msg::Odometry odom; // Instance the odometer topic data //实例化里程计话题数据
    odom.header.stamp = m_node->get_clock()->now();
    odom.header.frame_id = m_odomFrameId; // Odometer TF parent coordinates //里程计TF父坐标
    odom.pose.pose.position.x = m_robotPos.x;
    odom.pose.pose.position.y = m_robotPos.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odomQuatMsg; // Posture, Quaternion converted by Z-axis rotation //姿态，通过Z轴转角转换的四元数

    odom.child_frame_id = m_robotFrameId; // Odometer TF subcoordinates //里程计TF子坐标
    odom.twist.twist.linear.x = m_robotVel.x; // Speed in the X direction //X方向速度
    odom.twist.twist.linear.y = m_robotVel.y; // Speed in the Y direction //Y方向速度
    odom.twist.twist.angular.z = m_robotVel.z; // Angular velocity around the Z axis //绕Z轴角速度

    // There are two types of this matrix, which are used when the robot is at rest and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
    // 这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
    if (m_robotVel.x == 0 && m_robotVel.y == 0 && m_robotVel.z == 0) {
        // If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
        // 如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
        memcpy(&odom.pose.covariance, ODOM_POSE_COVARIANCE2, sizeof(ODOM_POSE_COVARIANCE2));
        memcpy(&odom.twist.covariance, ODOM_TWIST_COVARIANCE2, sizeof(ODOM_TWIST_COVARIANCE2));
    } else {
        // If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
        // 如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
        memcpy(&odom.pose.covariance, ODOM_POSE_COVARIANCE, sizeof(ODOM_POSE_COVARIANCE));
        memcpy(&odom.twist.covariance, ODOM_TWIST_COVARIANCE, sizeof(ODOM_TWIST_COVARIANCE));
    }
    m_odomPublisher->publish(odom); // Pub odometer topic //发布里程计话题
}

void TurnOnRobot::publishVoltage() {
    std_msgs::msg::Float32 voltageMsg;
    static int count = 0;
    if (count++ > 10) {
        count = 0;
        voltageMsg.data = m_powerVoltage;
        m_voltagePublisher->publish(voltageMsg);
    }
}

/**************************************
Function: Serial port communication check function, packet n has a byte, the NTH -1 byte is the check bit, the NTH byte bit frame end.Bit XOR results from byte 1 to byte n-2 are compared with byte n-1, which is a BCC check
Input parameter: Count_Number: Check the first few bytes of the packet
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BCC校验
输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/
unsigned char TurnOnRobot::checkSum(uint8_t countNumber, uint8_t mode) {
    uint8_t sum = 0;
    if (mode == 0) { //Receive data mode //接收数据模式
        for (int k = 0; k < countNumber; k++) {
            sum = sum ^ m_receiveData.rx[k];
        }
    } else if (mode == 1) { //Send data mode //发送数据模式
        for (int k = 0; k < countNumber; k++) {
            sum = sum ^ m_sendData.tx[k];
        }
    }
    return sum;
}

/**************************************
Function: The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
Update Note: This checking method can lead to read error data or correct data not to be processed. Instead of this checking method, frame-by-frame checking is now used.
             Refer to Get_ Sensor_ Data_ New() function
功能: 通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
更新说明：该校验方法会导致出现读取错误数据或者正确数据不处理的情况，现在已不用该校验方法，换成逐帧校验方式，getSensorDataNew()函数
***************************************/
bool TurnOnRobot::getSensorData() {
    uint16_t transition = 0, j = 0, headerPos = 0, tailPos = 0;
    static int flag_error = 0, temp = 1;
    std::vector<uint8_t> receiveDataPr(RECEIVE_DATA_SIZE, 0), receiveDataTemp(temp, 0);
    if (flag_error == 0) { //Normal condition detected //检测到正常情况
        m_stm32Serial.read(receiveDataPr, RECEIVE_DATA_SIZE);
    } else if (flag_error == 1) { //Error condition detected //检测到错误情况
        m_stm32Serial.read(receiveDataTemp, temp);
        flag_error = 0;
    }

    //Record the position of the head and tail of the frame //记录帧头帧尾位置
    for (j = 0; j < 24; j++) {
        if (receiveDataPr[j] == FRAME_HEADER || receiveDataPr[j] == FRAME_TAIL) {
            headerPos = j;
        }
    }

    if (tailPos == (headerPos + 23)) {
        //If the end of the frame is the last bit of the packet, copy the packet directly to m_receiveData.rx
        //如果帧尾在数据包最后一位，直接复制数据包到m_receiveData.rx
        // ROS_INFO("1-----");
        memcpy(m_receiveData.rx, receiveDataPr.data(), RECEIVE_DATA_SIZE);
        flag_error = 0; //Error flag position 0 for next reading //错误标志位置0，便于下次读取
    } else if (headerPos == (1 + tailPos)) {
        //If the header is behind the end of the frame, record the position of the header so that the next reading of the error bit data can correct the data position
        //如果帧头在帧尾后面，记录帧头出现的位置，便于下次读取出错位数据以纠正数据位置
        //|********7D (7B************|**********7D) 7B************|
        // ROS_INFO("2-----");
        temp = headerPos; //Record the length of the next read, calculated to be exactly the position of the frame head //记录下一次读取的长度，经计算正好为帧头的位置
        flag_error = 1; //Error flag position 1, error bit array for next read //错误标志位置1，让下一次读取出错位数组
        return false;
    } else {
        ////其它情况则认为数据包有错误，这种情况一般是正常的数据，但是除帧头帧尾在数据中间出现了7B或7D的数据
        // In other cases, the packet is considered to be faulty
        // This is generally normal data, but there is 7B or 7D data in the middle of the data except for the frame header and end.
        // ROS_INFO("3-----");
        return false;
    }

    m_receiveData.frameHeader = m_receiveData.rx[0]; //The first part of the data is the frame header 0X7B //数据的第一位是帧头0X7B
    m_receiveData.frameTail = m_receiveData.rx[23];  //The last bit of data is frame tail 0X7D //数据的最后一位是帧尾0X7D

    if (m_receiveData.frameHeader == FRAME_HEADER) //Judge the frame header //判断帧头
    {
        if (m_receiveData.frameTail == FRAME_TAIL) //Judge the end of the frame //判断帧尾
        {
            if (m_receiveData.rx[22] ==
                checkSum(22, READ_DATA_CHECK)) //BCC check passes or two packets are interlaced //BCC校验通过或者两组数据包交错
            {
                m_receiveData.flagStop = m_receiveData.rx[1]; //set aside //预留位
                m_robotVel.x = odomTrans(m_receiveData.rx[2],
                                         m_receiveData.rx[3]); //Get the speed of the moving chassis in the X direction //获取运动底盘X方向速度
                m_robotVel.y = odomTrans(m_receiveData.rx[4],
                                         m_receiveData.rx[5]); //Get the speed of the moving chassis in the Y direction, The Y speed is only valid in the omnidirectional mobile robot chassis
                //获取运动底盘Y方向速度，Y速度仅在全向移动机器人底盘有效
                m_robotVel.z = odomTrans(m_receiveData.rx[6],
                                         m_receiveData.rx[7]); //Get the speed of the moving chassis in the Z direction //获取运动底盘Z方向速度

                //MPU6050 stands for IMU only and does not refer to a specific model. It can be either MPU6050 or MPU9250
                //Mpu6050仅代表IMU，不指代特定型号，既可以是MPU6050也可以是MPU9250
                m_mpu6050Data.accele_xData = imuTrans(m_receiveData.rx[8],
                                                      m_receiveData.rx[9]);   //Get the X-axis acceleration of the IMU     //获取IMU的X轴加速度
                m_mpu6050Data.accele_yData = imuTrans(m_receiveData.rx[10],
                                                      m_receiveData.rx[11]); //Get the Y-axis acceleration of the IMU     //获取IMU的Y轴加速度
                m_mpu6050Data.accele_zData = imuTrans(m_receiveData.rx[12],
                                                      m_receiveData.rx[13]); //Get the Z-axis acceleration of the IMU     //获取IMU的Z轴加速度
                m_mpu6050Data.gyros_xData = imuTrans(m_receiveData.rx[14],
                                                     m_receiveData.rx[15]);  //Get the X-axis angular velocity of the IMU //获取IMU的X轴角速度
                m_mpu6050Data.gyros_yData = imuTrans(m_receiveData.rx[16],
                                                     m_receiveData.rx[17]);  //Get the Y-axis angular velocity of the IMU //获取IMU的Y轴角速度
                m_mpu6050Data.gyros_zData = imuTrans(m_receiveData.rx[18],
                                                     m_receiveData.rx[19]);  //Get the Z-axis angular velocity of the IMU //获取IMU的Z轴角速度
                //Linear acceleration unit conversion is related to the range of IMU initialization of STM32, where the range is ±2g=19.6m/s^2
                //线性加速度单位转化，和STM32的IMU初始化的时候的量程有关,这里量程±2g=19.6m/s^2
                Mpu6050.linear_acceleration.x = m_mpu6050Data.accele_xData / ACCEl_RATIO;
                Mpu6050.linear_acceleration.y = m_mpu6050Data.accele_yData / ACCEl_RATIO;
                Mpu6050.linear_acceleration.z = m_mpu6050Data.accele_zData / ACCEl_RATIO;
                //The gyroscope unit conversion is related to the range of STM32's IMU when initialized. Here, the range of IMU's gyroscope is ±500°/s
                //Because the robot generally has a slow Z-axis speed, reducing the range can improve the accuracy
                //陀螺仪单位转化，和STM32的IMU初始化的时候的量程有关，这里IMU的陀螺仪的量程是±500°/s
                //因为机器人一般Z轴速度不快，降低量程可以提高精度
                Mpu6050.angular_velocity.x = m_mpu6050Data.gyros_xData * GYROSCOPE_RATIO;
                Mpu6050.angular_velocity.y = m_mpu6050Data.gyros_yData * GYROSCOPE_RATIO;
                Mpu6050.angular_velocity.z = m_mpu6050Data.gyros_zData * GYROSCOPE_RATIO;

                //Get the battery voltage
                //获取电池电压
                transition = 0;
                transition |= m_receiveData.rx[20] << 8;
                transition |= m_receiveData.rx[21];
                //Unit conversion millivolt(mv)->volt(v) //单位转换毫伏(mv)->伏(v)
                m_powerVoltage = static_cast<float>(static_cast<uint16_t>(transition / 1000) +
                                                    (transition % 1000) * 0.001);

                return true;
            }
        }
    }
    return false;
}

/**************************************
Function: Read and verify the data sent by the lower computer frame by frame through the serial port, and then convert the data into international units
功能: 通过串口读取并逐帧校验下位机发送过来的数据，然后数据转换为国际单位
***************************************/
bool TurnOnRobot::getSensorDataNew() {
    int16_t transition_16 = 0;
    uint8_t receiveDataPr[1];
    static int count = 0;
    m_stm32Serial.read(receiveDataPr, sizeof(receiveDataPr));

    m_receiveData.rx[count] = receiveDataPr[0];
    m_receiveData.frameHeader = m_receiveData.rx[0];
    m_receiveData.frameTail = m_receiveData.rx[23];

    if (receiveDataPr[0] == FRAME_HEADER || count > 0) {
        // If the frame header is detected or the count is greater than 0
        count++;
    } else {
        count = 0;
    }
    if (count == 24) {
        count = 0;
        if (m_receiveData.frameTail == FRAME_TAIL) {
            if (m_receiveData.rx[22] == checkSum(22, READ_DATA_CHECK)) {
                m_receiveData.flagStop = m_receiveData.rx[1];
                m_robotVel.x = odomTrans(m_receiveData.rx[2], m_receiveData.rx[3]);
                m_robotVel.y = odomTrans(m_receiveData.rx[4], m_receiveData.rx[5]);
                m_robotVel.z = odomTrans(m_receiveData.rx[6], m_receiveData.rx[7]);

                m_mpu6050Data.accele_xData = imuTrans(m_receiveData.rx[8], m_receiveData.rx[9]);
                m_mpu6050Data.accele_yData = imuTrans(m_receiveData.rx[10], m_receiveData.rx[11]);
                m_mpu6050Data.accele_zData = imuTrans(m_receiveData.rx[12], m_receiveData.rx[13]);
                m_mpu6050Data.gyros_xData = imuTrans(m_receiveData.rx[14], m_receiveData.rx[15]);
                m_mpu6050Data.gyros_yData = imuTrans(m_receiveData.rx[16], m_receiveData.rx[17]);
                m_mpu6050Data.gyros_zData = imuTrans(m_receiveData.rx[18], m_receiveData.rx[19]);

                //Linear acceleration unit conversion is related to the range of IMU initialization of STM32, where the range is ±2g=19.6m/s^2
                //线性加速度单位转化，和STM32的IMU初始化的时候的量程有关,这里量程±2g=19.6m/s^2
                Mpu6050.linear_acceleration.x = m_mpu6050Data.accele_xData / ACCEl_RATIO;
                Mpu6050.linear_acceleration.y = m_mpu6050Data.accele_yData / ACCEl_RATIO;
                Mpu6050.linear_acceleration.z = m_mpu6050Data.accele_zData / ACCEl_RATIO;

                //The gyroscope unit conversion is related to the range of STM32's IMU when initialized. Here, the range of IMU's gyroscope is ±500°/s
                //Because the robot generally has a slow Z-axis speed, reducing the range can improve the accuracy
                //陀螺仪单位转化，和STM32的IMU初始化的时候的量程有关，这里IMU的陀螺仪的量程是±500°/s
                //因为机器人一般Z轴速度不快，降低量程可以提高精度
                Mpu6050.angular_velocity.x = m_mpu6050Data.gyros_xData * GYROSCOPE_RATIO;
                Mpu6050.angular_velocity.y = m_mpu6050Data.gyros_yData * GYROSCOPE_RATIO;
                Mpu6050.angular_velocity.z = m_mpu6050Data.gyros_zData * GYROSCOPE_RATIO;

                //Get the battery voltage
                //获取电池电压
                transition_16 = static_cast<int16_t> ((m_receiveData.rx[20] << 8) + m_receiveData.rx[21]);
                m_powerVoltage = static_cast<float>(static_cast<uint16_t>(transition_16 / 1000) +
                                                    (transition_16 % 1000) * 0.001);
                return true;
            }
        }
    }
    return false;
}

/**************************************
Function: Loop access to the lower computer data and issue topics
功能: 循环获取下位机数据与发布话题
***************************************/
void TurnOnRobot::control() {
    while (rclcpp::ok()) {
        if (getSensorDataNew()) {
            m_now = m_node->get_clock()->now();
            if (m_lastTime.nanoseconds() == 0) {
                m_lastTime = m_now;
            }
            m_samplingTime = static_cast<float>((m_now - m_lastTime).seconds());

            //Odometer correction parameters
            //里程计误差修正
            m_robotVel.x = m_robotVel.x * m_odom_xScale;
            m_robotVel.y = m_robotVel.y * m_odom_yScale;
            if (m_robotVel.z >= 0) {
                m_robotVel.z = m_robotVel.z * m_odom_zScalePositive;
            } else {
                m_robotVel.z = m_robotVel.z * m_odom_zScaleNegative;
            }
            //Speed * Time = displacement (odometer)
            //速度*时间=位移（里程计）
            m_robotPos.x +=
                    (m_robotVel.x * std::cos(m_robotPos.z) - m_robotVel.y * std::sin(m_robotPos.z)) * m_samplingTime;
            m_robotPos.y +=
                    (m_robotVel.x * std::sin(m_robotPos.z) + m_robotVel.y * std::cos(m_robotPos.z)) * m_samplingTime;
            m_robotPos.z += m_robotVel.z * m_samplingTime;

            //Calculate the three-axis attitude from the IMU with the angular velocity around the three-axis and the three-axis acceleration
            //通过IMU绕三轴角速度与三轴加速度计算三轴姿态
            quaternionSolution(static_cast<float>(Mpu6050.angular_velocity.x),
                               static_cast<float>(Mpu6050.angular_velocity.y),
                               static_cast<float>(Mpu6050.angular_velocity.z),
                               static_cast<float>(Mpu6050.linear_acceleration.x),
                               static_cast<float>(Mpu6050.linear_acceleration.y),
                               static_cast<float>(Mpu6050.linear_acceleration.z));

            publishOdom();
            publishImuSensor();
            publishVoltage();

            m_lastTime = m_now;
        }
        rclcpp::spin_some(m_node);
    }
}

TurnOnRobot::TurnOnRobot() : m_samplingTime(0),
                             m_powerVoltage(0) {
    //Clear the data
    //清空数据
    m_robotPos = {};
    m_robotVel = {};
    m_receiveData = {};
    m_sendData = {};
    m_mpu6050Data = {};

    // init m_node
    m_node = rclcpp::Node::make_shared("wheeltec_robot");

    m_node->declare_parameter<std::string>("usart_port_name", "/dev/wheeltec_controller");
    m_node->declare_parameter<int>("serial_baud_rate", 115200);
    m_node->declare_parameter<std::string>("odom_frame_id", "odom_combined");
    m_node->declare_parameter<std::string>("robot_frame_id", "base_footprint");
    m_node->declare_parameter<std::string>("gyro_frame_id", "gyro_link");

    m_node->declare_parameter<float>("odom_x_scale", 1.0);
    m_node->declare_parameter<float>("odom_y_scale", 1.0);
    m_node->declare_parameter<float>("odom_z_scale_positive", 1.0);
    m_node->declare_parameter<float>("odom_z_scale_negative", 1.0);

    m_node->get_parameter("usart_port_name", m_uartPortName);
    m_node->get_parameter("serial_baud_rate", m_serialBaudRate);
    m_node->get_parameter("odom_frame_id", m_odomFrameId);
    m_node->get_parameter("robot_frame_id", m_robotFrameId);
    m_node->get_parameter("gyro_frame_id", m_gyroFrameId);

    m_node->get_parameter("odom_x_scale", m_odom_xScale);
    m_node->get_parameter("odom_y_scale", m_odom_yScale);
    m_node->get_parameter("odom_z_scale_positive", m_odom_zScalePositive);
    m_node->get_parameter("odom_z_scale_negative", m_odom_zScaleNegative);

    m_voltagePublisher = m_node->create_publisher<std_msgs::msg::Float32>("PowerVoltage", 10);
    m_odomPublisher = m_node->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    m_imuPublisher = m_node->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

    m_cmdVelSub = m_node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
                                                                         [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                                                                             cmdVelCallback(*msg);
                                                                         });
    RCLCPP_INFO_STREAM(m_node->get_logger(), "Data ready");

    try {
        m_stm32Serial.setPort(m_uartPortName);
        m_stm32Serial.setBaudrate(m_serialBaudRate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(2000);
        m_stm32Serial.setTimeout(timeout);
        m_stm32Serial.open();
    } catch (serial::IOException &e) {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Unable to open serial port");
    }
    if (m_stm32Serial.isOpen()) {
        RCLCPP_INFO_STREAM(m_node->get_logger(), "Serial Port initialized");
    }
}

TurnOnRobot::~TurnOnRobot() {
    //Sends the stop motion command to the lower machine before the turn_on_robot object ends
    //对象turn_on_robot结束前向下位机发送停止运动命令
    m_sendData.tx[0] = FRAME_HEADER;
    m_sendData.tx[1] = 0;
    m_sendData.tx[2] = 0;

    m_sendData.tx[4] = 0;
    m_sendData.tx[3] = 0;

    m_sendData.tx[6] = 0;
    m_sendData.tx[5] = 0;

    m_sendData.tx[8] = 0;
    m_sendData.tx[7] = 0;

    m_sendData.tx[9] = checkSum(9, SEND_DATA_CHECK);
    m_sendData.tx[10] = FRAME_TAIL;

    try {
        m_stm32Serial.write(m_sendData.tx, SEND_DATA_SIZE);
    } catch (serial::IOException &e) {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Unable to send data through serial port");
    }

    m_stm32Serial.close();
    RCLCPP_INFO_STREAM(m_node->get_logger(), "Serial Port closed");
}

