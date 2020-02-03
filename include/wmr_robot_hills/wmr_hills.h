/*
    SOSCO, Wheeled Mobile Robot Hills (wmr-robot-hills)
    Copyright (C) 2019  Seyed Amir Shariati

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file /include/wmr-robot-hills/wmr_hills.h
 *
 * @brief Robot_Communication!
 *
 * @date December 2018
 **/

#ifndef WMR_HILLS
#define WMR_HILLS



#include <stdint.h>
#include <iostream>
#include <algorithm>

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>



//#include "wmr-robot-hills/motor_cmd.h"
#include "wmr-robot-hills/movement_data.h"
#include "wmr-robot-hills/odometry_data.h"
#include "wmr-robot-hills/pid_data.h"





#define SENSOR_DATA_PORT   5002   //Robot to Pc
#define PID_DATA_PORT       10121  //Robot to Pc
#define ODOM_DATA_PORT 5003   //Robot to Pc
#define MOVEMENT_DATA_PORT  3023   //Robot to Pc
#define MOVEMENT_CMD_PORT   6000   //PC to Robot
#define MOTOR_CMD_PORT  2222   //PC to Robot

#define ROBOT_IP "192.168.1.1"
#define HOST_IP "192.168.1.20"


#define IR_SENSOR_NUM  16;
#define ULTRASONIC_SENSOR_NUM  16


using boost::asio::ip::udp;
using boost::asio::ip::address;
/*** define udp class ***/


typedef uint8_t Byte;

union float2byte{
    float float_;
    u_char byte[4];
};


// #pragma pack(push, 1) // exact fit - no padding
typedef struct __attribute__((packed))Sensor_Data_Type{

    public:
    Byte Ultrasonic_1;
    Byte Ultrasonic_2;
    Byte Ultrasonic_3;
    Byte Ultrasonic_4;
    Byte Ultrasonic_5;
    Byte Ultrasonic_6;
    Byte Ultrasonic_7;
    Byte Ultrasonic_8;
    Byte Ultrasonic_9;
    Byte Ultrasonic_10;
    Byte Ultrasonic_11;
    Byte Ultrasonic_12;
    Byte Ultrasonic_13;
    Byte Ultrasonic_14;
    Byte Ultrasonic_15;
    Byte Ultrasonic_16;
    Byte IR_Sharp_1;
    Byte IR_Sharp_2;
    Byte IR_Sharp_3;
    Byte IR_Sharp_4;
    Byte IR_Sharp_5;
    Byte IR_Sharp_6;
    Byte IR_Sharp_7;
    Byte IR_Sharp_8;
    Byte IR_Sharp_9;
    Byte IR_Sharp_10;
    Byte IR_Sharp_11;
    Byte IR_Sharp_12;
    Byte IR_Sharp_13;
    Byte IR_Sharp_14;
    Byte IR_Sharp_15;
    Byte IR_Sharp_16;
} Sensor_Data_Type;
//  #pragma pack(pop) //back to whatever the previous packing mode was


// #pragma pack(push, 1) // exact fit - no padding
typedef struct __attribute__((packed))PID_Data_Type{

    public:
    float UP_REF_M1;
    float UI_REF_M1;
    Byte Err_ENcoder_M1_1;
    Byte Err_ENcoder_M1_2;
    Byte Err_ENcoder_M1_3;
    Byte Err_ENcoder_M1_4;
    float UP_REF_M2;
    float UI_REF_M2;
    Byte Err_ENcoder_M2_1;
    Byte Err_ENcoder_M2_2;
    Byte Err_ENcoder_M2_3;
    Byte Err_ENcoder_M2_4;
}PID_Data_Type;
//  #pragma pack(pop) //back to whatever the previous packing mode was

// #pragma pack(push, 1) // exact fit - no padding
typedef struct __attribute__((packed))Odometery_Data_Type{

    public:
    Byte RES_1;
    Byte RES_2;
    Byte RES_3;
    Byte RES_4;
    Byte RES_5;
    Byte RES_6;
    Byte RES_7;
    Byte RES_8;
    Byte RES_9;
    Byte RES_10;
    Byte T_1;
    Byte T_2;
    Byte T_3;
    Byte T_4;
    Byte x_1;
    Byte x_2;
    Byte x_3;
    Byte x_4;
    Byte y_1;
    Byte y_2;
    Byte y_3;
    Byte y_4;
    Byte s_r_1;
    Byte s_r_2;
    Byte s_r_3;
    Byte s_r_4;
    Byte s_l_1;
    Byte s_l_2;
    Byte s_l_3;
    Byte s_l_4;

    /*
    float Result_Teta;
    float Result_X;
    float Result_Y;
    float Speed_Motor_R;
    float Speed_Motor_L;
    */
}Odometery_Data_Type;
//  #pragma pack(pop) //back to whatever the previous packing mode was

// #pragma pack(push, 1) // exact fit - no padding
typedef struct __attribute__((packed))Movement_Data_Type{

    public:
    Byte byte_1;
    Byte Direction_R;
    Byte Direction_L;
    Byte Pulse_MR_LSB;
    Byte Pulse_MR_MSB;
    Byte Pulse_ML_LSB;
    Byte Pulse_ML_MSB;
    Byte Feed_Dir_R;
    Byte Feed_Dir_L;
}Movement_Data_Type;
//  #pragma pack(pop) //back to whatever the previous packing mode was

// #pragma pack(push, 1) // exact fit - no padding
typedef struct __attribute__((packed))Movement_Cmd_Type{

    public:
    const Byte Mode = 0x04;
   // Byte Mode;
    float Linear_Velocity;
    float Angular_Velocity;
}Movement_Cmd_Type;
//  #pragma pack(pop) //back to whatever the previous packing mode was

// #pragma pack(push, 1) // exact fit - no padding
typedef struct __attribute__((packed))Motor_Cmd_Type{

    public:
    Byte Mode; // 0x00-0x02
    Byte DirR; //0x00-0x02;
    Byte DirL;  //0x00-0x02;
    uint16_t SpeedR; //1-1700;
    uint16_t SpeedL; //1-1700;
}Motor_Cmd_Type;
//  #pragma pack(pop) //back to whatever the previous packing mode was



union sensor_data_union
{
    /***
     * 1  - 16 sonar
     * 17 - 32 IR
     ***/
    Sensor_Data_Type Sensor_Data_buf_u;
    u_char byte_data[32];
};









class wmr_robot
{
public:

    wmr_robot(boost::asio::io_service &io_service);
    ~wmr_robot();

    void run();

private:
    void wmr_robot_init();

    void start_sensor_receive();
    void start_odom_receive();
    void start_pid_receive();
    void start_move_data_receive();

    void handle_sensor_receive(const boost::system::error_code& error, std::size_t /*bytes_transferred*/);
    void handle_odom_receive(const boost::system::error_code& error, std::size_t /*bytes_transferred*/);
    void handle_pid_receive(const boost::system::error_code& error, std::size_t /*bytes_transferred*/);
    void handle_move_data_receive(const boost::system::error_code& error, std::size_t /*bytes_transferred*/);



    /*** ROS functions ***/
    void send_cmd_vel(float linear_velocity, float angular_velocity);

    void send_motor_cmd(Motor_Cmd_Type motor_cmd);
    //void send_motor_cmd(Byte mode, Byte Dir_Right, Byte Dir_Left, uint16_t Speed_Right, uint16_t Speed_Left);

    void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg);
    //void callback_motor_cmd(const wmr-robot-hills::motor_cmd& msg);

    void odometry_node();
    void pid_node();
    void move_data_node();
    void sensor_node();




    udp::endpoint robot_endpoint_;


    udp::socket socket_sensor_;
    udp::socket socket_pid_;
    udp::socket socket_odom_;
    udp::socket socket_move_data_;

    udp::socket socket_move_cmd_;
    udp::socket socket_motor_cmd_;

    /*boost::array<char, 32> recv_sensor_buffer_;
    boost::array<char, 30> recv_odom_buffer_;
    boost::array<char, 24> recv_pid_buffer_;
    boost::array<char, 9>  recv_move_data_buffer_;*/


    ros::Subscriber sub_motor_cmd_;
    ros::Subscriber sub_cmd_vel_;

    ros::Publisher pub_PID_Data_;
    ros::Publisher pub_odometry_Data_;
    ros::Publisher pub_Movement_data_;
    ros::Publisher pub_Sonar_;
    ros::Publisher pub_IR_Sharp_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_sub_;

    ros::Time current_time_ ;
    ros::Time last_time_ ;

    double X,Y,th;
    //tf2_ros::TransformBroadcaster odom_broadcaster_;
    tf::TransformBroadcaster transform_broadcaster_;
    geometry_msgs::TransformStamped odom_trans_;
    geometry_msgs::Quaternion odom_quat_ ;
    nav_msgs::Odometry odom_;

    //tf2::Quaternion odom_quat_;



    Sensor_Data_Type Sensor_Data_buf_;
    PID_Data_Type PID_Data_buf_;
    Odometery_Data_Type Odometery_Data_buf_;
    Movement_Data_Type Movement_Data_buf_;

    Movement_Cmd_Type Movement_cmd_buf_;
    Motor_Cmd_Type Motor_cmd_buf_;


};









#endif // WMR_HILLS
