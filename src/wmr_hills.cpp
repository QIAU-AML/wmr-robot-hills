#include "wmr-robot-hills/wmr_hills.h"


wmr_robot::wmr_robot(boost::asio::io_service& io_service )
    : socket_move_cmd_(io_service) ,
      socket_motor_cmd_(io_service) ,
      socket_sensor_(io_service, udp::endpoint( udp::v4(), SENSOR_DATA_PORT ) ) , /*socket_sensor_(io_service, {udp::v4(), 8888})*/
      socket_pid_( io_service, udp::endpoint( udp::v4(), PID_DATA_PORT ) ) ,
      socket_odom_( io_service, udp::endpoint( udp::v4(), ODOM_DATA_PORT ) ) ,
      socket_move_data_ ( io_service, udp::endpoint( udp::v4(), MOVEMENT_DATA_PORT ) )      
{

    /*** subscribers ***/
    //sub_motor_cmd_ = nh_.subscribe( "/motor_vel" , 1 , &callback_motor_cmd );
    sub_cmd_vel_   =  nh_sub_.subscribe<geometry_msgs::Twist>( "cmd_vel" , 1 , &wmr_robot::callback_cmd_vel , this );

    ROS_INFO_STREAM("\n .................... Robot subscriber .................... \n"
                    " Robot Velocity: "<<"  Topic ---> /cmd_vel"<<"\n"
                     <<"\n" <<"\n" );


    /*** publishers ***/
    pub_Sonar_     =  nh_.advertise<sensor_msgs::PointCloud>("/wmr_robot/sonar_raw",50);
    pub_IR_Sharp_  =  nh_.advertise<sensor_msgs::PointCloud>("/wmr_robot/ir_raw",50);
    pub_PID_Data_  =  nh_.advertise<wmr-robot-hills::pid_data>("/wmr_robot/pid",50);

    pub_odometry_Data_ =  nh_.advertise<nav_msgs::Odometry>("/wmr_robot/odometry",50);
    pub_Movement_data_ =  nh_.advertise<wmr-robot-hills::movement_data>("/wmr_robot/movement",50);


    ROS_INFO_STREAM("\n .................... Robot Publishers .................... \n"
                    " Sonar Sensor: "<<"    Topic ---> /wmr_robot/sonar_raw"<<"\n"<<
                    " IR_Sharp Sensor: "<<" Topic ---> /wmr_robot/ir_raw"<<"\n"<<
                    " PID Data: "<<"        Topic ---> /wmr_robot/pid"<<"\n"<<
                    " Odometery Data: "<<"  Topic ---> /wmr_robot/odometry"<<"\n"<<
                    " Movement Data: "<< "   Topic ---> /wmr_robot/movement"<<"\n" <<"\n" );


    wmr_robot_init();

}

wmr_robot::~wmr_robot()
{
    socket_move_cmd_.close();
    socket_motor_cmd_.close();
    socket_sensor_.close();
    socket_pid_.close();
    socket_odom_.close();
    socket_move_data_.close();
}

void wmr_robot::wmr_robot_init()
{
    send_cmd_vel( 0 , 0 );


    Motor_cmd_buf_.Mode = 0x01;
    Motor_cmd_buf_.DirR = 0x01; //0x02;
    Motor_cmd_buf_.DirL = 0x01; //0x02;
    Motor_cmd_buf_.SpeedR = 0; //1700;
    Motor_cmd_buf_.SpeedL = 0; //1700;

    send_motor_cmd(Motor_cmd_buf_);


    start_sensor_receive();
    start_pid_receive();
    start_odom_receive();
    start_move_data_receive();

}

void wmr_robot::run()
{
    //send_cmd_vel( 1.0 , 0.0 );

}

void wmr_robot::start_sensor_receive()
{
    socket_sensor_.async_receive_from( boost::asio::buffer( &Sensor_Data_buf_ , sizeof(Sensor_Data_buf_) ), robot_endpoint_ ,
                                       boost::bind( &wmr_robot::handle_sensor_receive , this ,
                                                    boost::asio::placeholders::error ,
                                                    boost::asio::placeholders::bytes_transferred) );
}

void wmr_robot::start_odom_receive()
{
    socket_odom_.async_receive_from( boost::asio::buffer( &Odometery_Data_buf_ , sizeof(Odometery_Data_buf_) ), robot_endpoint_ ,
                                       boost::bind( &wmr_robot::handle_odom_receive , this ,
                                                    boost::asio::placeholders::error ,
                                                    boost::asio::placeholders::bytes_transferred) );
}

void wmr_robot::start_pid_receive()
{
    socket_pid_.async_receive_from( boost::asio::buffer( &PID_Data_buf_ , sizeof(PID_Data_buf_) ), robot_endpoint_ ,
                                    boost::bind( &wmr_robot::handle_pid_receive , this ,
                                                 boost::asio::placeholders::error ,
                                                 boost::asio::placeholders::bytes_transferred) );
}

void wmr_robot::start_move_data_receive()
{
    socket_move_data_.async_receive_from( boost::asio::buffer( &Movement_Data_buf_ , sizeof(Movement_Data_buf_) ), robot_endpoint_ ,
                                    boost::bind( &wmr_robot::handle_move_data_receive , this ,
                                                 boost::asio::placeholders::error ,
                                                 boost::asio::placeholders::bytes_transferred) );
}

void wmr_robot::handle_sensor_receive(const boost::system::error_code &error, std::size_t size_ )
{
    if (!error || error == boost::asio::error::message_size)
    {
        sensor_node();

        start_sensor_receive();
    }

}

void wmr_robot::handle_odom_receive(const boost::system::error_code &error, std::size_t size_ )
{
    if (!error || error == boost::asio::error::message_size)
    {
        odometry_node();

        start_odom_receive();
    }
}

void wmr_robot::handle_pid_receive(const boost::system::error_code &error, std::size_t size_ )
{
    if (!error || error == boost::asio::error::message_size)
    {
        pid_node();

        start_pid_receive();
    }
}

void wmr_robot::handle_move_data_receive(const boost::system::error_code &error, std::size_t size_ )
{
    if (!error || error == boost::asio::error::message_size)
    {
        move_data_node();

        start_move_data_receive();
    }
}



/**************************************************************************************************************/

void wmr_robot::send_cmd_vel(float linear_velocity, float angular_velocity)
{
    Movement_cmd_buf_.Linear_Velocity  = linear_velocity ;
    Movement_cmd_buf_.Angular_Velocity = angular_velocity ;


    udp::endpoint remote_endpoint = udp::endpoint(address::from_string(ROBOT_IP), MOVEMENT_CMD_PORT);
    boost::system::error_code err;

    udp::endpoint host_endpoint = udp::endpoint(address::from_string(HOST_IP), MOVEMENT_CMD_PORT);

    try
    {
        socket_move_cmd_.open(udp::v4());
        socket_move_cmd_.bind(host_endpoint);
    }
    catch (std::exception& e)
    {
        std::cout << "\nwmr_robot::send_cmd_vel(): Host socket Bind Error.." <<std::endl;
        std::cerr << e.what() << std::endl ;
    }

    try
    {
        auto sent = socket_move_cmd_.send_to(boost::asio::buffer( &Movement_cmd_buf_  , sizeof(Movement_cmd_buf_) ) , remote_endpoint );
        //std::cout << "Sent cmd vel --- " << sent <<  " byte sent" << "\n";
        socket_move_cmd_.close();

    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl ;

    }

}
/**************************************************************************************************************/

void wmr_robot::send_motor_cmd(Motor_Cmd_Type motor_cmd)
{
    udp::endpoint remote_endpoint = udp::endpoint(address::from_string(ROBOT_IP), MOTOR_CMD_PORT);
    boost::system::error_code err;

    udp::endpoint host_endpoint = udp::endpoint(address::from_string(HOST_IP), MOTOR_CMD_PORT);

    try
    {
        socket_motor_cmd_.open(udp::v4());
        socket_motor_cmd_.bind(host_endpoint);
    }
    catch (std::exception& e)
    {
        std::cout << "\nwmr_robot::send_motor_cmd(): Host socket Bind Error.." <<std::endl;
        std::cerr << e.what() << std::endl ;
        ros::shutdown();
    }


    try
    {
        auto sent = socket_motor_cmd_.send_to(boost::asio::buffer( &motor_cmd  , sizeof(motor_cmd) ) , remote_endpoint  );
        //std::cout << "Sent cmd vel --- " << sent <<  " byte sent" << "\n";
        socket_motor_cmd_.close();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl ;
    }


}

void wmr_robot::callback_cmd_vel(const geometry_msgs::Twist::ConstPtr &msg)
{
    //ROS_INFO("I heard: [%f]", msg->velocity);

    /*move_cmd.linear.x = msg->linear.x;
    move_cmd.angular.z = msg->angular.z;
    WMR_Robot.Movement_Command(move_cmd.linear.x,move_cmd.angular.z);*/
    ROS_INFO("I heard velocity command : Linear=[%f] , Angular=[%f] ", msg->linear.x ,  msg->angular.z );
    send_cmd_vel( msg->linear.x , msg->angular.z );
}

void wmr_robot::odometry_node()
{

    current_time_ = ros::Time::now();

    // Geting Odometry Data From Socket
    float Result_Teta , Result_X , Result_Y , linear_x , angular_z ;

    //   Result_Teta;
    float2byte cast_float;
    cast_float.byte[0] = Odometery_Data_buf_.T_1;
    cast_float.byte[1] = Odometery_Data_buf_.T_2;
    cast_float.byte[2] = Odometery_Data_buf_.T_3;
    cast_float.byte[3] = Odometery_Data_buf_.T_4;
    Result_Teta = cast_float.float_;

   //    Result_X;
    cast_float.byte[0] = Odometery_Data_buf_.x_1;
    cast_float.byte[1] = Odometery_Data_buf_.x_2;
    cast_float.byte[2] = Odometery_Data_buf_.x_3;
    cast_float.byte[3] = Odometery_Data_buf_.x_4;
    Result_X = cast_float.float_;

   //   Result_Y;
    cast_float.byte[0] = Odometery_Data_buf_.y_1;
    cast_float.byte[1] = Odometery_Data_buf_.y_2;
    cast_float.byte[2] = Odometery_Data_buf_.y_3;
    cast_float.byte[3] = Odometery_Data_buf_.y_4;
    Result_Y = cast_float.float_;

    //  linear_x;
    cast_float.byte[0] = Odometery_Data_buf_.s_r_1;
    cast_float.byte[1] = Odometery_Data_buf_.s_r_2;
    cast_float.byte[2] = Odometery_Data_buf_.s_r_3;
    cast_float.byte[3] = Odometery_Data_buf_.s_r_4;
    linear_x = cast_float.float_;

   //   angular_z;
    cast_float.byte[0] = Odometery_Data_buf_.s_l_1;
    cast_float.byte[1] = Odometery_Data_buf_.s_l_2;
    cast_float.byte[2] = Odometery_Data_buf_.s_l_3;
    cast_float.byte[3] = Odometery_Data_buf_.s_l_4;
    angular_z = cast_float.float_;


    X += Result_X;
    Y += Result_Y;
    th += Result_Teta;



    //since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat_ = tf::createQuaternionMsgFromYaw(Result_Teta);


    //first, we'll publish the transform over tf
    odom_trans_.header.stamp = current_time_;
    odom_trans_.header.frame_id = "odom";
    odom_trans_.child_frame_id = "base_link";

    odom_trans_.transform.translation.x = X;
    odom_trans_.transform.translation.y = Y;
    odom_trans_.transform.translation.z = 0.0;
    odom_trans_.transform.rotation = odom_quat_;


    //send the transform
    transform_broadcaster_.sendTransform(odom_trans_);

//     //Laser publish the transform over tf
//     geometry_msgs::TransformStamped laser_trans;
//     laser_trans.header.stamp = ros::Time::now();
//     laser_trans.header.frame_id = "base";
//     laser_trans.child_frame_id = "laser";

//     laser_trans.transform.translation.z = 0.035;       // Should be regulate for the robot
//     geometry_msgs::Quaternion laser_quat = tf::createQuaternionMsgFromYaw(0);
//     laser_trans.transform.rotation = laser_quat;
//     //send the transform
//     transform_broadcaster_.sendTransform(laser_trans);


    //next, we'll publish the odometry message over ROS
    odom_.header.stamp = current_time_;
    odom_.header.frame_id = "odom";

    //set the position
    odom_.pose.pose.position.x = X;
    odom_.pose.pose.position.y = Y;
    odom_.pose.pose.position.z = 0.0;
    odom_.pose.pose.orientation = odom_quat_;

    //set the velocity
    odom_.child_frame_id = "base_link";
    odom_.twist.twist.linear.x  = linear_x;
    odom_.twist.twist.angular.z = angular_z;

    tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromYaw(th), tf::Vector3(X,Y, 0)), odom_.pose.pose);

    //publish the message
    pub_odometry_Data_.publish(odom_);

    last_time_ = current_time_;

}

void wmr_robot::pid_node()
{
    wmr-robot-hills::pid_data pid_msg;

    pid_msg.UP_REF_M1 = PID_Data_buf_.UP_REF_M1;
    pid_msg.UI_REF_M1 = PID_Data_buf_.UI_REF_M1;

    pid_msg.Err_ENcoder_M1_1 = PID_Data_buf_.Err_ENcoder_M1_1;
    pid_msg.Err_ENcoder_M1_2 = PID_Data_buf_.Err_ENcoder_M1_2;
    pid_msg.Err_ENcoder_M1_3 = PID_Data_buf_.Err_ENcoder_M1_3;
    pid_msg.Err_ENcoder_M1_4 = PID_Data_buf_.Err_ENcoder_M1_4;

    pid_msg.UP_REF_M2 = PID_Data_buf_.UP_REF_M2;
    pid_msg.UI_REF_M2 = PID_Data_buf_.UI_REF_M2;

    pid_msg.Err_ENcoder_M2_1 = PID_Data_buf_.Err_ENcoder_M2_1;
    pid_msg.Err_ENcoder_M2_2 = PID_Data_buf_.Err_ENcoder_M2_2;
    pid_msg.Err_ENcoder_M2_3 = PID_Data_buf_.Err_ENcoder_M2_3;
    pid_msg.Err_ENcoder_M2_4 = PID_Data_buf_.Err_ENcoder_M2_4;


    pub_PID_Data_.publish(pid_msg);
}

void wmr_robot::move_data_node()
{
    wmr-robot-hills::movement_data movement_msg;

    movement_msg.byte_1       = Movement_Data_buf_.byte_1;
    movement_msg.Direction_R  = Movement_Data_buf_.Direction_R;
    movement_msg.Direction_L  = Movement_Data_buf_.Direction_L;
    movement_msg.Pulse_MR_LSB = Movement_Data_buf_.Pulse_MR_LSB;
    movement_msg.Pulse_MR_MSB = Movement_Data_buf_.Pulse_MR_MSB;
    movement_msg.Pulse_ML_LSB = Movement_Data_buf_.Pulse_ML_LSB;
    movement_msg.Pulse_ML_MSB = Movement_Data_buf_.Pulse_ML_MSB;
    movement_msg.Feed_Dir_R   = Movement_Data_buf_.Feed_Dir_R;
    movement_msg.Feed_Dir_L   = Movement_Data_buf_.Feed_Dir_L;


    pub_Movement_data_.publish(movement_msg);

}

void wmr_robot::sensor_node()
{
    // Getting Sensors Data From Socket
    sensor_data_union conv_data;
    conv_data.Sensor_Data_buf_u = Sensor_Data_buf_ ;



    sensor_msgs::PointCloud pointCloud_sonar;
    pointCloud_sonar.header.stamp = ros::Time::now();
    pointCloud_sonar.header.frame_id = "sonar_link";

    for(int i=0; i<=15; i++)
    {
        geometry_msgs::Point32 p;
        //p.x =  Sonar_Sensor[i];
        p.x =  conv_data.byte_data[i];
        pointCloud_sonar.points.push_back(p);
    }

    pub_Sonar_.publish(pointCloud_sonar);



    sensor_msgs::PointCloud pointCloud_ir;
    pointCloud_ir.header.stamp = ros::Time::now();
    pointCloud_ir.header.frame_id = "ir_link";

    for(int i=16; i<=31; i++)
    {
        geometry_msgs::Point32 p;
        //p.x = IR_Sharp_Sensor[i];
        p.x = conv_data.byte_data[i];
        pointCloud_ir.points.push_back(p);
    }

    pub_IR_Sharp_.publish(pointCloud_ir);
}
