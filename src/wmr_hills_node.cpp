#include "wmr-robot-hills/wmr_hills.h"


#include <thread>



int main(int argc, char** argv )
{
    ros::init(argc, argv, "wmr_hills_node");    

    ROS_INFO_STREAM("\n .................... Connecting to robot .................... \n"
                    " Robot ip address: "<<"192.168.1.20"<<"\n"<<
                    " Sensors port: "<<"5002"<<"\n"<<
                    " PID port: "<<"10121"<<"\n"<<
                    " Odometery port: "<<"5003"<<"\n"<<
                    " Movement_Data Port: "<< "3023"<<"\n"
                    " Movement_CMD Port: "<< "6000" <<"\n"
                    " Motor_CMD Port: "<< "2222" <<"\n\n" );


    boost::asio::io_service io_serv;
    boost::asio::io_service::work work(io_serv);

    std::thread thread1( [&io_serv](){ io_serv.run(); }  );
    std::thread thread2( [&io_serv](){ io_serv.run(); }  );



    wmr_robot robot(io_serv);



    //io_serv.run();

    //robot.run();

    //ros::spin();

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    //thread_io.join();
    io_serv.stop();
    thread1.join();
    thread2.join();

    return 0;
}

