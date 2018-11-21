#include "ros/ros.h"
#include "BBL_robot/imu.h"
#include "BBL_robot/IMU_sEMG.h"

const int window_size = 6;
int emg_flag = 0;
int count = 0;


ros::Subscriber sub;
ros::Publisher pub;
BBL_robot::IMU_sEMG emg_msg;
std::vector<BBL_robot::IMU_sEMG> emg_msg_vector;

void window_process()
{
    for(int i = 0 ; i < window_size - 1 ; i ++)
    {
        emg_msg_vector[i] = emg_msg_vector[i+1];
    }
    emg_msg_vector[window_size - 1] = emg_msg;
    double x,y,z;
    x=y=z=0.0;
    for(int i = 0 ; i < window_size ; i ++)
    {
        x += emg_msg_vector[i].IMU_datax;
        y += emg_msg_vector[i].IMU_datay;
        z += emg_msg_vector[i].IMU_dataz;
    }
    emg_msg.IMU_datax = x/window_size;
    emg_msg.IMU_datay = y/window_size;
    emg_msg.IMU_dataz = z/window_size;
}


void callback(const BBL_robot::imu::ConstPtr& msg)
{
    if(emg_flag == 0) emg_flag = 1;

    emg_msg.IMU_datax = msg->data[0];
    emg_msg.IMU_datay = msg->data[1];
    emg_msg.IMU_dataz = msg->data[2];

    if(count < window_size)
    {
        emg_msg_vector[count] = emg_msg;
        count++;
        ROS_INFO("Receiving IMU Data");
    }
    else
    {
        window_process();
        ROS_INFO("Receiving IMU Data");
        pub.publish(emg_msg);
    } 
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imu_data_receiving");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    pub = nh.advertise<BBL_robot::IMU_sEMG>("emg_data", 1000);
    sub = nh.subscribe<BBL_robot::imu>("IMU_data",10,callback);
    emg_msg_vector.resize(window_size);

    ros::Duration(1.0).sleep();

    while(ros::ok())
    {
        if(emg_flag == 0)
        {
            ROS_ERROR("NO EMG DATA!");
            ros::Duration(0.5).sleep();
        }
    }
    spinner.stop();
    return 0;
}
