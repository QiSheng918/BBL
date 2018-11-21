#include<ros/ros.h>
#include<std_msgs/Float64MultiArray.h>
#include<sensor_msgs/JointState.h>

#include<iostream>
#include<cmath>

#include<Eigen/Core>
#include<Eigen/Dense>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>

#include "BBL_robot/IMU_sEMG.h"


//std_msgs::Float64MultiArray joint_velocity_msg;
//Eigen::Matrix<double,6,1> v_cart={1,0,0,0,0,0};
//ros::Rate looprate(10);
static const std::string PLANNING_GROUP = "arm";




class franka_robot
{
private:
    ros::NodeHandle nh;
    ros::Publisher joint_velocity_pub;
    ros::Subscriber joint_state_sub;
    ros::Subscriber IMU_sub;
    Eigen::Matrix<float,6,1> joint_v;

    bool emg_data_flag;
    double pid[3];
    double max_vel[3];

    std_msgs::Float64MultiArray joint_velocity_msg;
    Eigen::Matrix<float,6,1> v_cart;
    moveit::planning_interface::MoveGroupInterface move_group;
    geometry_msgs::Pose pose_now,pose_desire,pose_reference,pose_desire_reference;

    BBL_robot::IMU_sEMG global_msg;
    //Eigen::Matrix<float,6,6> J;
    float theta[6];

public:
    franka_robot();
    Eigen::Matrix<double,6,6> jacobian(float theta[]);
    Eigen::Matrix<double,6,6> Ad_T(Eigen::Matrix<double,4,4> T);
    void start();
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg);
    void IMU_callback(const BBL_robot::IMU_sEMG::ConstPtr& IMU_msg);
};

franka_robot::franka_robot():move_group(PLANNING_GROUP)
{ 
//    move_group.setNamedTarget("init_pose");
//    move_group.move();
//    ros::Duration(0.5).sleep();  
    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.2); 
    v_cart<<0,0,0,0,0,0;
    emg_data_flag=true;
    for(int i=0;i<3;i++){
       pid[i]=1.2;
       max_vel[i]=1.7;
    }
   // for(int i=0;i<6;i++)  theta[i]=0;
    
}

Eigen::Matrix<double,6,6> franka_robot::Ad_T(Eigen::Matrix<double,4,4> T)
{

    Eigen::Matrix<double,3,3> p33,zeros33;
    p33<<0,-T(2,3),T(1,3),
       T(2,3), 0,-T(0,3),
       -T(1,3),T(0,3),0;
    Eigen::Matrix<double,3,3> R=T.block(0,0,3,3);
    Eigen::Matrix<double,6,6> AdT;
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++)  zeros33(i,j)=0;
    }
    AdT.block(0,0,3,3)=R;
    AdT.block(3,3,3,3)=R;
    AdT.block(0,3,3,3)=p33*R;
    AdT.block(3,0,3,3)=zeros33;
    return AdT;
}


Eigen::Matrix<double,6,6> franka_robot::jacobian(float theta[])
{
    Eigen::Matrix<double,6,1> twist[6];

    twist[0]<<0,0,0,0,0,-1;
    twist[1]<<-0.2261,0,0,0,1,0;
    twist[2]<<-0.6011,0,0,0,1,0;
    twist[3]<<0,0,0,0,0,-1;
    twist[4]<<-1.0361,0,0,0,1,0;
    twist[5]<<0,0,0,0,0,-1;
    Eigen::Matrix<double,3,3> SO3[6];
    Eigen::Matrix<double,4,4> SE3[6];
    Eigen::Matrix<double,3,3> W;
    Eigen::Matrix<double,3,3> I33;
    Eigen::Matrix<double,3,1> p[6];
    Eigen::Matrix<double,3,1> v;
    Eigen::Matrix<double,1,4> zeros_one;
    I33<<1,0,0,0,1,0,0,0,1;
    zeros_one<<0,0,0,1;
    
    for(int i=0;i<6;i++){
    W<<0,-twist[i](5,0),twist[i](4,0),
       twist[i](5,0), 0,-twist[i](3,0),
       -twist[i](4,0),twist[i](3,0),0;
    v<<twist[i](0,0),twist[i](1,0),twist[i](2,0);
    SO3[i]=I33+sin(theta[i])*W+(1-cos(theta[i]))*W*W;

    p[i]=(theta[i]*I33+(1-cos(theta[i]))*W+(theta[i]-sin(theta[i]))*W*W)*v;
    SE3[i].block(0,0,3,3)=SO3[i];
    SE3[i].block(0,3,3,1)=p[i];
    SE3[i].block(3,0,1,4)=zeros_one;
}


    Eigen::Matrix<double,4,4> I44;
    Eigen::Matrix<double,6,6> J;
    I44<<1,0,0,0,
         0,1,0,0,
  	     0,0,1,0,
	     0,0,0,1;
    for(int i=0;i<6;i++){
      J.block(0,i,6,1)=Ad_T(I44)*twist[i];
      I44=I44*SE3[i];
    } 
    return J;
}

void franka_robot::joint_state_callback(const sensor_msgs::JointState::ConstPtr &joint_state_msg)
{
     for(int i=0;i<6;i++)  theta[i]=joint_state_msg->position[i];     
     //ROS_INFO("ok");     
}


void franka_robot::IMU_callback(const BBL_robot::IMU_sEMG::ConstPtr& IMU_msg)
{
    
     //ROS_INFO("You are in IMU callback!");
     double beishu = 10; 
     if(emg_data_flag) 
     {
         //相当于这个数据是一个总的参考点
         global_msg.IMU_datax = IMU_msg->IMU_datax;
         global_msg.IMU_datay = IMU_msg->IMU_datay;
         global_msg.IMU_dataz = IMU_msg->IMU_dataz;
         emg_data_flag =false;
     }
        double delta_x = (IMU_msg->IMU_datax - global_msg.IMU_datax)/beishu;
        double delta_y = (IMU_msg->IMU_datay - global_msg.IMU_datay)/beishu;
        double delta_z = (IMU_msg->IMU_dataz - global_msg.IMU_dataz)/beishu;

        pose_desire.position.x = pose_reference.position.x - delta_x;
        pose_desire.position.y = pose_reference.position.y - delta_y;
        pose_desire.position.z = pose_reference.position.z - delta_z;

}
void franka_robot::start()
{
     pose_now = move_group.getCurrentPose(move_group.getEndEffectorLink()).pose;  
     pose_desire_reference=pose_reference=pose_now;  
     joint_velocity_pub=nh.advertise<std_msgs::Float64MultiArray>("/velocity_group_controller/command",100);
     IMU_sub=nh.subscribe<BBL_robot::IMU_sEMG>("emg_data",1000,&franka_robot::IMU_callback,this);
     joint_state_sub=nh.subscribe<sensor_msgs::JointState>("/joint_states",10,&franka_robot::joint_state_callback,this);

     ros::Duration(1.0).sleep();
     int k=0;
     while(ros::ok()){
        if(!emg_data_flag){
            while(ros::ok())//防止下面程序执行完之后emg数据还没更新
            {
                double error_x = pose_desire_reference.position.x - pose_desire.position.x;
                double error_y = pose_desire_reference.position.y - pose_desire.position.y;
                double error_z = pose_desire_reference.position.z - pose_desire.position.z;
                if(fabs(error_x)>0.0000005||fabs(error_y)>0.0000005||fabs(error_z) > 0.0000005) 
                {
                    ROS_INFO("Reveiving New Data Now!");
                    //这个循环一般都不会死掉
                    break;
                }
            }

            pose_desire_reference=pose_desire;
            pose_now = move_group.getCurrentPose(move_group.getEndEffectorLink()).pose;
            
            v_cart[0] = pid[0] * (pose_desire.position.x - pose_now.position.x);
            v_cart[1] = pid[1] * (pose_desire.position.y - pose_now.position.y);
            v_cart[2] = pid[2] * (pose_desire.position.z - pose_now.position.z);
            
            std::cout<<"the desire C v is:"<<v_cart<<std::endl;
            for(int i = 0 ; i < 3; i ++)
            {
               if(v_cart[i]>max_vel[i])   v_cart[i] = max_vel[i];
               if(v_cart[i]<(-max_vel[i]))  v_cart[i] = -max_vel[i];
            }
            
            
//           std::cout<<this->jacobian(theta)<<std::endl;
//           for(int i=0;i<6;i++)   std::cout<<theta[i]<<",";
//           std::cout<<std::endl;
////           
            // ROS_INFO("the determinant of J is %f",jacobian(theta).determinant());  
            // std::cout<<this->jacobian(theta)<<std::endl;   
           Eigen::Matrix<double,6,6> J=jacobian(theta);   
           std::cout<<"the jacobian is"<<J<<std::endl;  
    	   Eigen::Matrix<double,6,6> M=J*J.transpose();   
           std::cout<<"the determinant of M is"<<M.determinant()<<std::endl;
        //    Eigen::Matrix<float,6,1> v;
//std::cout<<<<std::endl;
        //    joint_velocity_msg.data.resize(0);
        //    int m=k%10-5;
        //    if(m>=0)  v<<0.2,0.2,0,0,0,0;
        //    else             v<<-0.2,-0.2,0,0,0,0;
        //    k++;
        //    for(int j=0;j<6;j++)   joint_velocity_msg.data.push_back(v[j]);       
        //    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6> >  es(M);
        //    Eigen::Matrix<double,6,1> values=es.eigenvalues();
           //joint_velocity_pub.publish(joint_velocity_msg);
        //    ros::Duration(0.2).sleep();
        //    std::cout<<es.eigenvalues()<<std::endl;
        //    if(M.determinant()>1.4579e-05){
            // Eigen::Matrix<float,6,1> v_cart2;
            //std::cout<<J.inverse()<<std::endl;
            if(M.determinant()>0.000000001){
               joint_v=J.inverse().cast<float>()*v_cart;
               std::cout<<"the desire joint velocity is:"<<joint_v<<std::endl;
               joint_velocity_msg.data.resize(0);
               for(int i=0;i<6;i++)  {
		          if(joint_v(i)>1)  joint_v(i)=1;
                  if(joint_v(i)<-1)  joint_v(i)=-1;         
                  joint_velocity_msg.data.push_back(joint_v(i));              
               }
              
              joint_velocity_pub.publish(joint_velocity_msg);
              ros::Duration(0.1).sleep();
           }
           else{
              joint_velocity_msg.data.resize(0);
              joint_v<<0,0,0,0,0,0;
              for(int i=0;i<6;i++)   joint_velocity_msg.data.push_back(joint_v(i));
              joint_velocity_pub.publish(joint_velocity_msg);
              ros::Duration(0.1).sleep();
           }
        }
        else
        {
            ROS_ERROR("WAITING FOR EMG DATA!");
            ros::Duration(0.2).sleep();
        }
     }
}


int main(int argc,char**argv)
{
  ros::init(argc,argv,"test");
  ros::AsyncSpinner spinner(3);
  spinner.start();
  franka_robot robot;
  robot.start();
 // ros::spin();
  return 0;
}

