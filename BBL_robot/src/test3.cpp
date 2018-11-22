#include<ros/ros.h>
#include<std_msgs/Float64MultiArray.h>
#include<sensor_msgs/JointState.h>

#include<iostream>
#include<cmath>

#include<Eigen/Core>
#include<Eigen/Dense>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>

#include "BBL_robot/IMU_sEMG.h"
#include "BBL_robot/joint_value_target.h"


//std_msgs::Float64MultiArray joint_velocity_msg;
//Eigen::Matrix<double,6,1> v_cart={1,0,0,0,0,0};
//ros::Rate looprate(10);
static const std::string PLANNING_GROUP = "arm";
// geometry_msgs::Pose pose_reference;




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

    bool flag;

    std_msgs::Float64MultiArray joint_velocity_msg;
    Eigen::Matrix<float,3,1> v_cart;
    moveit::planning_interface::MoveGroup move_group;
    geometry_msgs::Pose pose_now,pose_desire,pose_desire_reference,pose_reference;

    BBL_robot::IMU_sEMG global_msg;
    //Eigen::Matrix<float,6,6> J;
    float theta[6];
    ros::Publisher  pub; // 目标位置
    ros::Publisher  pub2;//当前位置

public:
    franka_robot();
    Eigen::Matrix<double,3,3> jacobian(float theta[]);
    Eigen::Matrix<double,3,1> forward_kinematics(float theta[]);
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
    v_cart<<0,0,0;
    emg_data_flag=true;
    for(int i=0;i<3;i++){
       pid[i]=4;
       max_vel[i]=1.7;
    }
    flag=true;
    joint_v<<0,0,0,0,0,0;
    
   // for(int i=0;i<6;i++)  theta[i]=0;
    
}



Eigen::Matrix<double,3,3> franka_robot::jacobian(float theta[])
{

    double L1 = 0.2261;
    double L2 = 0.3749;
    double L3 = 0.4351;
    Eigen::Matrix<double,3,3> J;
    J(0,0) = -sin(theta[0])*(L2*sin(theta[1])+L3*sin(theta[1]+theta[2]));
    J(0,1) = L2*cos(theta[0])*cos(theta[1])+L3*cos(theta[0])*cos(theta[1]+theta[2]);
    J(0,2) = L3*cos(theta[0])*cos(theta[1]+theta[2]);
    J(1,0) = -cos(theta[0])*(L2*sin(theta[1])+L3*sin(theta[1]+theta[2]));
    J(1,1) = -L2*sin(theta[0])*cos(theta[1])-L3*sin(theta[0])*cos(theta[1]+theta[2]);
    J(1,2) = -L3*sin(theta[0])*cos(theta[1]+theta[2]);
    J(2,0) = 0;
    J(2,1) = -L2*sin(theta[1])-L3*sin(theta[1]+theta[2]);
    J(2,2) = -L3*sin(theta[1]+theta[2]);
    return J;
}


Eigen::Matrix<double,3,1> franka_robot::forward_kinematics(float theta[])
{

    double L1 = 0.2261;
    double L2 = 0.3749;
    double L3 = 0.4351;
    Eigen::Matrix<double,3,1> p;
    p(0,0) = cos(theta[0])*(L2*sin(theta[1])+L3*sin(theta[1]+theta[2]));
    p(1,0) = -sin(theta[0])*(L2*sin(theta[1])+L3*sin(theta[1]+theta[2]));
    p(2,0) = L1+L2*cos(theta[1])+L3*cos(theta[1]+theta[2]);
    return p;
}


void franka_robot::joint_state_callback(const sensor_msgs::JointState::ConstPtr &joint_state_msg)
{
     for(int i=0;i<3;i++)  theta[i]=joint_state_msg->position[i];     
     if(flag && emg_data_flag){
          std::cout<<"theta"<<theta[0]<<","<<theta[1]<<","<<theta[2]<<std::endl;
          Eigen::Matrix<float,3,1> temp=(this->forward_kinematics(theta).cast<float>());
          std::cout<<"temp"<<temp<<std::endl;
          pose_reference.position.x=temp(0,0);
          pose_reference.position.y=temp(1,0);
          pose_reference.position.z=temp(2,0);
          
          flag=false;
          pose_now=pose_desire_reference=pose_reference;
          std::cout<<pose_reference<<std::endl;
     }

     //ROS_INFO("ok");     
}


void franka_robot::IMU_callback(const BBL_robot::IMU_sEMG::ConstPtr& IMU_msg)
{
    
     //ROS_INFO("You are in IMU callback!");
     double beishu = 5; 
     if(emg_data_flag && !flag ) 
     {
         //相当于这个数据是一个总的参考点
         global_msg.IMU_datax = IMU_msg->IMU_datax;
         global_msg.IMU_datay = IMU_msg->IMU_datay;
         global_msg.IMU_dataz = IMU_msg->IMU_dataz;
         emg_data_flag =false;
         std::cout<<"the global_msg is:"<<global_msg<<std::endl;
     }
        std::cout<<"the IMU_msg is:"<<IMU_msg->IMU_datax<<","<<IMU_msg->IMU_datay<<","<<IMU_msg->IMU_dataz<<std::endl;
        double delta_x = (IMU_msg->IMU_datax - global_msg.IMU_datax)/beishu;
        double delta_y = (IMU_msg->IMU_datay - global_msg.IMU_datay)/beishu;
        double delta_z = (IMU_msg->IMU_dataz - global_msg.IMU_dataz)/beishu;
        std::cout<<"error:"<<delta_x<<","<<delta_y<<","<<delta_z<<std::endl;
        pose_desire.position.x = pose_reference.position.x + delta_x;
        pose_desire.position.y = pose_reference.position.y + delta_y;
        pose_desire.position.z = pose_reference.position.z + delta_z;
        std::cout<<"the pose_reference is:"<<pose_reference<<std::endl;
        std::cout<<"the pose_desire is:"<<pose_desire<<std::endl;

        

}
void franka_robot::start()
{
     
     pub = nh.advertise<BBL_robot::joint_value_target>("pose_target", 1000);
     pub2 = nh.advertise<BBL_robot::joint_value_target>("pose_now", 1000);  

     joint_velocity_pub=nh.advertise<std_msgs::Float64MultiArray>("/velocity_group_controller/command",100);
     IMU_sub=nh.subscribe<BBL_robot::IMU_sEMG>("emg_data",1000,&franka_robot::IMU_callback,this);
     joint_state_sub=nh.subscribe<sensor_msgs::JointState>("/joint_states",10,&franka_robot::joint_state_callback,this);
     
     BBL_robot::joint_value_target pose_target_topic;
     BBL_robot::joint_value_target pose_now_topic;
     ros::Duration(1.0).sleep();
     int k=0;
     while(ros::ok()){
        
        if(!emg_data_flag){
            // while(ros::ok())//防止下面程序执行完之后emg数据还没更新
            // {
            //     double error_x = pose_desire_reference.position.x - pose_desire.position.x;
            //     double error_y = pose_desire_reference.position.y - pose_desire.position.y;
            //     double error_z = pose_desire_reference.position.z - pose_desire.position.z;

            //     std::cout<<"error_desire:"<<error_x<<","<<error_y<<","<<error_z<<std::endl;
            //     if(fabs(error_x)>0.0000005||fabs(error_y)>0.0000005||fabs(error_z) > 0.0000005) 
            //     {
            //         ROS_INFO("Reveiving New Data Now!");
            //         //这个循环一般都不会死掉
            //         break;
            //     }
            // }

            pose_desire_reference=pose_desire;
           // pose_now = move_group.getCurrentPose(move_group.getEndEffectorLink()).pose;
            Eigen::Matrix<double,3,1> temp=this->forward_kinematics(theta);
            std::cout<<"the forward kinetics is:"<<temp<<std::endl;
            pose_now.position.x=temp(0,0);
            pose_now.position.y=temp(1,0);
            pose_now.position.z=temp(2,0);
            
            v_cart[0] = pid[0] * (pose_desire.position.x - pose_now.position.x);
            v_cart[1] = pid[1] * (pose_desire.position.y - pose_now.position.y);
            v_cart[2] = pid[2] * (pose_desire.position.z - pose_now.position.z);

            pose_target_topic.joint_value.clear();
            pose_now_topic.joint_value.clear();

                pose_now_topic.joint_value.push_back(pose_now.position.x);
                pose_now_topic.joint_value.push_back(pose_now.position.y);
                pose_now_topic.joint_value.push_back(pose_now.position.z);

                pose_target_topic.joint_value.push_back(pose_desire.position.x);
                pose_target_topic.joint_value.push_back(pose_desire.position.y);
                pose_target_topic.joint_value.push_back(pose_desire.position.z);




                pub.publish(pose_target_topic);
                pub2.publish(pose_now_topic);
            
            std::cout<<"the desire C v is:"<<v_cart<<std::endl;
            // for(int i = 0 ; i < 3; i ++)
            // {
            //    if(v_cart[i]>max_vel[i])   v_cart[i] = max_vel[i];
            //    if(v_cart[i]<(-max_vel[i]))  v_cart[i] = -max_vel[i];
            // }
            
            
//           std::cout<<this->jacobian(theta)<<std::endl;
//           for(int i=0;i<6;i++)   std::cout<<theta[i]<<",";
//           std::cout<<std::endl;
////           
            // ROS_INFO("the determinant of J is %f",jacobian(theta).determinant());  
            // std::cout<<this->jacobian(theta)<<std::endl;   
           Eigen::Matrix<double,3,3> J=jacobian(theta);   
           std::cout<<"the jacobian is"<<J<<std::endl;  
    	   Eigen::Matrix<double,3,3> M=J*J.transpose();   
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
            if(M.determinant()>0.0001){
               Eigen::Matrix<float,3,1>  joint_v_temp=J.inverse().cast<float>()*v_cart;
               joint_v.block(0,0,3,1)=joint_v_temp;
               std::cout<<"the desire joint velocity is:"<<joint_v<<std::endl;
               joint_velocity_msg.data.resize(0);
               for(int i=0;i<3;i++)  {
		          if(joint_v(i)>1)  joint_v(i)=1;
                  if(joint_v(i)<-1)  joint_v(i)=-1;
                //   if(joint_v(i)<0.02 && joint_v(i)>-0.02)  joint_v(i)=0;         
                  joint_velocity_msg.data.push_back(joint_v(i));              
               }
               joint_v(4)=-joint_v(1)-joint_v(2);
               joint_velocity_msg.data.push_back(0); 
               joint_velocity_msg.data.push_back(joint_v(4));
               joint_velocity_msg.data.push_back(0);              
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

