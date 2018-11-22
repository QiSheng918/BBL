#include<ros/ros.h>
#include<BBL_robot/imu.h>
#include<cmath>

int main(int argc,char** argv)
{
  ros::init(argc,argv,"publisher_test");
  ros::NodeHandle nh;
 // ros::Time t1,t2;
  double t1,t;
  ros::Publisher pub=nh.advertise<BBL_robot::imu>("IMU_data",100);
  ros::Rate looprate(10);
  BBL_robot::imu msg;

  float pi=3.1415925;
  double array4[3] = {0,0,0};
  std::vector<double> array41(array4,array4+3);
  msg.data = array41;
  t1=ros::Time::now().toSec();
  int k=0;
  while(ros::ok()){
  t=ros::Time::now().toSec()-t1;
 // msg.data.resize(0);
//  msg.data[1]=0.5*sin(0.2*pi*t);
  //  if((k%100-50)>0)   msg.data[1]=1;
  //  else msg.data[1]=0;
  //  k++; 
  msg.data[0]=0.3*sin(0.4*pi*t);
  msg.data[1]=0.3*sin(0.4*pi*t);
  msg.data[2]=0.3*sin(0.4*pi*t);
  //msg.data[0]=0;
  pub.publish(msg);
  looprate.sleep();
}
  return 0;

}
