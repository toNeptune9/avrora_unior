#include "ros/ros.h"
#include <sstream>
#include <string>
#include "std_msgs/Float64.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
std_msgs::Float64 msgLeft;
std_msgs::Float64 msgRight;
ros::NodeHandle *n;
void drivingCallback(const std_msgs::Float64::ConstPtr& msg)
{
  double velocity =msg->data;
  n->setParam("desired_velocity",velocity);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive");
  ros::NodeHandle nh;
  n = &nh;

  ros::Publisher driveLeft_pub = nh.advertise<std_msgs::Float64>("/junior_car/EffortDriveLeft_controller/command", 10);
  ros::Publisher driveRight_pub = nh.advertise<std_msgs::Float64>("/junior_car/EffortDriveRight_controller/command", 10);
  ros::Subscriber sub = nh.subscribe("driving", 1000, drivingCallback);

  ros::Rate loop_rate(100);


  while (ros::ok())
  {
     double vel;
     nh.getParam("desired_velocity",vel);
    if (nh.getParam("desired_velocity",vel))
    {

      ROS_INFO("Got param: %f ", vel);
      if (vel >= 0) //move forward
      {
        msgLeft.data = vel;
        msgRight.data = -vel;
        driveLeft_pub.publish(msgLeft);
        driveRight_pub.publish(msgRight);
      }
      else //move backward
      {
        msgLeft.data = -vel;
        msgRight.data = vel;
        driveLeft_pub.publish(msgLeft);
        driveRight_pub.publish(msgRight);
      }

    }
    else
    {
       ROS_ERROR("Failed to get param 'desired_velocity'");
     }
    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;
}
