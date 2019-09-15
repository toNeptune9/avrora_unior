#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <utility>


ros::Publisher left_pub;
ros::Publisher right_pub;

void chatterCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg)
{
  std_msgs::Float64 x;
  x.data = msg->drive.speed;
  left_pub.publish(x);
  right_pub.publish(x);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drive_remap");
    ros::NodeHandle n;
    std::string lw_topic, rw_topic, central_drive_topic;
    central_drive_topic = "/junior_car/ackermann_cmd";
    lw_topic = "/junior_car/EffortDriveLeft_controller/command";
    rw_topic = "/junior_car/EffortDriveRight_controller/command";
    if (n.getParam("left_wheel_topic", lw_topic))
    {
        ROS_INFO("Got param: %s", lw_topic.c_str());
    }

    if (n.getParam("right_wheel_topic", rw_topic))
    {
        ROS_INFO("Got param: %s", rw_topic.c_str());
    }

    if (n.getParam("right_wheel_topic", central_drive_topic))
    {
        ROS_INFO("Got param: %s", central_drive_topic.c_str());
    }

    left_pub = std::move(n.advertise<std_msgs::Float64>(lw_topic, 1024));
    right_pub = std::move(n.advertise<std_msgs::Float64>(rw_topic, 1024));
    ros::Subscriber sub = n.subscribe(central_drive_topic, 1024, chatterCallback);
    ros::spin();
    return 0;
}
