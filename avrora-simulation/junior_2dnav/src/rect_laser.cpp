#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <utility>
#include <cmath>
#include <sstream>
#include <string>
#include "sensor_msgs/LaserScan.h"

ros::Subscriber sub;
ros::Publisher pub;
const double laser_incline = 0.261799;
const double laser_height = 0.4;
const double max_obstacle = laser_height/tan(laser_incline);

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  sensor_msgs::LaserScan msg;
  msg.header.stamp = scan->header.stamp;
  msg.header.frame_id = "fake_laser";
  msg.angle_min = scan->angle_min;
  msg.angle_max = scan->angle_max;
  msg.angle_increment = scan->angle_increment;
  msg.time_increment = scan->time_increment;
  msg.range_min = scan->range_min;
  msg.range_max = scan->range_max;
  msg.ranges.resize(scan->ranges.size());
  msg.intensities.resize(scan->intensities.size());
  for (int i=0; i<scan->ranges.size() ; i++)
  {
    double r = scan->ranges[i] * cos(laser_incline);
    if (r > max_obstacle)
      r = std::numeric_limits<double>::infinity();
    msg.ranges[i] = r;
    msg.intensities[i] = scan->intensities[i];
  }
  pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rectificated_laser");
  ros::NodeHandle n;
  std::string scan_topic, remap_topic;
  remap_topic = "/rectificated_scan";
  scan_topic="/scan";
  pub = n.advertise<sensor_msgs::LaserScan>(remap_topic, 50);
  sub = n.subscribe(scan_topic, 1, scanCallback);
  ros::spin();
  return 0;
}
