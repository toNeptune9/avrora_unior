#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <utility>
#include <cmath>
#include <sstream>
#include <string>
#include "ackermann_msgs/AckermannDriveStamped.h"

ros::Publisher left_pub;
ros::Publisher right_pub;
std_msgs::Float64 fi_r;
std_msgs::Float64 fi_l;


// These are known const values. radius, lateral_distance between front and rear wheels
// front distance between front steering wheels
// pi2 is pi/2 constant value in radians

double const lateral_distance = 0.7;
double const front_distance = 0.54;
double const pi2 = 1.5708;

///double far = lateral_distance / (r + (front_distance/2));
//double near = lateral_distance / (r - (front_distance/2));


double radiusCalc (double angle)
{
  return lateral_distance/tan(angle);
}

double formulaCalc (double radius, bool sign)
{
  if (sign)
  {
    return lateral_distance / (radius + (front_distance/2));
  }
  else
  { 
    return lateral_distance / (radius - (front_distance/2));
  }
}

void steeringCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg)
{
  double radius = radiusCalc(msg->drive.steering_angle);
  ROS_INFO("---Calculated radius : %.4f,", radius);

  if (msg->drive.steering_angle >= 0 && msg->drive.steering_angle <= 1.20)
  {  

    fi_r.data = atan(formulaCalc(radius, false));
    fi_l.data = atan(formulaCalc(radius,true));
    ROS_INFO("---Calculated angles : left wheel  %.4f, right wheel %.4f", fi_r.data,fi_l.data);
    left_pub.publish(fi_r);
    right_pub.publish(fi_l);
  }
  if (msg->drive.steering_angle <= 0 && msg->drive.steering_angle >= -1.20)
  {
      ROS_INFO("---Calculated angles : right wheel %.4f, left wheel %.4f", fi_r.data,fi_l.data);
      fi_r.data = atan(formulaCalc(radius,true));
      fi_l.data = atan(formulaCalc(radius, false));
      left_pub.publish(fi_l);
      right_pub.publish(fi_r);
  }
  if (msg->drive.steering_angle > 1.20)
  { 
    double radius_exception(1.2);
    ROS_INFO("---Calculated radius exception : %.4f,", radius);
    fi_r.data = atan(formulaCalc(radius_exception, false));
    fi_l.data = atan(formulaCalc(radius_exception,true));
    ROS_INFO("---Calculated angles : left wheel  %.4f, right wheel %.4f", fi_r.data,fi_l.data);
    left_pub.publish(fi_r);
    right_pub.publish(fi_l);
  }
  if (msg->drive.steering_angle < -1.2)
  {
     double radius_exception(-1.2);
    ROS_INFO("---Calculated radius exception : %.4f,", radius);
    fi_r.data = atan(formulaCalc(radius_exception, false));
    fi_l.data = atan(formulaCalc(radius_exception,true));
    ROS_INFO("---Calculated angles : left wheel  %.4f, right wheel %.4f", fi_r.data,fi_l.data);
    left_pub.publish(fi_r);
    right_pub.publish(fi_l);

  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "steer_remap");
  ros::NodeHandle n;
  std::string lw_topic, rw_topic, central_steer_topic;
  central_steer_topic = "/junior_car/ackermann_cmd";
  lw_topic = "/junior_car/SteerLeft_controller/command";
  rw_topic = "/junior_car/SteerRight_controller/command";

  if (n.getParam("left_wheel_steer_topic", lw_topic))
  {
      ROS_INFO("Got param: %s", lw_topic.c_str());
  }

  if (n.getParam("right_wheel_steer_topic", rw_topic))
  {
      ROS_INFO("Got param: %s", rw_topic.c_str());
  }

  if (n.getParam("right_wheel_steer_topic", central_steer_topic))
  {
      ROS_INFO("Got param: %s", central_steer_topic.c_str());
  }

  left_pub = std::move(n.advertise<std_msgs::Float64>(lw_topic, 1024));
  right_pub = std::move(n.advertise<std_msgs::Float64>(rw_topic, 1024));
  ros::Subscriber sub = n.subscribe(central_steer_topic, 1024, steeringCallback);
  ros::spin();
  return 0;
}
