#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>

ros::Subscriber sub;
ros::Publisher pub;
tf::TransformBroadcaster *br;

void odomCallback(nav_msgs::Odometry const &msg)
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z));
    tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    transform.setRotation(q);
    br->sendTransform(tf::StampedTransform(transform,ros::Time::now(), "odom","footprint"));

}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle node;
  br = new tf::TransformBroadcaster;
  std::string odom_topic;
  odom_topic="/odom";
  sub = node.subscribe(odom_topic,1000,odomCallback);
  ros::spin();
  return 0;
};