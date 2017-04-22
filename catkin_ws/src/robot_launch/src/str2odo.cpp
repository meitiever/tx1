#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.1;
double vy = -0.1;
double vth = 0.1;

void VelCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string vel = msg->data;
  std::vector<std::string> v;
  boost::split(v, vel, boost::is_any_of(" "), boost::token_compress_on);
  try {
    vx = boost::lexical_cast<double>(v[0]);
    vy = boost::lexical_cast<double>(v[1]);
    vth= boost::lexical_cast<double>(v[2]);
    x  = boost::lexical_cast<double>(v[3]);
    y  = boost::lexical_cast<double>(v[4]);
    th = boost::lexical_cast<double>(v[5]);
    //ROS_INFO("I heard: [%f]", vx);
  }
  catch(...) { }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "str2odom");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("VEL2", 50);
  ros::Subscriber sub = n.subscribe("VEL", 1000, VelCallback);

  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time;

  ros::Rate r(10.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    r.sleep();
  }
}

