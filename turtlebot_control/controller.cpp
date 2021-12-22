#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <cmath>

class shapeListener
{
public:
  std_msgs::String shape;
  shapeListener() { shapeListener::shape.data = "None"; } //constructor
  void callback(const std_msgs::String::ConstPtr &msg);
};

void shapeListener::callback(const std_msgs::String::ConstPtr &msg)
{
  shapeListener::shape.data = msg->data.c_str();
  return;
}

class poseListener
{
public:
  float x_moved = 20;
  float y_moved = 20;
  float x_initial = 40;
  float y_initial = 40;
  float velocity = 20;
  int aux = 0;
  void callback(const nav_msgs::Odometry::ConstPtr &msg);
};

void poseListener::callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  /*   ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x, msg->twist.twist.angular.z); */

  poseListener::x_moved = msg->pose.pose.position.x;
  poseListener::y_moved = msg->pose.pose.position.y;
  poseListener::velocity = msg->twist.twist.linear.x + msg->twist.twist.linear.y + msg->twist.twist.angular.z;

  if (poseListener::aux == 0)
  {
    poseListener::x_initial = msg->pose.pose.position.x;
    poseListener::y_initial = msg->pose.pose.position.y;
    ROS_INFO("Initial pose: -> x: [%f], y: [%f]", poseListener::x_initial, poseListener::y_initial);
  }
   poseListener::aux = poseListener::aux + 1;
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle n; // init node
  ros::Publisher pub = n.advertise<std_msgs::String>("controller", 1000);
  shapeListener shapeListener1;
  ros::Subscriber sub = n.subscribe("shape_ui", 10, &shapeListener::callback, &shapeListener1);
  poseListener poseRobot;
  ros::Subscriber odom = n.subscribe("odom", 1000, &poseListener::callback, &poseRobot);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  int moves = 0;
  char initial_shape = shapeListener1.shape.data.c_str()[0];
  while (initial_shape == 'N')
  {
    initial_shape = shapeListener1.shape.data.c_str()[0];
    ros::spinOnce();
  }
  ROS_INFO("Shape to do: %s", shapeListener1.shape.data.c_str());

  geometry_msgs::Twist vel;
  switch (initial_shape)
  {
  case 'c':

    while (ros::ok())
    {

      vel.linear.x = 0.2;
      vel.linear.y = 0.2;
      vel.angular.z = 0.2;

      vel_pub.publish(vel);

      float d_x = std::abs(poseRobot.x_moved - poseRobot.x_initial);
      float d_y = std::abs(poseRobot.y_moved - poseRobot.y_initial);

      ROS_INFO("Position-> x: [%f], y: [%f]", poseRobot.x_moved, poseRobot.y_moved);
      ROS_INFO("Initial-> x: [%f], y: [%f]", poseRobot.x_initial, poseRobot.y_initial);

      if ((d_y < 0.1) && (d_x < 0.1)&&(poseRobot.aux >1000))
      {
        // publish msg to ui
        std_msgs::String msg;
        msg.data = "end";
        pub.publish(msg);
        break;
      }
      ros::spinOnce();
    }
    break;

  default:
    break;
  }

  while (poseRobot.velocity > 0.1)
  {
    // stop motion
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.angular.z = 0.0;
    vel_pub.publish(vel);
    ros::spinOnce();
  }

  return 0;
}
