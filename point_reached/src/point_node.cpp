#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"

#define END_POSE 1

class pointListener
{
public:
    float distance_end=20;
    void callback(const nav_msgs::Odometry::ConstPtr &msg);
private:
    float initial=20;
    int aux = 0;
};

void pointListener::callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x, msg->twist.twist.angular.z);

    if (pointListener::aux == 0)
    {
        pointListener::aux = pointListener::aux + 1;
        pointListener::initial = msg->pose.pose.position.x;
        ROS_INFO("Initial pose: %f", pointListener::initial);
    }

    pointListener::distance_end = END_POSE -(msg->pose.pose.position.x - pointListener::initial);
    ROS_INFO("Distance to end: %f", pointListener::distance_end);
    return;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance");
    ros::NodeHandle n; // init node
    ros::Publisher dist_pub = n.advertise<std_msgs::Float64>("dist_to_end", 1000);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    pointListener endPoint;
    ros::Subscriber sub = n.subscribe("odom", 1000, &pointListener::callback, &endPoint);
    std_msgs::Float64 msg;
    geometry_msgs::Twist vel;
    vel.linear.x = 0.1;


    while (ros::ok())
    {
        vel_pub.publish(vel);
        msg.data = endPoint.distance_end;
        ROS_INFO("Distance to end [%f]", endPoint.distance_end);
        dist_pub.publish(msg);
        
        if ((endPoint.distance_end) < 0.01)
        {
            vel.linear.x = 0;
            vel_pub.publish(vel);
            ROS_INFO("End reached");
            return 0;
        }
        ros::spinOnce();
    }

    return 0;
}
