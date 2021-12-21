#include "ros/ros.h"
#include "std_msgs/String.h"

class finishListener
{
public:
  std::string data;
  void callback(const std_msgs::String::ConstPtr &msg);
};

void finishListener::callback(const std_msgs::String::ConstPtr &msg)
{
  finishListener::data = "end";
  return;
} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "shape_ui_node");
  ros::NodeHandle n; // init node
  ros::Publisher shape_pub = n.advertise<std_msgs::String>("shape_ui", 1000);
  finishListener Finisher;
  ros::Subscriber sub = n.subscribe("controller", 1000, &finishListener::callback, &Finisher);
  std::string shape_input = "None";

      ROS_INFO("Select shape for turtle trajectory: triangle, circle, square:");

      std::getline(std::cin, shape_input);
      ROS_INFO("Request to draw a %s", shape_input.c_str());
      std_msgs::String msg;
      msg.data = shape_input;

  while (ros::ok()){      
  

      shape_pub.publish(msg);
    
    if (Finisher.data.compare("end") == 0){
      ROS_INFO("Shape finished");

       return 0;
    }

     ros::spinOnce();
  }

  return 0;
}
