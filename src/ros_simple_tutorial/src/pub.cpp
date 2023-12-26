#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::String>("message", 10);
  std_msgs::String msg;

  ros::Rate r(1);
  while (ros::ok())
  {
    msg.data = "bigbigpark";

    pub.publish(msg);

    r.sleep();
  }

  return 0;
}