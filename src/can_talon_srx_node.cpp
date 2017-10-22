#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "can_talon_srx_node");

  ros::Rate update_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    update_rate.sleep();
  }

  return 0;
}
