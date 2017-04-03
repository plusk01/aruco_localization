#include <ros/ros.h>

#include "ArucoLocalizer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "aruco_node");
  ArucoLocalizer thing;

  ros::spin();
  return 0;
}
