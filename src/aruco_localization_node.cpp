#include <ros/ros.h>

#include "aruco_localization/ArucoLocalizer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "aruco_node");
  aruco_localizer::ArucoLocalizer thing;

  ros::spin();
  return 0;
}
