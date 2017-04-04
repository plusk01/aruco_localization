#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include <ros/ros.h>
#include <aruco/aruco.h>
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
// #include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

namespace aruco_localizer {

class ArucoLocalizer
{
public:
	ArucoLocalizer();
	~ArucoLocalizer();

private:
	// ROS node handles
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

	// image transport pub/sub
	image_transport::ImageTransport it_;
	image_transport::CameraSubscriber image_sub_;
	image_transport::Publisher image_pub_;

	// ROS tf and camera model
	// tf::TransformListener tf_listener_;
	image_geometry::PinholeCameraModel cam_model_;

	// ROS publishers and subscribers
	ros::Publisher estimate_pub_;

	// ArUco Map Detector
	aruco::MarkerMap mmConfig_;
	aruco::MarkerDetector mDetector_;
	aruco::MarkerMapPoseTracker mmPoseTracker_;
	aruco::CameraParameters camParams_;

	bool show_output_video_;

	//
	// Methods
	//

	// image_transport camera subscriber
	void cameraCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cinfo);

	// This is where the real ArUco processing is done
	void processImage(cv::Mat& frame);

	// Convert ROS CameraInfo message to ArUco style CameraParameters
	aruco::CameraParameters ros2arucoCamParams(const sensor_msgs::CameraInfoConstPtr& cinfo);

	// functions to convert to tf and then broadcast
	tf::Transform aruco2tf(const cv::Mat& rvec, const cv::Mat& tvec);
	void sendtf(const cv::Mat& rvec, const cv::Mat& tvec);
};

}