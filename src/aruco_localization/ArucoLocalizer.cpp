#include "aruco_localization/ArucoLocalizer.h"

namespace aruco_localizer {
  
// ----------------------------------------------------------------------------

ArucoLocalizer::ArucoLocalizer() :
    nh_(ros::NodeHandle()), nh_private_("~")
{

    // Read in ROS params
    std::string mmConfigFile = nh_private_.param<std::string>("markermap_config", "");
    double markerSize = nh_private_.param<double>("marker_size", 0.0298);
    bool showOutputVideo = nh_private_.param<bool>("show_output_video", false);

    // Subscribe to input video feed and publish output video feed
    it_ = image_transport::ImageTransport(nh_);
    image_sub_ = it_.subscribeCamera("input_image", 1, &ArucoLocalizer::cameraCallback, this);
    image_pub_ = it_.advertise("output_image", 1);

    // Create ROS publishers
    // tag_list_pub = nh_.advertise<aprilvo::AprilTagList>("apriltags", 100);
    estimate_pub = nh_private_.advertise<nav_msgs::Odometry>("estimate", 1);

    //
    // Set up the ArUco detector
    //

    // Set up the Marker Map dimensions, spacing, dictionary, etc from the YAML
    mmConfig_.readFromFile(mmConfigFile);

    // Prepare the marker detector by:
    // (1) setting the dictionary we are using
    mDetector_.setDictionary(mmConfig_.getDictionary());
    // (2) setting the corner refinement method
    // ... TODO -- make this corner sub pix or something
    mDetector_.setCornerRefinementMethod(MarkerDetector::LINES);

    // set markmap size. Convert to meters if necessary
    if (mmConfig_.isExpressedInPixels())
        mmConfig_ = mmConfig_.convertToMeters(markerSize)

    // Configure the Pose Tracker
    if (camParams_.isValid() && mmConfig_.isExpressedInMeters())
        mmPoseTracker_.setParams(camParams_, mmConfig_);
}

// ----------------------------------------------------------------------------

ArucoLocalizer::~ArucoLocalizer() { }

// ----------------------------------------------------------------------------

void ArucoLocalizer::cameraCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cinfo) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // update the camera model with the camera's intrinsic parameters
  cam_model_.fromCameraInfo(cinfo);

  processCvImage(cv_ptr);

  /*


    // Detection of the board
    vector<aruco::Marker> detected_markers=TheMarkerDetector.detect(TheInputImage);

    // print the markers detected that belongs to the markerset
    for(auto idx:TheMarkerMapConfig.getIndices(detected_markers))
        detected_markers[idx].draw(TheInputImageCopy, Scalar(0, 0, 255), 2);

    //detect 3d info if possible
    if (TheMSPoseTracker.isValid()){
        if ( TheMSPoseTracker.estimatePose(detected_markers)){
            aruco::CvDrawingUtils::draw3dAxis(TheInputImageCopy,  TheCameraParameters,TheMSPoseTracker.getRvec(),TheMSPoseTracker.getTvec(),TheMarkerMapConfig[0].getMarkerSize()*2);
            frame_pose_map.insert(make_pair(index,TheMSPoseTracker.getRTMatrix() ));
            cout<<"pose rt="<<TheMSPoseTracker.getRvec()<<" "<<TheMSPoseTracker.getTvec()<<endl;
        }
    }


  */

  if (show_output_video) {
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(1);
  }

  // Output modified video stream
  image_pub_.publish(cv_ptr->toImageMsg());
}

// ----------------------------------------------------------------------------

}