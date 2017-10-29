#include "aruco_localization/ArucoLocalizer.h"

namespace aruco_localizer {

// ----------------------------------------------------------------------------

ArucoLocalizer::ArucoLocalizer() :
    nh_(ros::NodeHandle()), nh_private_("~"), it_(nh_)
{

    // Read in ROS params
    std::string mmConfigFile = nh_private_.param<std::string>("markermap_config", "");
    markerSize_ = nh_private_.param<double>("marker_size", 0.0298);
    nh_private_.param<bool>("show_output_video", showOutputVideo_, false);
    nh_private_.param<bool>("debug_save_input_frames", debugSaveInputFrames_, false);
    nh_private_.param<bool>("debug_save_output_frames", debugSaveOutputFrames_, false);
    nh_private_.param<std::string>("debug_image_path", debugImagePath_, "/tmp/arucoimages");

    // Subscribe to input video feed and publish output video feed
    it_ = image_transport::ImageTransport(nh_);
    image_sub_ = it_.subscribeCamera("input_image", 1, &ArucoLocalizer::cameraCallback, this);
    image_pub_ = it_.advertise("output_image", 1);

    // Create ROS publishers
    estimate_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>("estimate", 1);
    meas_pub_ = nh_private_.advertise<aruco_localization::MarkerMeasurementArray>("measurements", 1);
    marker101_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>("marker101", 1);

    // Create ROS services
    calib_attitude_ = nh_private_.advertiseService("calibrate_attitude", &ArucoLocalizer::calibrateAttitude, this);

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
    mDetector_.setCornerRefinementMethod(aruco::MarkerDetector::LINES);

    // set markmap size. Convert to meters if necessary
    if (mmConfig_.isExpressedInPixels())
        mmConfig_ = mmConfig_.convertToMeters(markerSize_);

    // Configuring of Pose Tracker is done once a
    // CameraInfo message has been received

    //
    // Misc
    //

    // Initialize the attitude bias to zero
    quat_att_bias_.setRPY(0, 0, 0);

    // Create the `debug_image_path` if it doesn't exist
    std::experimental::filesystem::create_directories(debugImagePath_);
}

// ----------------------------------------------------------------------------

ArucoLocalizer::~ArucoLocalizer() { }

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

bool ArucoLocalizer::calibrateAttitude(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    // Get the latest attitude in the body frame (is this in the body frame?)
    tf::StampedTransform transform;
    tf_listener_.lookupTransform("base", "chiny", ros::Time(0), transform);

    // Store the old bias correction term to correctly capture the original biased attitude
    tf::Quaternion q0(quat_att_bias_.x() ,quat_att_bias_.y(), quat_att_bias_.z(), quat_att_bias_.w());

    // extract the inverse of the current attitude, unbiased using the current quat bias term
    // Get the original biased attitude by multiplying by the old bias correction term
    quat_att_bias_ = transform.getRotation()*q0;

    // Let the caller know what the new zeroed setpoint is
    double r, p, y;
    tf::Matrix3x3(quat_att_bias_).getRPY(r,p,y);
    res.success = true;
    res.message = "Zeroed at: R=" + std::to_string(r*(180.0/M_PI)) + ", P=" + std::to_string(p*(180.0/M_PI));
    return true;
}

// ----------------------------------------------------------------------------

void ArucoLocalizer::sendtf(const cv::Mat& rvec, const cv::Mat& tvec) {

    // We want all transforms to use the same exact time
    ros::Time now = ros::Time::now();

    // Create the transform from the camera to the ArUco Marker Map
    tf::Transform transform = aruco2tf(rvec, tvec);

    tf_br_.sendTransform(tf::StampedTransform(transform.inverse(), now, "aruco", "camera"));

    // Publish measurement of the pose of the ArUco board w.r.t the camera frame
    geometry_msgs::PoseStamped poseMsg;
    tf::poseTFToMsg(transform, poseMsg.pose);
    poseMsg.header.frame_id = "camera";
    poseMsg.header.stamp = now;
    estimate_pub_.publish(poseMsg);
}

// ----------------------------------------------------------------------------

void ArucoLocalizer::processImage(cv::Mat& frame, bool drawDetections) {

    // Detection of the board
    std::vector<aruco::Marker> detected_markers = mDetector_.detect(frame);

    if (drawDetections) {
        // print the markers detected that belongs to the markerset
        for (auto idx : mmConfig_.getIndices(detected_markers))
            detected_markers[idx].draw(frame, cv::Scalar(0, 0, 255), 1);
    }

    //
    // Calculate pose of each individual marker w.r.t the camera
    //

    aruco_localization::MarkerMeasurementArray measurement_msg;
    measurement_msg.header.frame_id = "camera";

    double avg_depth = 0;

    for (auto marker : detected_markers) {
        // Create Tvec, Rvec based on the camera and marker geometry
        marker.calculateExtrinsics(markerSize_, camParams_, false);

        // Create the ROS pose message and add to the array
        aruco_localization::MarkerMeasurement msg;
        msg.position.x = marker.Tvec.at<float>(0);
        msg.position.y = marker.Tvec.at<float>(1);
        msg.position.z = marker.Tvec.at<float>(2);

        avg_depth += msg.position.z;

        // Represent Rodrigues parameters as a quaternion
        tf::Quaternion quat = rodriguesToTFQuat(marker.Rvec);

        // Extract Euler angles
        double r, p, y;
        tf::Matrix3x3(quat).getRPY(r,p,y);

        msg.euler.x = r*180/M_PI;
        msg.euler.y = p*180/M_PI;
        msg.euler.z = y*180/M_PI;

        // Convert back to Euler and create orientation msg
        quat = tf::createQuaternionFromRPY(r,p,y);
        tf::quaternionTFToMsg(quat, msg.orientation);

        // attach the ArUco ID to this measurement
        msg.aruco_id = marker.id;

        measurement_msg.poses.push_back(msg);

        if (marker.id == 101) std::cout << msg << std::endl;

    }
    std::cout << "Avg depth: " << avg_depth/detected_markers.size() << std::endl;
    meas_pub_.publish(measurement_msg);

    //
    // Calculate pose of the entire marker map w.r.t the camera
    //

    // If the Pose Tracker was properly initialized, find 3D pose information
    if (mmPoseTracker_.isValid()) {
        if (mmPoseTracker_.estimatePose(detected_markers)) {

            if (drawDetections)
                aruco::CvDrawingUtils::draw3dAxis(frame, camParams_, mmPoseTracker_.getRvec(), mmPoseTracker_.getTvec(), mmConfig_[0].getMarkerSize()*2);

            sendtf(mmPoseTracker_.getRvec(), mmPoseTracker_.getTvec());
        }
    }

}

// ----------------------------------------------------------------------------

void ArucoLocalizer::cameraCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cinfo) {

    // static int counter = 0;

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // update the camera model with the camera's intrinsic parameters
    // cam_model_.fromCameraInfo(cinfo);

    // Configure the Pose Tracker if it has not been configured before
    if (!mmPoseTracker_.isValid() && mmConfig_.isExpressedInMeters()) {

        camParams_ = ros2arucoCamParams(cinfo);

        // Now, if the camera params have been ArUco-ified, set up the tracker
        if (camParams_.isValid())
            mmPoseTracker_.setParams(camParams_, mmConfig_);

    }

    // ==========================================================================
    // Process the incoming video frame

    // Get image as a regular Mat
    cv::Mat frame = cv_ptr->image;

    if (debugSaveInputFrames_) saveInputFrame(frame);

    // Process the image and do ArUco localization on it
    processImage(frame, showOutputVideo_);

    if (debugSaveOutputFrames_) saveOutputFrame(frame);

    if (showOutputVideo_) {
        // Update GUI Window
        cv::imshow("detections", frame);
        cv::waitKey(1);
    }

    // ==========================================================================

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
}

// ----------------------------------------------------------------------------

aruco::CameraParameters ArucoLocalizer::ros2arucoCamParams(const sensor_msgs::CameraInfoConstPtr& cinfo) {
    cv::Mat cameraMatrix(3, 3, CV_64FC1);
    cv::Mat distortionCoeff(4, 1, CV_64FC1);
    cv::Size size(cinfo->height, cinfo->width);

    // Make a regular 3x3 K matrix from CameraInfo
    for(int i=0; i<9; ++i)
        cameraMatrix.at<double>(i%3, i-(i%3)*3) = cinfo->K[i];

    // The ArUco library requires that there are only 4 distortion params (k1, k2, p1, p2, 0) 
    if (cinfo->D.size() == 4 || cinfo->D.size() == 5) {

        // Make a regular 4x1 D matrix from CameraInfo
        for(int i=0; i<4; ++i)
            distortionCoeff.at<double>(i, 0) = cinfo->D[i];

    } else {

        ROS_WARN("[aruco] Length of distortion matrix is not 4, assuming zero distortion.");
        for(int i=0; i<4; ++i)
            distortionCoeff.at<double>(i, 0) = 0;

    }

    return aruco::CameraParameters(cameraMatrix, distortionCoeff, size);
}

// ----------------------------------------------------------------------------

// From ArUco to camera frame
tf::Transform ArucoLocalizer::aruco2tf(const cv::Mat& rvec, const cv::Mat& tvec) {
    // convert tvec to a double
    cv::Mat tvec64; tvec.convertTo(tvec64, CV_64FC1);

    // Convert Rodrigues paramaterization of the rotation to quat
    tf::Quaternion q1 = rodriguesToTFQuat(rvec);

    // Extract Euler angles
    double r, p, y;
    tf::Matrix3x3(q1).getRPY(r,p,y);

    // Convert back to Euler and create orientation msg
    q1 = tf::createQuaternionFromRPY(r,p,y);

    tf::Vector3 origin(tvec64.at<double>(0), tvec64.at<double>(1), tvec64.at<double>(2));

    // The measurements coming from ArUco are vectors from the camera coordinate system
    // pointing at the center of the ArUco board.
    return tf::Transform(q1, origin);
}

// ----------------------------------------------------------------------------

tf::Quaternion ArucoLocalizer::rodriguesToTFQuat(const cv::Mat& rvec) {
    // convert rvec to double
    cv::Mat rvec64; rvec.convertTo(rvec64, CV_64FC1);

    // Unpack Rodrigues paramaterization of the rotation
    cv::Mat rot(3, 3, CV_64FC1);
    cv::Rodrigues(rvec64, rot);

    // Convert OpenCV to tf matrix
    tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                         rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                         rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

    // convert rotation matrix to an orientation quaternion
    tf::Quaternion quat;
    tf_rot.getRotation(quat);

    // For debugging
    // double r, p, y;
    // tf::Matrix3x3(quat).getRPY(r,p,y);
    // std::cout << "RPY: [ " << r*(180.0/M_PI) << ", " << p*(180.0/M_PI) << ", " << y*(180.0/M_PI) << " ]\t";

    return quat;
}

// ----------------------------------------------------------------------------

void ArucoLocalizer::saveInputFrame(const cv::Mat& frame) {
    static unsigned int counter = 0;
    saveFrame(frame, "aruco%03i_in.png", counter++);
}

// ----------------------------------------------------------------------------

void ArucoLocalizer::saveOutputFrame(const cv::Mat& frame) {
    static unsigned int counter = 0;
    saveFrame(frame, "aruco%03i_out.png", counter++);
}

// ----------------------------------------------------------------------------

void ArucoLocalizer::saveFrame(const cv::Mat& frame, std::string format_spec, unsigned int img_num) {
    // Create a filename
    std::stringstream ss;
    char buffer[100];
    sprintf(buffer, format_spec.c_str(), img_num);
    ss << debugImagePath_ << "/" << buffer;
    std::string filename = ss.str();

    // save the frame
    cv::imwrite(filename, frame);
}

// ----------------------------------------------------------------------------

}