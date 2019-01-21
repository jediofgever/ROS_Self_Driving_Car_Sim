/**
 * @author Fetullah Atas
 * @email [fetulahatas1@gmail.com]
 * @create date 2018-11-06 17:36:21
 * @modify date 2018-11-06 17:36:21
 * @desc [description]
*/

#include <perception/sensor_perception.h>

// Constructor
SensorPerception::SensorPerception() {
    // Create ROS node handler.
    nh_ = ros::NodeHandlePtr(new ros::NodeHandle());

    // Subscriber to lidar scan data topic.
    lidar_subscriber_ =
        nh_->subscribe("/prius/center_laser/scan", 1,
                       &SensorPerception::LidarScanCallback, this);

    // Camera image topic subscriber.
    camera_subscriber_ =
        nh_->subscribe("/prius/front_camera/image_raw", 1,
                       &SensorPerception::CameraImageCallback, this);
}

// Destructor
SensorPerception::~SensorPerception() {}

// Get the recorded Lidar scan pointcloud.
// return: Pointcloud ROS type.
const sensor_msgs::PointCloud2 SensorPerception::GetLidarScan() const {
    return lidar_scan_;
}

// Set the recorded Lidar scan pointcloud.
// param [in] value: Pointcloud ROS type.
void SensorPerception::SetLidarScan(sensor_msgs::PointCloud2 value) {
    lidar_scan_ = value;
}

// Get the recorded camera image.
// return: Image matrix.
const cv::Mat SensorPerception::GetCameraImage() const { return camera_image_; }

// Set the recorded camera image.
// param [in] value: Image matrix.
void SensorPerception::SetCameraImage(cv::Mat value) { camera_image_ = value; }

// Callback from LIDAR scan topic subscriber.
void SensorPerception::LidarScanCallback(
    const sensor_msgs::PointCloud2::ConstPtr &msg) {
    SetLidarScan(*msg.get());
}

//  Callback from camera image topic subscriber.
void SensorPerception::CameraImageCallback(
    const sensor_msgs::Image::ConstPtr &msg) {
    cv_bridge::CvImageConstPtr cv_ptr;  // CV bridge image.
    try {
        // Copy the Image msg into the CV bridge variable.
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        return;
    }
    cv::Mat CameraImg =
        cv_ptr->image;  // Obtain the openCV image from the CVbridge.

    SetCameraImage(CameraImg);
}
