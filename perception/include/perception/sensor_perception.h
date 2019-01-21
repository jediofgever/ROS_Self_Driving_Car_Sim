/**
 * @author Fetullah Atas
 * @email [fetulahatas1@gmail.com]
 * @create date 2018-11-06 17:34:53
 * @modify date 2018-11-06 17:35:07
 * @desc [description]
*/
 
#ifndef _SENSORPERCEPTION_H
#define _SENSORPERCEPTION_H

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

class SensorPerception {
   public:
    // Constructor
    SensorPerception();

    // Destructor
    ~SensorPerception();

    // Get the recorded Lidar scan pointcloud.
    // return: Pointcloud ROS type.
    const sensor_msgs::PointCloud2 GetLidarScan() const;

    // Set the recorded Lidar scan pointcloud.
    // param [in] value: Pointcloud ROS type.
    void SetLidarScan(sensor_msgs::PointCloud2 value);

    // Get the recorded camera image.
    // return: Image matrix.
    const cv::Mat GetCameraImage() const;

    // Set the recorded camera image.
    // param [in] value: Image matrix.
    void SetCameraImage(cv::Mat value);

   private:
    // Callback from LIDAR scan topic subscriber.
    void LidarScanCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

    //  Callback from camera image topic subscriber.
    void CameraImageCallback(const sensor_msgs::Image::ConstPtr &msg);

    // Subscriber to lidar scan data topic.
    ros::Subscriber lidar_subscriber_;

    // Lidar scan pointcloud.
    sensor_msgs::PointCloud2 lidar_scan_;

    // Camera image topic subscriber.
    ros::Subscriber camera_subscriber_;

    // Captured camera image.
    cv::Mat camera_image_;

    // ROS node handler pointer.
    ros::NodeHandlePtr nh_;
};
#endif
