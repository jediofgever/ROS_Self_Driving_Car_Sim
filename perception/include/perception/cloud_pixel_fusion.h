/**
 * @author Fetullah Atas
 * @email [fetulahatas1@gmail.com]
 * @create date 2018-11-06 17:35:33
 * @modify date 2018-11-06 17:35:33
 * @desc [description]
*/
 


#ifndef PROJECT_PIXEL_CLOUD_FUSION_H
#define PROJECT_PIXEL_CLOUD_FUSION_H

#define __APP_NAME__ "cloud_pixel_fusion"

#include <chrono>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class CloudPixelFusion {
   public:
    // construct
    CloudPixelFusion();
    // deconstruct
    ~CloudPixelFusion();

    // Process Fusion
    void ProcessCloudPixelFusion(sensor_msgs::PointCloud2ConstPtr &incloud,
                                 cv::Mat *current_frame_);

    void ShowImage();

   private:
    tf::StampedTransform FindTransform(const std::string &in_target_frame,
                                       const std::string &in_source_frame,
                                       tf::TransformListener *tf_);

    pcl::PointXYZ TransformPointPCL(const pcl::PointXYZ &in_point,
                                    const tf::StampedTransform &in_transform);

    bool IsPointinBoundries(geometry_msgs::Point32 *point, float max_distance,
                            float min_distance, float filter_angle);

    bool Project3DPoint(const geometry_msgs::Point32 &d3_point,
                        cv::Point *img_point);

    // ROS node handler pointer.
    ros::NodeHandlePtr nh_;

    // in order to publish rgb colored point cloud
    ros::Publisher publisher_fused_cloud_;

    // to keep transform of base_link and center_laser_link
    tf::StampedTransform camera_lidar_tf_;

    // tf listener pointer
    tf::TransformListener *tf_;

    // base_link string and center_laser_link strings
    std::string base_frame_id_;
    std::string cloud_frame_id_;

    // colorful poincloud from pcl
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud_;

    // filering pointclouds that are in view variables
    float dbg_projection_max_distance_;
    float dbg_projection_min_distance_;
    float dbg_projection_max_obstacle_size_;
    float dbg_projection_filter_angle_;

    int img_width, img_height;
    float fov_h;

    // keep a copy of drivers view
    cv::Mat frame_;
};

#endif
