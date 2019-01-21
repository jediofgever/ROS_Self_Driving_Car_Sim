/**
 * @author Fetullah Atas
 * @email [fetulahatas1@gmail.com]
 * @create date 2018-11-06 17:36:06
 * @modify date 2018-11-06 17:36:06
 * @desc [description]
*/
#include "perception/cloud_pixel_fusion.h"

CloudPixelFusion::CloudPixelFusion() {
    // init node handler pointer
    nh_ = ros::NodeHandlePtr(new ros::NodeHandle());

    // Publish rgb colored pointcloud
    publisher_fused_cloud_ =
        nh_->advertise<sensor_msgs::PointCloud2>("fused_point_cl", 1);

    // tf listener
    tf_ = new tf::TransformListener(ros::Duration(1));

    // frames
    base_frame_id_ = "/front_camera_link";
    cloud_frame_id_ = "/center_laser_link";

    // filtering params
    nh_->param<float>("dbg_projection_max_distance",
                      dbg_projection_max_distance_, 40.0);
    nh_->param<float>("dbg_projection_min_distance",
                      dbg_projection_min_distance_, 1.0);
    nh_->param<float>("dbg_projection_filter_angle",
                      dbg_projection_filter_angle_, 1.7);

    nh_->param<int>("camera_img_width", img_width, 1600);
    nh_->param<int>("camera_img_height", img_height, 800);
    nh_->param<float>("camera_horizontal_fov", fov_h, 1.62);
}

// deconstructor
CloudPixelFusion::~CloudPixelFusion() {}

tf::StampedTransform CloudPixelFusion::FindTransform(
    const std::string &in_target_frame, const std::string &in_source_frame,
    tf::TransformListener *tf_) {
    tf::StampedTransform transform;

    try {
        tf_->waitForTransform(in_target_frame, in_source_frame, ros::Time(0),
                              ros::Duration(0.0));
        tf_->lookupTransform(in_target_frame, in_source_frame, ros::Time(0),
                             transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("[%s] %s", ex.what());
    }

    return transform;
}

pcl::PointXYZ CloudPixelFusion::TransformPointPCL(
    const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform) {
    tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
    tf::Vector3 tf_point_t = in_transform * tf_point;
    return pcl::PointXYZ(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
}

// Method to filter out Safety area  that is not in view of camera for more
// efficient processing (needs to be moved to utils)
// param [in] in_obstacle_array: List of obstacles to be filtered.
// return: Filtered result array.
bool CloudPixelFusion::IsPointinBoundries(geometry_msgs::Point32 *point,
                                          float max_distance,
                                          float min_distance,
                                          float filter_angle) {
    // Define range for obstacles
    if (point->x > min_distance && point->x < max_distance) {
        double point_distance = sqrt(pow(point->x, 2) + pow(point->y, 2));
        double boundry_line_y = point_distance * sin(filter_angle);

        if (point->y < boundry_line_y) {
            return true;
        }
    }

    return false;
}

// Raytrace 3D point into the corresponding 2D Image pixel.
// param [in] img_point: 3D point.
// param [out] proj_point: 2D projected pixel coordinates.
// return : Projection success.
bool CloudPixelFusion::Project3DPoint(const geometry_msgs::Point32 &d3_point,
                                      cv::Point *img_point) {
    float x = d3_point.x - camera_lidar_tf_.getOrigin().x();

    float y = d3_point.y - camera_lidar_tf_.getOrigin().y();

    float z = d3_point.z - camera_lidar_tf_.getOrigin().z();

    float vertical_ang = (z / x);
    float horizontal_ang = (y / x);

    float ar = (float)img_height / img_width;
    float fov_v = fov_h * ar;
    float PSv = (float)fov_v / img_height;
    float PSh = (float)fov_h / img_width;

    img_point->x = -horizontal_ang / PSh + img_width / 2;
    img_point->y = -vertical_ang / PSv + img_height / 2;

    // ROS_INFO("x %.2f,z %.2f", camera_lidar_tf_.getOrigin().x(),
    // camera_lidar_tf_.getOrigin().z());
}

void CloudPixelFusion::ProcessCloudPixelFusion(
    sensor_msgs::PointCloud2ConstPtr &incloud, cv::Mat *current_frame_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*incloud, *in_cloud);

    camera_lidar_tf_ = FindTransform(base_frame_id_, cloud_frame_id_, tf_);

    frame_ = current_frame_->clone();

    std::vector<pcl::PointXYZ> cam_cloud(in_cloud->points.size());
    for (std::size_t i = 0; i < in_cloud->points.size(); i++) {
        cam_cloud[i] = TransformPointPCL(in_cloud->points[i], camera_lidar_tf_);

        cv::Point img_point;
        geometry_msgs::Point32 point;
        point.x = cam_cloud[i].x;
        point.y = cam_cloud[i].y;
        point.z = cam_cloud[i].z;

        if (IsPointinBoundries(&point, dbg_projection_max_distance_,
                               dbg_projection_min_distance_,
                               dbg_projection_filter_angle_)) {
            if (Project3DPoint(point, &img_point)) {
                int u = img_point.x;
                int v = img_point.y;

                if ((u >= 0) && (u < current_frame_->cols) && (v >= 0) &&
                    (v < current_frame_->rows)) {
                    double distance =
                        sqrt(pow(cam_cloud[i].x, 2) + pow(cam_cloud[i].y, 2) +
                             pow(cam_cloud[i].z, 2));
                    cv::Scalar clr = cv::Scalar(255, 0, distance * 15);
                    cv::circle(frame_, img_point, 1, clr, 1);

                    pcl::PointXYZRGB colored_3d_point;

                    cv::Vec3b rgb_pixel = current_frame_->at<cv::Vec3b>(v, u);
                    colored_3d_point.x = in_cloud->points[i].x;
                    colored_3d_point.y = in_cloud->points[i].y;
                    colored_3d_point.z = in_cloud->points[i].z;
                    colored_3d_point.r = rgb_pixel[2];
                    colored_3d_point.g = rgb_pixel[1];
                    colored_3d_point.b = rgb_pixel[0];
                    out_cloud->points.push_back(colored_3d_point);
                }
            }
        }
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*out_cloud, cloud_msg);
    cloud_msg.header = incloud->header;
    publisher_fused_cloud_.publish(cloud_msg);
}

void CloudPixelFusion::ShowImage() {
    cv::namedWindow("Drivers view", cv::WINDOW_AUTOSIZE);
    cv::imshow("Drivers view", frame_);
    cv::waitKey(1);
}
