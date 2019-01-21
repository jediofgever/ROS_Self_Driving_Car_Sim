/**
 * @author Fetullah Atas
 * @email [fetulahatas1@gmail.com]
 * @create date 2018-11-06 17:36:16
 * @modify date 2018-11-06 17:36:16
 * @desc [description]
*/
#include <perception/perception_node.h>

// Main
int main(int argc, char **argv) {
    // ROS init.
    int node_loop_rate = 10;              // ROS node execution frequency (Hz).
    ros::init(argc, argv, "perception");  // Initialize ROS module.
    ros::NodeHandle n;                    // ROS node handler.
    ros::Rate loop_rate(node_loop_rate);  // Loop rate object (50Hz).
    // Create Instance of perception_node to execute
    PerceptionNode perception_node;

    // start spinning Ros with defined rate
    while (ros::ok()) {
        perception_node.ProcessScenario();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

// Constructor
PerceptionNode::PerceptionNode() {
    // Create ROS node handler.
    nh_ = ros::NodeHandlePtr(new ros::NodeHandle());
}

// Destructor
PerceptionNode::~PerceptionNode() {
    // Bouml preserved body begin 0002C41B
}

void PerceptionNode::ProcessScenarioView() {
    // GEt drivers view image
    cv::Mat frame = sensor_perception_.GetCameraImage();

    // Show image
    if (frame.cols > 0 && frame.rows > 0) {
        cv::namedWindow("cam_dbg",
                        cv::WINDOW_AUTOSIZE);  // Create a window for display.
        cv::imshow("cam_dbg", frame);
        cv::waitKey(1);
    }
}

void PerceptionNode::ProcessScenario() {
    sensor_msgs::PointCloud2 in_cloud = sensor_perception_.GetLidarScan();
    sensor_msgs::PointCloud2::ConstPtr cld_ptr(
        new sensor_msgs::PointCloud2(in_cloud));

    // Detect Obstacles based on Occupancy Grid
    grid_cell_costmap_.DetectObstacles(cld_ptr);

    // uncomment if you want to see driver view
    // ProcessScenarioView();

    //  begin Fusion Section
    if (cld_ptr->data.size() > 0) {
        cv::Mat frame = sensor_perception_.GetCameraImage();
        cv::Mat *ptr = &frame;

        if (frame.cols > 0 && frame.rows > 0) {
            cloud_pixel_fusion_.ProcessCloudPixelFusion(cld_ptr, ptr);
            cloud_pixel_fusion_.ShowImage();
        }
        // end fusion section
    }
}
