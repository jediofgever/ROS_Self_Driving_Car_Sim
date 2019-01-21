/**
 * @author Fetullah Atas
 * @email [fetulahatas1@gmail.com]
 * @create date 2018-11-06 17:35:50
 * @modify date 2018-11-06 17:35:50
 * @desc [description]
*/
 

#ifndef _PerceptionNode_H
#define _PerceptionNode_H

#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "cloud_pixel_fusion.h"
#include "grid_cell_costmap.h"
#include "sensor_perception.h"

// Execution of GridCell , Obstacle Detection are in this class
class PerceptionNode {
   public:
    // Constructor
    PerceptionNode();

    // Destructor
    ~PerceptionNode();

    // Process Obstacle Detection , Ocuuancy Grid and All  other Perception
    // Tasks
    void ProcessScenario();

    // Visualizes camera/driver view , Optional
    void ProcessScenarioView();

   private:
    // Sensor perceptions.
    SensorPerception sensor_perception_;

    // ROS node handler pointer.
    ros::NodeHandlePtr nh_;

    // Instance of GridCell Costmap , We do Obstacle detection in this Instance
    // as well
    GridCellCostmap grid_cell_costmap_;

    CloudPixelFusion cloud_pixel_fusion_;
};
#endif
