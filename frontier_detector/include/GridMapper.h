//
// jplaced
// 2022, Universidad de Zaragoza
//

#ifndef EXPLORBSLAM_GRIDMAPPER_H
#define EXPLORBSLAM_GRIDMAPPER_H

#include "rclcpp/rclcpp.hpp"
// #include <ros/time.h>
#include <unistd.h>

#include <chrono>
#include <Eigen/Core>

// #include <tf/transform_datatypes.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
// #include <std_msgs/Bool.h>
#include <std_msgs/msg/bool.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <bullet/LinearMath/btVector3.h>

// #include <orb_slam2_ros/ORBState.h>
//#include <frontier_detector/GetOctomap.h>

class GridMapper : public rclcpp::Node
{
public:
    // GridMapper (ros::NodeHandle &node_handle);
    GridMapper ();
    ~GridMapper ();
    void Init();
    void Update ();

    int rate_hz_;
    std::string name_of_node_;

protected:
    // void initializeParams(ros::NodeHandle& nh);
    void initializeParams();
    rclcpp::Time current_frame_time_;

    inline std::vector<unsigned> point2Index (const nav_msgs::msg::MapMetaData& info, const geometry_msgs::msg::Point& p) {

        // geometry_msgs::msg::Point pt;
        tf2::Transform world_to_map;
        // geometry_msgs::msg::Transform world_to_map;
        // geometry_msgs::msg::TransformStamped world_to_map;
        // tf2::fromMsg(p, pt);
        // tf2::poseMsgToTF(info.origin, world_to_map);
        tf2::fromMsg(info.origin, world_to_map);
        geometry_msgs::msg::TransformStamped inverted_transform_msg;
        tf2::toMsg(world_to_map.inverse(), inverted_transform_msg.transform);
        geometry_msgs::msg::Point p2;
        tf2::doTransform(p,p2,inverted_transform_msg);
        // btVector3 p2 = world_to_map.inverse()* p;

        unsigned i, j;
        i = floor(p2.x/info.resolution);
        j = floor(p2.y/info.resolution); 

        return {i,j};
    }

private:
    void PublishOccupancyGrid ();

    void octoMapCallback (const octomap_msgs::msg::Octomap::ConstPtr& msg);
    void GBAStateCallback (const std_msgs::msg::Bool::ConstPtr& msg);

    void octomapToOccupancyGrid(const octomap::OcTree& octree, nav_msgs::msg::OccupancyGrid& map, nav_msgs::msg::OccupancyGrid& map_rect, const double minZ_, const double maxZ_ );
    nav_msgs::msg::OccupancyGrid rectifyMap(const nav_msgs::msg::OccupancyGrid& map, cv::Mat matrix_OCC, cv::Mat matrix_FREE);

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_rect_publisher_;

    // Subscribers
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr GBAstate_subscriber_;

    // Services
    //ros::ServiceClient octoGetter_client_;

    // ros::NodeHandle node_handle_;

    // Params
    bool publish_occupancy_grid_param_;
    bool publish_occupancy_grid_rect_param_;

    double projection_min_height_;
    double projection_max_height_;

    bool gba_state_;

    // Assign occupancy grid map values
    static const unsigned char MAT_UNKNOWN_ = 255;
    static const unsigned char MAT_KNOWN_ = 0; // UNOCCUPIED (0) OCCUPIED (255)

    std::string map_frame_id_param_;

    octomap::OcTree* octomap_ = new octomap::OcTree(0.05);

};

#endif //EXPLORBSLAM_GRIDMAPPER_H
