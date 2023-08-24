//
// jplaced
// 2022, Universidad de Zaragoza
//

#ifndef EXPLORBSLAM_OCTOMAPPER_H
#define EXPLORBSLAM_OCTOMAPPER_H



#include <Eigen/Core>
#include <chrono>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen/tf2_eigen.hpp>
#include <tf2/convert.h>
#include <tf2/tf2/convert.h>
// #include "tf_conversions/tf_eigen.h"

#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
// #include <octomap_msgs/octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_ros/conversions.hpp>

#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <std_msgs/msg/bool.hpp>

#include <boost/asio/use_future.hpp>
#include <boost/asio.hpp>

// #include <boost/bind.hpp>
// #include <boost/bind/bind.hpp>
// #include <boost/serialization>

#include <orb_slam2_ros/msg/orb_state.hpp>
//#include <frontier_detector/GetOctomap.h>

class OctoMapper : public rclcpp::Node
{
public:
    // OctoMapper (ros::NodeHandle &node_handle);
    OctoMapper ();
    ~OctoMapper ();
    void Init();
    void Update ();

    int rate_hz_;
    std::string name_of_node_;

protected:
    // void initializeParams(ros::NodeHandle& nh);
    void initializeParams();
    rclcpp::Time current_frame_time_;

private:
    void PublishOctoMap ();

    void MapCallback (const sensor_msgs::msg::PointCloud2::ConstPtr& msg);
    void ORBStateCallback (const orb_slam2_ros::msg::ORBState::ConstPtr& msg);
    void GBAStateCallback (const std_msgs::msg::Bool::ConstPtr& msg);

    //bool GetOctomapSrv(frontier_detector::GetOctomap::Request &req, frontier_detector::GetOctomap::Response &res);

    bool getTfTransformMatrix(Eigen::Affine3d& transform_matrix, const std::string source_frame, const std::string target_frame);

    // Transform listener
    boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
    boost::shared_ptr<tf2_ros::TransformListener> tfListener;

    // Publishers
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_PC_subscriber_;
    // ros::Subscriber ORBstate_subscriber_;
    rclcpp::Subscription<orb_slam2_ros::msg::ORBState>::SharedPtr ORBstate_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr GBAstate_subscriber_;


    // Services
    //ros::ServiceServer octoGetter_server_;

    // ros::NodeHandle node_handle_;

    // Params
    bool publish_octomap_param_;

    bool octomap_rebuild_;

    bool update_octomap_;
    bool clear_octomap_;
    bool octomap_tf_based_;
    int lastBigMapChange_;
    int last_big_update_;
    int previous_cloud_size_;
    int since_last_big_update_;
    double max_height_param_;

    std::string map_frame_id_param_;
    std::string target_frame_id_param_;

    octomap::OcTree* octomap_ = new octomap::OcTree(0.10);
    octomap::Pointcloud* pointcloud_map_points_ = new octomap::Pointcloud;
    octomap::pose6d octo_frame_;
    octomap::point3d octo_origin_;

    Eigen::Affine3d T_optical_target_;

    bool gba_state_;
    int orb_state_;

};

#endif //EXPLORBSLAM_OCTOMAPPER_H
