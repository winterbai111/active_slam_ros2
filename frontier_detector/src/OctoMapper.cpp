//
// jplaced
// 2022, Universidad de Zaragoza
//

#include "OctoMapper.h"

//#define OCTO_RESOLUTION_ 0.05
#define OCTO_MIN_POINTS_TH 1000
#define OCTO_CLEAR_TH_ 4

int main(int argc, char **argv) {

    // ros::init(argc, argv, "Octomapper");
    // ros::start();

    rclcpp::init(argc, argv);
    // auto node = rclcpp::Node::make_shared("Octomapper");

    auto node = std::make_shared<OctoMapper>();

    if(argc > 1) {
        RCLCPP_WARN (node->get_logger(),"Arguments supplied via command line are neglected.");
    }

    // ros::NodeHandle node_handle;
    // OctoMapper OCTO_NODE (node_handle);
    // OctoMapper OCTO_NODE;
    // node->Init();
    // rclcpp::Rate r(int(node->rate_hz_));
    
    rclcpp::Rate r(1);
    while (rclcpp::ok()) {
        // RCLCPP_INFO (node->get_logger(),"Octo loop");
        // Update and publish OctoMap with extracted PC
        node->Update ();

        // Spin ROS with rate r
        rclcpp::spin_some(node);
        r.sleep();

    }

    rclcpp::shutdown();

    return 0;

}

// OctoMapper::OctoMapper (ros::NodeHandle &node_handle) {

OctoMapper::OctoMapper() : 
    Node("octomapper"){
    // name_of_node_ = ros::this_node::getName();
    name_of_node_ = this->get_name();
    // node_handle_ = node_handle;
    previous_cloud_size_ = 0;
    since_last_big_update_ = 0;
    octo_origin_ = { 0.0, 0.0, 0.0 };
    octo_frame_ = octomap::pose6d(0, 0, 0, 0, 0, 0);
    update_octomap_ = false;
    clear_octomap_ = false;
    T_optical_target_ = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
    gba_state_ = false;
    orb_state_ = 0; // Not initialized
    Init ();
}


OctoMapper::~OctoMapper () {

    delete octomap_;
    delete pointcloud_map_points_;

}


void OctoMapper::Init () {

    RCLCPP_INFO(this->get_logger(),"%s : Initializing.", name_of_node_.c_str());

    // Static ROS parameters
    // initializeParams(node_handle_);
    initializeParams();

    // Initialization transformation listener
    // tfBuffer.reset(new tf2_ros::Buffer);
    // tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));
    tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);

    // Enable publishing OctoMap
    octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>(name_of_node_+"/octomap", 3);
    // PointCloud subscriber
    map_PC_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("RGBD/map_points", 10, std::bind(&OctoMapper::MapCallback, this, std::placeholders::_1));
    ORBstate_subscriber_ = this->create_subscription<orb_slam2_ros::msg::ORBState>("RGBD/info/state", 10, std::bind(&OctoMapper::ORBStateCallback, this, std::placeholders::_1));
    GBAstate_subscriber_ = this->create_subscription<std_msgs::msg::Bool> ("RGBD/gba_running", 10, std::bind(&OctoMapper::GBAStateCallback, this, std::placeholders::_1));

    // Services
    //octoGetter_server_ = node_handle_.advertiseService(name_of_node_+"/get_octomap", &OctoMapper::GetOctomapSrv, this);

    // getTfTransformMatrix(T_optical_target_, "camera_link_optical", target_frame_id_param_);
    // getTfTransformMatrix(T_optical_target_, "odom", "base_link");

    RCLCPP_INFO(this->get_logger(),"%s : Initialized successfully.", name_of_node_.c_str());

}

// void OctoMapper::initializeParams(ros::NodeHandle &nh){
void OctoMapper::initializeParams(){
    // nh.param<int>(name_of_node_+ "/rate", rate_hz_, int(1));
    // nh.param<bool>(name_of_node_+ "/octomap/publish_octomap", publish_octomap_param_, true);
    // nh.param<std::string>(name_of_node_+ "/target_frame_id", target_frame_id_param_, "base_footprint");
    // nh.param<std::string>(name_of_node_+ "/pointcloud_frame_id", map_frame_id_param_, "map");
    // nh.param<double>(name_of_node_+ "/max_height", max_height_param_, 3.0);
    // nh.param<bool>(name_of_node_+"/octomap/rebuild",octomap_rebuild_, false);

    this->declare_parameter(name_of_node_+ "/rate",rclcpp::ParameterValue(int(1)) );
    this->declare_parameter(name_of_node_+ "/octomap/publish_octomap",rclcpp::ParameterValue(true) );
    this->declare_parameter(name_of_node_+ "/target_frame_id", rclcpp::ParameterValue(std::string("base_link")));
    this->declare_parameter(name_of_node_+ "/pointcloud_frame_id", rclcpp::ParameterValue(std::string("map")));
    this->declare_parameter(name_of_node_+"/max_height", rclcpp::ParameterValue(3.0));
    this->declare_parameter(name_of_node_+"/octomap/rebuild",  rclcpp::ParameterValue(false));

    this->get_parameter(name_of_node_+ "/rate", rate_hz_);
    this->get_parameter(name_of_node_+ "/octomap/publish_octomap", publish_octomap_param_);
    this->get_parameter(name_of_node_+ "/target_frame_id", target_frame_id_param_);
    this->get_parameter(name_of_node_+ "/pointcloud_frame_id", map_frame_id_param_);
    this->get_parameter(name_of_node_+"/max_height",max_height_param_);
    this->get_parameter(name_of_node_+"/octomap/rebuild", octomap_rebuild_);


}


void OctoMapper::ORBStateCallback(const orb_slam2_ros::msg::ORBState::ConstPtr &msg) {

    if (msg->state == 4) orb_state_ = 1; // OK
    else if (msg->state == 5) orb_state_ = 2; // LOST
    else orb_state_ = 0; // UNKNOWN, SYSTEM_NOT_READY, NOT_INITIALIZED

}


void OctoMapper::GBAStateCallback(const std_msgs::msg::Bool::ConstPtr &msg) {

    gba_state_ = msg->data;

}


void OctoMapper::MapCallback (const sensor_msgs::msg::PointCloud2::ConstPtr& msg) {
    /* This callback consumes little time, so pull all map points every time
     */

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    if (orb_state_ == 1) { // ORB-SLAM is OK
        // Convert input cloud from sensor_msgs::PointCloud2 to sensor_msgs::PointCloud
        sensor_msgs::msg::PointCloud out_pointcloud;
        sensor_msgs::convertPointCloud2ToPointCloud(*msg, out_pointcloud);

        update_octomap_ = false; // Avoid OctoMap update at the same time points are being updated

        // Convert cloud from sensor_msgs::PointCloud to octomap::Pointcloud
        pointcloud_map_points_->clear();
        for (int i = 0; i < out_pointcloud.points.size(); i++) {
            if(out_pointcloud.points[i].z < max_height_param_) {
                Eigen::Vector3d map_pt(out_pointcloud.points[i].x,
                                       out_pointcloud.points[i].y,
                                       out_pointcloud.points[i].z);

                pointcloud_map_points_->push_back(map_pt[0], map_pt[1], map_pt[2]);
            }
        }

        update_octomap_ = true;
        RCLCPP_DEBUG(this->get_logger(),"%s: Pulled PC with %li points.", name_of_node_.c_str(), pointcloud_map_points_->size());

    } else if (orb_state_ == 2){
        update_octomap_ = false;
        RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),1,"%s: Not updating OctoMap if ORB-SLAM is LOST.", name_of_node_.c_str());
    } else {
        update_octomap_ = false;
        clear_octomap_ = true;
        RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),1,"%s: Not updating OctoMap if ORB-SLAM is UNKNOWN, NOT_READY, NOT_INITIALIZED.", name_of_node_.c_str());
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = end - begin;
    RCLCPP_DEBUG(this->get_logger(),"%s: PointCloud reading took %f [s].", name_of_node_.c_str(), diff.count());
}


void OctoMapper::Update () {
    /* This update does consume time, so do it when necessary:
     * Only update if at least OCTO_MIN_POINTS_TH were added to the
     * PointCloud or if it has not been updated for OCTO_CLEAR_TH_ times
     */

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Not updating during GBAs or while ORB-SLAM not OK
    if (gba_state_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),1, "%s: Not updating OctoMap during GBA.", name_of_node_.c_str());
    } else if (clear_octomap_){
        // TODO create clearing service?
        octomap_->clear();
        clear_octomap_ = false;
        PublishOctoMap();
        RCLCPP_WARN(this->get_logger(),"%s: Cleared OctoMap.", name_of_node_.c_str());
    } else if (update_octomap_){

        int points_difference = pointcloud_map_points_->size() - previous_cloud_size_;
        points_difference = abs(points_difference);

        // Add all points (slow update)
        if (points_difference >= OCTO_MIN_POINTS_TH || since_last_big_update_ >= OCTO_CLEAR_TH_) {

            octomap_->clear();
            octomap_->insertPointCloud(*pointcloud_map_points_, octo_origin_, octo_frame_);

            previous_cloud_size_ = pointcloud_map_points_->size();
            since_last_big_update_ = 0;

            //ROS_DEBUG("%s: OctoMap updated with %li points.", name_of_node_.c_str(), pointcloud_map_points_->size());
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::chrono::duration<double> diff = end - begin;
            RCLCPP_DEBUG(this->get_logger(),"%s: OctoMap full update took %f [s].", name_of_node_.c_str(), diff.count());

        // Add only recent points (fast update). Old points won't be updated nor deleted
        } else if (points_difference > 0) {
            since_last_big_update_++;
            octomap::Pointcloud pointcloud_map_points_reduced;

            int start = (int) previous_cloud_size_/1.1; // heuristic
            for (int i = start; i < pointcloud_map_points_->size(); i++) {
                pointcloud_map_points_reduced.push_back(pointcloud_map_points_->getPoint(i));
            }

            octomap_->insertPointCloud(pointcloud_map_points_reduced, octo_origin_, octo_frame_);
            previous_cloud_size_ = pointcloud_map_points_->size();

            //ROS_DEBUG("%s: OctoMap updated with %li points.", name_of_node_.c_str(), pointcloud_map_points_reduced.size());
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::chrono::duration<double> diff = end - begin;
            RCLCPP_DEBUG(this->get_logger(),"%s: OctoMap incremental update took %f [s].", name_of_node_.c_str(), diff.count());
        }

        // Publishers
        PublishOctoMap();

    }

}


void OctoMapper::PublishOctoMap () {

    // if (octomap_publisher_.getNumSubscribers() > 0){
    if (octomap_publisher_->get_subscription_count() > 0){    
        octomap_msgs::msg::Octomap msgOctomap;
        msgOctomap.header.frame_id = map_frame_id_param_;
        msgOctomap.header.stamp = this->get_clock()->now();
        if (octomap_msgs::binaryMapToMsg(*octomap_, msgOctomap)) {
            octomap_publisher_->publish(msgOctomap);
            // RCLCPP_INFO(this->get_logger(),"Octomap_publish!");
        }
    }

}


bool OctoMapper::getTfTransformMatrix(Eigen::Affine3d& transform_matrix, const std::string source_frame, const std::string target_frame) {

    try {
        geometry_msgs::msg::TransformStamped transform_to_robot = tfBuffer->lookupTransform(target_frame, source_frame,rclcpp::Time(0),rclcpp::Duration::from_seconds(2));//,rclcpp::Duration::from_seconds(0.05));

        transform_matrix = tf2::transformToEigen(transform_to_robot);
        return true;
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(),"%s",ex.what());
        return false;
    }

}


/*
bool OctoMapper::GetOctomapSrv(frontier_detector::GetOctomap::Request  &req,
                               frontier_detector::GetOctomap::Response &res)
{
    //ros::WallTime startTime = ros::WallTime::now();
    //RCLCPP_INFO("Sending binary map data on service request");

    res.map.header.frame_id = map_frame_id_param_;
    res.map.header.stamp = rclcpp::Time::now();
    if (!octomap_msgs::binaryMapToMsg(*octomap_, res.map))
        return false;

    //double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    //RCLCPP_INFO("Binary octomap sent in %f sec", total_elapsed);
    return true;
}
 */
