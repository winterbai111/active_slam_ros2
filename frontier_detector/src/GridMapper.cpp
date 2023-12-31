//
// jplaced
// 2022, Universidad de Zaragoza
//

#include "GridMapper.h"

int main(int argc, char **argv) {

    // ros::init(argc, argv, "Gridmapper");
    rclcpp::init(argc, argv);

    // auto node = rclcpp::Node::make_shared("Gridmapper");
    auto node = std::make_shared<GridMapper>();

    // ros::start();

    if(argc > 1) {
        RCLCPP_WARN (node->get_logger(),"Arguments supplied via command line are neglected.");
    }

    // ros::NodeHandle node_handle;
    // GridMapper GRID_NODE; // (node);
    // GRID_NODE.Init();
    // rclcpp::Rate r(int(GRID_NODE.rate_hz_));

    rclcpp::Rate r(1);

    while (rclcpp::ok()) {
        // RCLCPP_INFO (node->get_logger(),"Grid loop");
        // Update and publish GridMap
        node->Update ();

        // Spin ROS with rate r
        rclcpp::spin_some(node);
        r.sleep();

    }

    rclcpp::shutdown();

    return 0;

}

// GridMapper::GridMapper (ros::NodeHandle &node_handle) {
GridMapper::GridMapper() : Node("GridMapper_node") {

    // name_of_node_ = ros::this_node::getName();
    name_of_node_ = this->get_name();
    // node_handle_ = node_handle;
    gba_state_ = false;
    Init ();
}


GridMapper::~GridMapper () {

    delete octomap_;

}


void GridMapper::Init () {

    RCLCPP_INFO(this->get_logger(),"%s : Initializing.", name_of_node_.c_str());

    usleep(3000000);

    // Static ROS parameters
    // initializeParams(node_handle_);
    initializeParams();

    // Enable publishing octomap and occupancy grid map
    auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(5)).transient_local().reliable();
    occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(name_of_node_+"/projected_map", custom_qos); //, 10
    occupancy_grid_rect_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(name_of_node_+"/rectified_map", custom_qos);//, 10
    // occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(name_of_node_+"/projected_map", 5); //, 10
    // occupancy_grid_rect_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(name_of_node_+"/rectified_map", 5);//, 10

    // Octomap subscriber
    octomap_subscriber_  = this->create_subscription<octomap_msgs::msg::Octomap>("octomapper/octomap", 10, std::bind(&GridMapper::octoMapCallback, this, std::placeholders::_1) );
    GBAstate_subscriber_ = this->create_subscription<std_msgs::msg::Bool> ("RGBD/gba_running", 20, std::bind(&GridMapper::GBAStateCallback, this, std::placeholders::_1));

    // Services
    //octoGetter_client_ = node_handle_.serviceClient<frontier_detector::GetOctomap>("/octomapper/get_octomap");

    RCLCPP_INFO(this->get_logger(),"%s : Initialized successfully.", name_of_node_.c_str());

}

// void GridMapper::initializeParams(ros::NodeHandle &nh){
void GridMapper::initializeParams(){

    // nh.param<int>(name_of_node_+ "/rate", rate_hz_, int(1));
    // nh.param<bool>(name_of_node_+ "/occupancy/publish_occupancy_grid", publish_occupancy_grid_param_, true);
    // nh.param<bool>(name_of_node_+ "/occupancy/publish_occupancy_grid_rect", publish_occupancy_grid_rect_param_, true);
    // nh.param<std::string>(name_of_node_+ "/pointcloud_frame_id", map_frame_id_param_, "map");
    // nh.param<double>(name_of_node_+"/occupancy/projected_map/min_height", projection_min_height_,  0.15);
    // nh.param<double>(name_of_node_+"/occupancy/projected_map/max_height", projection_max_height_,  +1.5);

    this->declare_parameter(name_of_node_+ "/rate",rclcpp::ParameterValue(int(1)) );
    this->declare_parameter(name_of_node_+ "/occupancy/publish_occupancy_grid",rclcpp::ParameterValue(true) );
    this->declare_parameter(name_of_node_+ "/occupancy/publish_occupancy_grid_rect", rclcpp::ParameterValue(true));
    this->declare_parameter(name_of_node_+ "/pointcloud_frame_id", rclcpp::ParameterValue(std::string("map")));
    this->declare_parameter(name_of_node_+"/occupancy/projected_map/min_height", rclcpp::ParameterValue(0.15));
    this->declare_parameter(name_of_node_+"/occupancy/projected_map/max_height",  rclcpp::ParameterValue(1.5));

    this->get_parameter(name_of_node_+ "/rate", rate_hz_);
    this->get_parameter(name_of_node_+ "/occupancy/publish_occupancy_grid", publish_occupancy_grid_param_);
    this->get_parameter(name_of_node_+ "/occupancy/publish_occupancy_grid_rect", publish_occupancy_grid_rect_param_);
    this->get_parameter(name_of_node_+ "/pointcloud_frame_id", map_frame_id_param_);
    this->get_parameter(name_of_node_+"/occupancy/projected_map/min_height", projection_min_height_);
    this->get_parameter(name_of_node_+"/occupancy/projected_map/max_height",  projection_max_height_);

}


void GridMapper::octoMapCallback(const octomap_msgs::msg::Octomap::ConstPtr &msg) {

    if (octomap_ != NULL)
        delete (octomap_);

    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    if (tree) {
        octomap_ = dynamic_cast<octomap::OcTree*>(tree);
        // RCLCPP_INFO(this->get_logger(),"octomap received!");
    } else {
        RCLCPP_ERROR(this->get_logger(),"Failed to call convert Octomap.");
        return;
    }



}


void GridMapper::GBAStateCallback(const std_msgs::msg::Bool::ConstPtr &msg) {

    gba_state_ = msg->data;

}


void GridMapper::Update () {

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Not updating during GBAs or while ORB-SLAM not OK
    if (gba_state_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),1, "%s: Not updating GridMap during GBA.", name_of_node_.c_str());
    } else {
        // Publishers
        if (publish_occupancy_grid_param_ || publish_occupancy_grid_rect_param_)
            PublishOccupancyGrid();
    }

}


void GridMapper::PublishOccupancyGrid () { 

    static nav_msgs::msg::OccupancyGrid msg;
    static nav_msgs::msg::OccupancyGrid msg_rectified;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    octomapToOccupancyGrid(*octomap_, msg, msg_rectified, projection_min_height_, projection_max_height_);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = end - begin;
    RCLCPP_DEBUG(this->get_logger(),"%s: OccupancyGrid conversion took %f [s].", name_of_node_.c_str(), diff.count());

    if (publish_occupancy_grid_param_) {
        if (occupancy_grid_publisher_->get_subscription_count() > 0) {
            msg.header.frame_id = map_frame_id_param_;
            msg.header.stamp = this->get_clock()->now();
            occupancy_grid_publisher_->publish(msg);
            // RCLCPP_INFO(this->get_logger(),"Gridmap project publish!");
        }
    }

    if (publish_occupancy_grid_rect_param_){
        msg_rectified.header.frame_id = map_frame_id_param_;
        msg_rectified.header.stamp = this->get_clock()->now();
        occupancy_grid_rect_publisher_->publish(msg_rectified);
        // RCLCPP_INFO(this->get_logger(),"Gridmap rectified publish!");
    }

}


void GridMapper::octomapToOccupancyGrid (const octomap::OcTree& octomap, nav_msgs::msg::OccupancyGrid& map, nav_msgs::msg::OccupancyGrid& map_rect, const double minZ_, const double maxZ_) {


    // Copy OctoMap in order to maintain input
    octomap::OcTree octree(octomap);
    double resolution = octree.getResolution();

    static const uint8_t a = 1;
    static const uint8_t b = 0;
    static const uint8_t c = 2;

    map.info.resolution = resolution;
    double minX, minY, minZ;
    double maxX, maxY, maxZ;

    octree.getMetricMin(minX, minY, minZ);
    octree.getMetricMax(maxX, maxY, maxZ);

    minZ = std::max(minZ_, minZ);
    maxZ = std::min(maxZ_, maxZ);

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey, maxKey;

    if (!octree.coordToKeyChecked(minPt, minKey))
    {
        RCLCPP_ERROR(this->get_logger(),"Could not create OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
        return;
    }
    if (!octree.coordToKeyChecked(maxPt, maxKey))
    {
        RCLCPP_ERROR(this->get_logger(),"Could not create OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
        return;
    }

    map.info.width  = maxKey[b] - minKey[b] + 1;
    map.info.height = maxKey[a] - minKey[a] + 1;

    // might not exactly be min / max:
    octomap::point3d origin = octree.keyToCoord(minKey, octree.getTreeDepth());

    map.info.origin.position.x = origin.x() - resolution * 0.5;
    map.info.origin.position.y = origin.y() - resolution * 0.5;
    map.info.origin.position.z = 0.;

    map.info.origin.orientation.x = 0.;
    map.info.origin.orientation.y = 0.;
    map.info.origin.orientation.z = 0.;
    map.info.origin.orientation.w = 1.0;

    // Allocate space to hold the data
    map.data.resize(map.info.width * map.info.height, -1);

    // Matrix of map's size is inited with unknown (-1) value at each point
    for(std::vector<int8_t>::iterator it = map.data.begin(); it != map.data.end(); ++it) {
        *it = -1;
    }

    cv::Mat map_data_matrix_occ;
    map_data_matrix_occ.create(map.info.height, map.info.width, CV_8U);
    map_data_matrix_occ.setTo(MAT_UNKNOWN_);

    cv::Mat map_data_matrix_non_occ;
    map_data_matrix_non_occ.create(map.info.height, map.info.width, CV_8U);
    map_data_matrix_non_occ.setTo(MAT_UNKNOWN_);

    // Iterate over the whole OctoMap
    for(octomap::OcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it){

        if (!octree.search(it.getKey())){ //.......................................................UNKNOWN CELLS
            // Already set to MAT_UNKNOWN_ in occupancy map and matrix initialization
        } else{ //.................................................................................KNOWN CELLS
            octomap::point3d octoPos = it.getCoordinate();

            // Get index of point
            geometry_msgs::msg::Point p;
            p.x = octoPos.x();
            p.y = octoPos.y();
            p.z = octoPos.z();

            std::vector<unsigned> index = point2Index(map.info, p);

            unsigned i, j, k;
            i = index.at(0);
            j = index.at(1);
            k = map.info.width * j + i;

            if ( octree.isNodeOccupied(*it) && (p.z > minZ) && (p.z < maxZ) ){ //..................OCCUPIED CELLS
                map.data[k] = 100;
                map_data_matrix_occ.at<uchar>(j,i) = MAT_KNOWN_;
            } else if ( map.data[k] == -1 ){//.......................................................FREE CELLS
                map.data[k] = 0;
                map_data_matrix_non_occ.at<uchar>(j,i) = MAT_KNOWN_;
            }
        }
    }


    // Rectified map
    if (publish_occupancy_grid_rect_param_)
        map_rect = rectifyMap(map, map_data_matrix_occ, map_data_matrix_non_occ);

    return;

}


nav_msgs::msg::OccupancyGrid GridMapper::rectifyMap(const nav_msgs::msg::OccupancyGrid& map, cv::Mat matrix_OCC, cv::Mat matrix_FREE){

    bool filter_occupied = true;
    bool filter_free = true;

    if (filter_occupied) {
        // Firstly work on the occupied cells
        cv::Mat map_data_matrix_occ_temp1, map_data_matrix_occ_temp2;

        // Remove isolated occupied cells
        cv::Mat kernel = (cv::Mat_<int>(3, 3) << 1, 1, 1,
                                                 1, -1, 1,
                                                 1, 1, 1);

        // Find isolated cells
        cv::morphologyEx(matrix_OCC,
                         map_data_matrix_occ_temp1,
                         cv::MORPH_HITMISS,
                         kernel,
                         cv::Point(-1, -1),
                         1,
                         cv::BORDER_CONSTANT,
                         cv::morphologyDefaultBorderValue());
        // Overwrite isolated occupied cells with unknown cells
        cv::max(matrix_OCC, map_data_matrix_occ_temp1, matrix_OCC);

        // Erodes unknown zones, thus dilating occupied ones
        cv::erode(matrix_OCC,
                  matrix_OCC,
                  cv::getStructuringElement(cv::MorphShapes::MORPH_RECT,
                                            cv::Size(2, 2),
                                            cv::Point(-1, -1)),
                  cv::Point(-1, -1),
                  1,
                  cv::BORDER_CONSTANT,
                  cv::morphologyDefaultBorderValue());

        cv::erode(matrix_OCC,
                  matrix_OCC,
                  cv::getStructuringElement(cv::MorphShapes::MORPH_RECT,
                                            cv::Size(4, 1),
                                            cv::Point(-1, -1)),
                  cv::Point(-1, -1),
                  1,
                  cv::BORDER_CONSTANT,
                  cv::morphologyDefaultBorderValue());
        cv::erode(matrix_OCC,
                  matrix_OCC,
                  cv::getStructuringElement(cv::MorphShapes::MORPH_RECT,
                                            cv::Size(1, 4),
                                            cv::Point(-1, -1)),
                  cv::Point(-1, -1),
                  2,
                  cv::BORDER_CONSTANT,
                  cv::morphologyDefaultBorderValue());
//        cv::erode(matrix_OCC,
//                  matrix_OCC,
//                  cv::getStructuringElement(cv::MorphShapes::MORPH_RECT,
//                                            cv::Size(5, 1),
//                                            cv::Point(-1, -1)),
//                  cv::Point(-1, -1),
//                  1,
//                  cv::BORDER_CONSTANT,
//                  cv::morphologyDefaultBorderValue());

        cv::dilate(matrix_OCC,
                  matrix_OCC,
                  cv::getStructuringElement(cv::MorphShapes::MORPH_RECT,
                                            cv::Size(3, 3),
                                            cv::Point(-1, -1)),
                  cv::Point(-1, -1),
                  1,
                  cv::BORDER_CONSTANT,
                  cv::morphologyDefaultBorderValue());

    }

    // Secondly work on the free cells
    if (filter_free) {
        // Remove isolated free cells/rays
        cv::morphologyEx(matrix_FREE,
                         matrix_FREE,
                         cv::MORPH_CLOSE,
                         cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE,
                                                   cv::Size(3, 3),
                                                   cv::Point(-1, -1)),
                         cv::Point(-1, -1),
                         1,
                         cv::BORDER_CONSTANT,
                         cv::morphologyDefaultBorderValue());

        cv::morphologyEx(matrix_FREE,
                         matrix_FREE,
                         cv::MORPH_CLOSE,
                         cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE,
                                                   cv::Size(2, 2),
                                                   cv::Point(-1, -1)),
                         cv::Point(-1, -1),
                         1,
                         cv::BORDER_CONSTANT,
                         cv::morphologyDefaultBorderValue());

        // Erodes unknown zones
        cv::erode(matrix_FREE,
                  matrix_FREE,
                  cv::getStructuringElement(cv::MorphShapes::MORPH_RECT,
                                            cv::Size(3, 3),
                                            cv::Point(-1, -1)),
                  cv::Point(-1, -1),
                  1,
                  cv::BORDER_CONSTANT,
                  cv::morphologyDefaultBorderValue());
    }


    if (!filter_free && !filter_occupied){
        return map;
    } else {
        // nav_msgs/OccupancyGrid msg out of map after morphological operations
        nav_msgs::msg::OccupancyGrid map_rect = map;
        for (int j = 0; j < map_rect.info.height; j++) {

            // Pointer to the current row start
            uchar *mat_ptr_occ = matrix_OCC.ptr<uchar>(j);
            uchar *mat_ptr_non_occ = matrix_FREE.ptr<uchar>(j);

            for (int i = 0; i < map_rect.info.width; i++) {
                unsigned k = map_rect.info.width * j + i;
                switch (mat_ptr_non_occ[i]) {
                    case MAT_UNKNOWN_:
                        map_rect.data[k] = -1; // UNKNOWN CELLS
                        break;
                    case MAT_KNOWN_:
                        map_rect.data[k] = 0; // FREE CELLS
                        break;
                }
                // Overwrite occupied cells
                if (mat_ptr_occ[i] == MAT_KNOWN_) {
                    map_rect.data[k] = 100; // OCCUPIED CELLS
                }
            }

        }

        return map_rect;
    }

}