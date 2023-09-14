/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Node.hpp"

#include <iostream>

Node::Node(
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options),
  image_transport_(nullptr),
  map_points_publisher_(nullptr),
  pose_publisher_(nullptr),
  camera_info_sub_(nullptr),
  service_server_(nullptr),
  node_name_(node_name),
  initialized_(false),
  min_observations_per_point_(2),
  /////added//
  was_lost_(false)
{
  declare_parameter("publish_pointcloud", rclcpp::ParameterValue(true));
  declare_parameter("publish_pose", rclcpp::ParameterValue(true));
  declare_parameter("publish_tf", rclcpp::ParameterValue(true));
  declare_parameter("pointcloud_frame_id", rclcpp::ParameterValue(std::string("map")));
  declare_parameter("camera_frame_id", rclcpp::ParameterValue(std::string("camera_link")));
  declare_parameter("map_file", rclcpp::ParameterValue(std::string("map.bin")));
  declare_parameter("voc_file", rclcpp::ParameterValue(std::string("file_not_set")));
  declare_parameter("load_map", rclcpp::ParameterValue(false));

  // ORB SLAM configuration parameters
  declare_parameter("camera_fps", rclcpp::ParameterValue(30));
  declare_parameter("camera_rgb_encoding", rclcpp::ParameterValue(true));
  declare_parameter("ORBextractor/nFeatures", rclcpp::ParameterValue(1200));
  declare_parameter("ORBextractor/scaleFactor", rclcpp::ParameterValue(1.2f));
  declare_parameter("ORBextractor/nLevels", rclcpp::ParameterValue(8));
  declare_parameter("ORBextractor/iniThFAST", rclcpp::ParameterValue(20));
  declare_parameter("ORBextractor/minThFAST", rclcpp::ParameterValue(7));
  declare_parameter("ThDepth", rclcpp::ParameterValue(35.0f));
  declare_parameter("depth_map_factor", rclcpp::ParameterValue(1.0f));
  declare_parameter("camera_baseline", rclcpp::ParameterValue(0.0f));
}

void Node::init(const ORB_SLAM2::System::eSensor & sensor)
{
  get_parameter("publish_pointcloud", publish_pointcloud_param_);
  get_parameter("publish_pose", publish_pose_param_);
  get_parameter("publish_tf", publish_tf_param_);
  get_parameter("pointcloud_frame_id", map_frame_id_param_);
  get_parameter("camera_frame_id", camera_frame_id_param_);
  get_parameter("map_file", map_file_name_param_);
  get_parameter("voc_file", voc_file_name_param_);
  get_parameter("load_map", load_map_param_);

  sensor_ = sensor;

  image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

  rendered_image_publisher_ = image_transport_->advertise(node_name_ + "/debug_image", 1);
  if (publish_pointcloud_param_) {
    Eigen::Affine3d T_rt(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
    cam_base_bia_ = Eigen::AngleAxisd( -23*3.14159265359/180.0, Eigen::Vector3d::UnitY());
    getTfTransformMatrix(T_rt, "depth", "base_link");
    cam_base_R_ = T_rt.rotation();
    cam_base_T_ = T_rt.translation();
    map_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        node_name_ + "/map_points", 1);
  }

  // Enable publishing camera's pose as PoseStamped message
  if (publish_pose_param_) {
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(node_name_ + "/pose", 1);
  }

  // Publishing gba and tracking states
  status_gba_publisher_ = create_publisher<std_msgs::msg::Bool> (node_name_+"/gba_running", 1);
  state_publisher_ = create_publisher<orb_slam2_ros::msg::ORBState>(node_name_+"/info/state", 10);
  state_desc_publisher_  = create_publisher<std_msgs::msg::String>(node_name_+"/info/state_description", 10);
  // Graph publisher
  vertex_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray> (node_name_+"/vertex_list", 1);
  edge_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray> (node_name_+"/edge_list", 1);
  point_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray> (node_name_+"/point_list", 1);
  
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", 1, std::bind(&Node::cameraInfoCallback, this, std::placeholders::_1));

  Node::LoadOrbParameters_manually();

  service_server_ = create_service<orb_slam2_ros::srv::SaveMap>(
      node_name_ + "/save_map",
      std::bind(
          &Node::SaveMapSrv, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

Node::~Node()
{
  // Stop all threads
  orb_slam_->Shutdown();

  // Save camera trajectory
  orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  delete orb_slam_;
}

void Node::Update()
{
  if (!initialized_) {
    RCLCPP_WARN(get_logger(), "Camera info not received, node has not been initialized!");
    return;
  }

  cv::Mat position = orb_slam_->GetCurrentPosition();

  if (!position.empty()) {
    PublishPositionAsTransform(position);
    if (publish_pose_param_) {
      PublishPositionAsPoseStamped(position);
    }
  }

  PublishRenderedImage(orb_slam_->DrawCurrentFrame());
  if (publish_pointcloud_param_) {
    PublishMapPoints(orb_slam_->GetAllMapPoints());
  }

  /////added/////

  int trackingState = orb_slam_->GetTrackingState();
    if (!position.empty() && trackingState == 2) { // OK tracking state
        if (publish_tf_param_){
            PublishPositionAsTransform(position);
        }

        if (publish_pose_param_) {
            PublishPositionAsPoseStamped(position);
        }

        if (was_lost_){
            RCLCPP_INFO(this->get_logger(),"Successful relocalization.");
            was_lost_ = false;
        }

    } else {
        switch (trackingState) {
            case 0: RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),1,"ORB-SLAM failed: No images yet.");
                break;
            case 1: RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),1,"ORB-SLAM failed: Not initialized.");
                break;
            case 3: RCLCPP_ERROR_THROTTLE(this->get_logger(),*this->get_clock(),1,"ORB-SLAM failed: Tracking LOST.");
                break;
        }

        // RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),1,"Broadcasting odometry TF from %s to %s.", target_frame_id_param_.c_str(), map_frame_id_param_.c_str());
        // // Get TF from robot's odometry published topic and send to tf
        // geometry_msgs::msg::TransformStamped msg = tf2::eigenToTransform(odometry_pose_);
        // msg.header.frame_id = map_frame_id_param_;
        // msg.header.stamp = this->get_clock()->now();
        // // msg.header.seq += 1;
        // msg.child_frame_id = target_frame_id_param_;

        // // Broadcast tf
        // static tf2_ros::TransformBroadcaster tf_broadcaster;
        // tf_broadcaster.sendTransform(msg);

        was_lost_ = true;
    }

  PublishGBAStatus (orb_slam_->isRunningGBA());
  PublishState(trackingState);

  auto vertex_getter=orb_slam_->getVertex();
  publishVertices(vertex_getter);

  auto edges_getter=orb_slam_->getEdges();
  publishEdges(edges_getter);

  auto point_getter=orb_slam_->getMapPoints();
  publishPoints(point_getter);
  
}

void Node::PublishMapPoints(std::vector<ORB_SLAM2::MapPoint *> map_points)
{
  sensor_msgs::msg::PointCloud2 cloud = MapPointsToPointCloud(map_points);
  map_points_publisher_->publish(cloud);
}

void Node::PublishPositionAsTransform(cv::Mat position)
{
  if (publish_tf_param_) {
    tf2::Transform transform = TransformFromMat(position);
    geometry_msgs::msg::TransformStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = map_frame_id_param_;
    tmp_tf_stamped.header.stamp = current_frame_time_;
    tmp_tf_stamped.child_frame_id = camera_frame_id_param_;
    tf2::impl::Converter<false, true>::convert(transform, tmp_tf_stamped.transform);
    tf_broadcaster_->sendTransform(tmp_tf_stamped);
  }
}

// void Node::PublishPositionAsTransform (cv::Mat position) {
//     // Get transform from map to camera frame
//     tf2::Transform tf_transform = TransformFromMat(position);

//     // Make transform from camera frame to target frame
//     tf2::Transform tf_map2target = TransformToTarget(tf_transform, camera_frame_id_param_, "base_link");

//     // Make message
//     tf2::Stamped<tf2::Transform> tf_map2target_stamped;
//     tf_map2target_stamped = tf2::Stamped<tf2::Transform>(tf_map2target, current_frame_time_, map_frame_id_param_);
//     geometry_msgs::msg::TransformStamped msg = tf2::toMsg(tf_map2target_stamped);
//     msg.child_frame_id = "odom";
//     // Broadcast tf
//     tf_broadcaster_->sendTransform(msg);
// }

void Node::PublishPositionAsPoseStamped(cv::Mat position)
{
  tf2::Transform grasp_tf = TransformFromMat(position);
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = current_frame_time_;
  pose_msg.header.frame_id = map_frame_id_param_;
  tf2::toMsg(grasp_tf, pose_msg.pose);
  pose_publisher_->publish(pose_msg);
}


void Node::PublishRenderedImage(cv::Mat image)
{
  std_msgs::msg::Header header;
  header.stamp = current_frame_time_;
  header.frame_id = map_frame_id_param_;
  const sensor_msgs::msg::Image::SharedPtr rendered_image_msg =
    cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  rendered_image_publisher_.publish(rendered_image_msg);
}


tf2::Transform Node::TransformFromMat(cv::Mat position_mat)
{
  cv::Mat rotation(3, 3, CV_32F);
  cv::Mat translation(3, 1, CV_32F);

  rotation = position_mat.rowRange(0, 3).colRange(0, 3);
  translation = position_mat.rowRange(0, 3).col(3);


  tf2::Matrix3x3 tf_camera_rotation(
    rotation.at<float>(0, 0), rotation.at<float>(0, 1), rotation.at<float>(0, 2),
    rotation.at<float>(1, 0), rotation.at<float>(1, 1), rotation.at<float>(1, 2),
    rotation.at<float>(2, 0), rotation.at<float>(2, 1), rotation.at<float>(2, 2));

  tf2::Vector3 tf_camera_translation(
    translation.at<float>(0), translation.at<float>(1), translation.at<float>(2));

  // Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf2::Matrix3x3 tf_orb_to_ros(
    0, 0, 1,
    -1, 0, 0,
    0, -1, 0);

  // Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

  // Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

  // Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

  return tf2::Transform(tf_camera_rotation, tf_camera_translation);
}


sensor_msgs::msg::PointCloud2 Node::MapPointsToPointCloud(
  std::vector<ORB_SLAM2::MapPoint *> map_points)
{
  if (map_points.empty()) {
    RCLCPP_WARN(get_logger(), "Map point vector is empty!");
  }

  sensor_msgs::msg::PointCloud2 cloud;

  const int num_channels = 3;  // x y z

  cloud.header.stamp = current_frame_time_;
  cloud.header.frame_id = map_frame_id_param_;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = {"x", "y", "z"};
  for (int i = 0; i < num_channels; i++) {
    cloud.fields[i].name = channel_id[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count = 1;
    cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

  unsigned char * cloud_data_ptr = &(cloud.data[0]);

  float * data_array = new float[num_channels];
  for (unsigned int i = 0; i < cloud.width; i++) {
    if (map_points.at(i)->nObs >= min_observations_per_point_) {
      // x. Do the transformation by just reading at the position of z instead of x
      data_array[0] = map_points.at(i)->GetWorldPos().at<float>(2);
      // y. Do the transformation by just reading at the position of x instead of y
      data_array[1] = -1.0 * map_points.at(i)->GetWorldPos().at<float>(0);
      // z. Do the transformation by just reading at the position of y instead of z
      data_array[2] = -1.0 * map_points.at(i)->GetWorldPos().at<float>(1);

      // Eigen::Vector3f map_pt_f(map_points.at(i)->GetWorldPos().at<float>(0),
      //                           map_points.at(i)->GetWorldPos().at<float>(1),
      //                           map_points.at(i)->GetWorldPos().at<float>(2));

      //   Eigen::Vector3d map_pt = map_pt_f.cast<double>();

      //   map_pt = cam_base_R_ * map_pt;
      //   map_pt = cam_base_bia_ * map_pt;
      //   map_pt += cam_base_T_;

      //   data_array[0] = map_pt[0];
      //   data_array[1] = map_pt[1];
      //   data_array[2] = map_pt[2];

      // TODO(tbd): dont hack the transformation but have a central conversion
      //  function for MapPointsToPointCloud and TransformFromMat

      memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
    }
  }

  return cloud;
}

void Node::SaveMapSrv(
  const shared_ptr<rmw_request_id_t>,
  const shared_ptr<orb_slam2_ros::srv::SaveMap::Request> request,
  const shared_ptr<orb_slam2_ros::srv::SaveMap::Response> response)
{
  response->success = orb_slam_->SaveMap(request->name);

  if (response->success) {
    RCLCPP_INFO(get_logger(), "Map was saved as %s", request->name.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "Map could not be saved.");
  }
}

void Node::cameraInfoCallback(sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!initialized_) {
    RCLCPP_INFO(get_logger(), "Camera info received.");
    LoadOrbParameters(msg);
  }
}

void Node::LoadOrbParameters(sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
{
  // Create a parameters object to pass to the Tracking system
  ORB_SLAM2::ORBParameters parameters{};

  get_parameter("camera_fps", parameters.maxFrames);
  get_parameter("camera_rgb_encoding", parameters.RGB);
  get_parameter("ORBextractor/nFeatures", parameters.nFeatures);
  get_parameter("ORBextractor/scaleFactor", parameters.scaleFactor);
  get_parameter("ORBextractor/nLevels", parameters.nLevels);
  get_parameter("ORBextractor/iniThFAST", parameters.iniThFAST);
  get_parameter("ORBextractor/minThFAST", parameters.minThFAST);

  if (sensor_ == ORB_SLAM2::System::STEREO || sensor_ == ORB_SLAM2::System::RGBD) {
    get_parameter("ThDepth", parameters.thDepth);
    get_parameter("depth_map_factor", parameters.depthMapFactor);
  }

  parameters.fx = camera_info->k[0];
  parameters.fy = camera_info->k[4];
  parameters.cx = camera_info->k[2];
  parameters.cy = camera_info->k[5];
  parameters.k1 = camera_info->d[0];
  parameters.k2 = camera_info->d[1];
  parameters.p1 = camera_info->d[2];
  parameters.p2 = camera_info->d[3];
  parameters.k3 = camera_info->d[4];

  if (sensor_ == ORB_SLAM2::System::STEREO || sensor_ == ORB_SLAM2::System::RGBD) {
    if (!get_parameter("camera_baseline", parameters.baseline)) {
      parameters.baseline = camera_info->p[3];
    }
  }

  orb_slam_ = new ORB_SLAM2::System(
      voc_file_name_param_,
      sensor_,
      parameters,
      map_file_name_param_,
      load_map_param_);

  initialized_ = true;
}

void Node::LoadOrbParameters_manually()
{
  // Create a parameters object to pass to the Tracking system
  ORB_SLAM2::ORBParameters parameters{};

  get_parameter("camera_fps", parameters.maxFrames);
  get_parameter("camera_rgb_encoding", parameters.RGB);
  get_parameter("ORBextractor/nFeatures", parameters.nFeatures);
  get_parameter("ORBextractor/scaleFactor", parameters.scaleFactor);
  get_parameter("ORBextractor/nLevels", parameters.nLevels);
  get_parameter("ORBextractor/iniThFAST", parameters.iniThFAST);
  get_parameter("ORBextractor/minThFAST", parameters.minThFAST);

  if (sensor_ == ORB_SLAM2::System::STEREO || sensor_ == ORB_SLAM2::System::RGBD) {
    get_parameter("ThDepth", parameters.thDepth);
    get_parameter("depth_map_factor", parameters.depthMapFactor);
  }

  parameters.fx = 535.4;
  parameters.fy = 539.2;
  parameters.cx = 320.1;
  parameters.cy = 247.6;
  parameters.k1 = 0.0;
  parameters.k2 = 0.0;
  parameters.p1 = 0.0;
  parameters.p2 = 0.0;
  parameters.k3 = 0.0;

  if (sensor_ == ORB_SLAM2::System::STEREO || sensor_ == ORB_SLAM2::System::RGBD) {
    if (!get_parameter("camera_baseline", parameters.baseline)) {
      parameters.baseline = 9.052;
    }
  }

  orb_slam_ = new ORB_SLAM2::System(
      voc_file_name_param_,
      sensor_,
      parameters,
      map_file_name_param_,
      load_map_param_);

  initialized_ = true;
}

//////////////////////////ADDED/////////////////////////////

void Node::PublishGBAStatus (bool gba_status) {
    if (status_gba_publisher_->get_subscription_count() > 0) {
        std_msgs::msg::Bool gba_status_msg;
        gba_status_msg.data = gba_status;
        status_gba_publisher_->publish(gba_status_msg);
    }
}

void Node::PublishState(int trackingState) {
     // save state from tracking, even if there are no subscribers
     orb_slam2_ros::msg::ORBState orb_state_ ;
     orb_state_ = toORBStateMessage(trackingState);

    if (state_publisher_->get_subscription_count() > 0) {
        // publish state as ORBState int
        orb_state_.header.stamp = this->get_clock()->now();;
        state_publisher_->publish(orb_state_);
    }

    if (state_desc_publisher_->get_subscription_count() > 0) {
        // publish state as string
        std_msgs::msg::String state_desc_msg;
        state_desc_msg.data = stateDescription(orb_state_);
        state_desc_publisher_->publish(state_desc_msg);
    }

}

const orb_slam2_ros::msg::ORBState Node::toORBStateMessage(int trackingState)
{
    orb_slam2_ros::msg::ORBState state_msg;
    state_msg.state = orb_slam2_ros::msg::ORBState::UNKNOWN;

    switch (trackingState) {
        case 0: state_msg.state = orb_slam2_ros::msg::ORBState::NO_IMAGES_YET;
            break;
        case 1: state_msg.state = orb_slam2_ros::msg::ORBState::SYSTEM_NOT_READY;
            break;
        case 2: state_msg.state = orb_slam2_ros::msg::ORBState::OK;
            break;
        case 3: state_msg.state = orb_slam2_ros::msg::ORBState::LOST;
            break;
    }

    return state_msg;
}

const char* Node::stateDescription(orb_slam2_ros::msg::ORBState orb_state)
{
    switch (orb_state.state) {
        case orb_slam2_ros::msg::ORBState::SYSTEM_NOT_READY: return "System not ready";
        case orb_slam2_ros::msg::ORBState::NO_IMAGES_YET: return "No images yet";
        case orb_slam2_ros::msg::ORBState::NOT_INITIALIZED: return "Not initialized";
        case orb_slam2_ros::msg::ORBState::OK: return "OK";
        case orb_slam2_ros::msg::ORBState::LOST: return "Tracking lost";
    }

    return "???";
}

void Node::publishVertices(std::list<float>& l){
    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg.layout.data_offset = 0;
    msg.layout.dim[0].size = l.size();
    msg.layout.dim[0].stride = 8; // ID1, quaternion, position

    for(float f : l){
        msg.data.push_back(f);
    }

    vertex_publisher_->publish(msg);
}

void Node::publishEdges(std::list<float>& l){
    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg.layout.data_offset = 0;
    msg.layout.dim[0].size = l.size();
    msg.layout.dim[0].stride = 38; // ID1, ID2, Hessian (6x6)

    for(float f : l){
        msg.data.push_back(f);
    }

    edge_publisher_->publish(msg);
}


void Node::publishPoints(std::list<float>& l){
    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg.layout.data_offset = 0;
    msg.layout.dim[0].size = l.size();

    // Array of unknown size since the number of observation varies

    for(float f : l){
        msg.data.push_back(f);
    }

    point_publisher_->publish(msg);
}

bool Node::getTfTransformMatrix(Eigen::Affine3d &transform_matrix, const std::string source_frame, const std::string target_frame) {
    try{
        geometry_msgs::msg::TransformStamped transform_to_robot = tf_buffer_->lookupTransform(target_frame, source_frame,rclcpp::Time(0),rclcpp::Duration::from_seconds(0.05));

        transform_matrix = tf2::transformToEigen(transform_to_robot);
        return true;
    }
    catch (tf2::TransformException &ex){
        RCLCPP_WARN(this->get_logger(),"%s",ex.what());
        return false;
    }
}

tf2::Transform Node::TransformToTarget (tf2::Transform tf_in, std::string frame_in, std::string frame_target) {
    // Transform tf_in from frame_in to frame_target
    tf2::Transform tf_map2orig = tf_in;
    tf2::Transform tf_orig2target;
    tf2::Transform tf_map2target;

    tf2::Stamped<tf2::Transform> transformStamped_temp;
    try {
        // Get the transform from camera to target
        geometry_msgs::msg::TransformStamped tf_msg = tf_buffer_->lookupTransform(frame_in, frame_target,rclcpp::Time(0));
        // Convert to tf2
        tf2::fromMsg(tf_msg, transformStamped_temp);
        tf_orig2target.setBasis(transformStamped_temp.getBasis());
        tf_orig2target.setOrigin(transformStamped_temp.getOrigin());

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(),"%s",ex.what());
        //ros::Duration(1.0).sleep();
        tf_orig2target.setIdentity();
    }

    // Transform from map to target
    tf_map2target = tf_map2orig * tf_orig2target;

    /*
      // Print debug info
      double roll, pitch, yaw;
      // Print debug map2orig
      tf2::Matrix3x3(tf_map2orig.getRotation()).getRPY(roll, pitch, yaw);
      ROS_INFO("Static transform Map to Orig [%s -> %s]",
                      map_frame_id_param_.c_str(), frame_in.c_str());
      ROS_INFO(" * Translation: {%.3f,%.3f,%.3f}",
                      tf_map2orig.getOrigin().x(), tf_map2orig.getOrigin().y(), tf_map2orig.getOrigin().z());
      ROS_INFO(" * Rotation: {%.3f,%.3f,%.3f}",
                      RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));
      // Print debug tf_orig2target
      tf2::Matrix3x3(tf_orig2target.getRotation()).getRPY(roll, pitch, yaw);
      ROS_INFO("Static transform Orig to Target [%s -> %s]",
                      frame_in.c_str(), frame_target.c_str());
      ROS_INFO(" * Translation: {%.3f,%.3f,%.3f}",
                      tf_orig2target.getOrigin().x(), tf_orig2target.getOrigin().y(), tf_orig2target.getOrigin().z());
      ROS_INFO(" * Rotation: {%.3f,%.3f,%.3f}",
                      RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));
      // Print debug map2target
      tf2::Matrix3x3(tf_map2target.getRotation()).getRPY(roll, pitch, yaw);
      ROS_INFO("Static transform Map to Target [%s -> %s]",
                      map_frame_id_param_.c_str(), frame_target.c_str());
      ROS_INFO(" * Translation: {%.3f,%.3f,%.3f}",
                      tf_map2target.getOrigin().x(), tf_map2target.getOrigin().y(), tf_map2target.getOrigin().z());
      ROS_INFO(" * Rotation: {%.3f,%.3f,%.3f}",
                      RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));

     */
    return tf_map2target;
}

