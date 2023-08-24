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

#ifndef NODE_HPP_
#define NODE_HPP_

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <opencv2/core/core.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <vector>
#include <string>
#include <memory>

#include "System.h"

#include "tf2/convert.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2/utils.h"
#include <tf2_eigen/tf2_eigen.h>

#include "orb_slam2_ros/srv/save_map.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "orb_slam2_ros/msg/orb_state.hpp"

class Node : public rclcpp::Node
{
public:
  Node(
    const std::string & node_name,
    const rclcpp::NodeOptions & node_options);

  void init(const ORB_SLAM2::System::eSensor & sensor);

  ~Node();

protected:
  void Update();
  bool isInitialized()
  {
    return initialized_;
  }

  ORB_SLAM2::System * orb_slam_;
  rclcpp::Time current_frame_time_;
  std::shared_ptr<image_transport::ImageTransport> image_transport_;

private:
  void PublishMapPoints(std::vector<ORB_SLAM2::MapPoint *> map_points);
  void PublishPositionAsTransform(cv::Mat position);
  void PublishPositionAsPoseStamped(cv::Mat position);
  void PublishRenderedImage(cv::Mat image);
  void SaveMapSrv(
    const shared_ptr<rmw_request_id_t>/*request_header*/,
    const shared_ptr<orb_slam2_ros::srv::SaveMap::Request> request,
    const shared_ptr<orb_slam2_ros::srv::SaveMap::Response> response);
  void LoadOrbParameters(sensor_msgs::msg::CameraInfo::SharedPtr camera_info);
  void cameraInfoCallback(sensor_msgs::msg::CameraInfo::SharedPtr msg);

  tf2::Transform TransformFromMat(cv::Mat position_mat);
  sensor_msgs::msg::PointCloud2 MapPointsToPointCloud(
    std::vector<ORB_SLAM2::MapPoint *> map_points);

  ORB_SLAM2::System::eSensor sensor_;

  image_transport::Publisher rendered_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  // Publishing gba and tracking states
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_gba_publisher_;
  rclcpp::Publisher<orb_slam2_ros::msg::ORBState>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_desc_publisher_;
  // Graph publisher
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vertex_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr edge_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr point_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Service<orb_slam2_ros::srv::SaveMap>::SharedPtr service_server_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string node_name_;

  bool initialized_;

  std::string map_frame_id_param_;
  std::string camera_frame_id_param_;
  std::string map_file_name_param_;
  std::string voc_file_name_param_;
  bool load_map_param_;
  bool publish_pointcloud_param_;
  bool publish_tf_param_;
  bool publish_pose_param_;
  int min_observations_per_point_;

  //////////added//////////////
  bool was_lost_;

  void PublishGBAStatus (bool gba_status);
  void PublishState(int trackingState);
  const orb_slam2_ros::msg::ORBState toORBStateMessage(int trackingState);
  const char* stateDescription(orb_slam2_ros::msg::ORBState orb_state);

  void publishVertices(std::list<float>& l);
  void publishEdges(std::list<float>& l);
  void publishPoints(std::list<float>& l);
};

#endif  // NODE_HPP_
