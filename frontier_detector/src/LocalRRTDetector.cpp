/*  jplaced
    2022, Universidad de Zaragoza
    Modified from original work: http://wiki.ros.org/rrt_exploration
    Citation:
    Umari, H., & Mukhopadhyay, S. (2017, September).
    Autonomous robotic exploration based on multiple rapidly-exploring randomized trees.
    In 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) (pp. 1396-1402). IEEE.
*/

// This node is similar to the global_rrt_frontier_detector. However, it works
// differently, as the tree here keeps resetting every time a frontier point is
// detected. This node is intended to be run along side the
// global_rrt_frontier_detector node, it is responsible for fast detection of
// frontier points that lie in the close vicinity of the robot.
// In multi-robot configuration, each robot runs an instance of the
// local_rrt_frontier_detector. So for a team of 3 robots, there will be 4 nodes
// for detecting frontier points; 3 local detectors and 1 global detector.
// Running additional instances of the local frontier detector can enhance the
// speed of frontier points detection, if needed. All detectors will be
// publishing detected frontier points on the same topic ("/detected_points").
// Paper: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8202319

// #include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
#include "stdint.h"

#include "Functions.h"
#include "Mtrand.h"

#include <sstream>
#include <iostream>
#include <string>
#include <vector>

#include <tf2_ros/transform_listener.h>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
// #include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2_ros/buffer.h"
#include <chrono>
#include <thread>

// #include "visualization_msgs/Marker.h"

// global variables
nav_msgs::msg::OccupancyGrid mapData;
geometry_msgs::msg::PointStamped exploration_goal;
visualization_msgs::msg::Marker points,line;
float xdim,ydim,resolution,Xstartx,Xstarty,init_map_x,init_map_y;

rdm r; // for generating random numbers

//Subscribers callback functions---------------------------------------
void mapCallBack(const nav_msgs::msg::OccupancyGrid::ConstPtr& msg){
  mapData=*msg;
}

void initCallBack(const visualization_msgs::msg::Marker::ConstPtr& msg){
  points.points = msg->points;
}

int main(int argc, char **argv){

  unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
  MTRand_int32 irand(init, length); // 32-bit int generator
  // this is an example of initializing by an array
  // you may use MTRand(seed) with any 32bit integer
  // as a seed for a simpler initialization
  MTRand drand; // double in [0, 1) generator, already init

  // generate the same numbers as in the original C test program
  // ros::init(argc, argv, "local_rrt_frontier_detector");
  // ros::NodeHandle nh;

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("local_rrt_frontier_detector_node");

  // fetching all parameters
  float eta,init_map_x,init_map_y,range;
  std::string map_topic,base_frame_topic, map_frame_id;

  std::string ns;
  // ns=ros::this_node::getName();
  ns = node->get_name();

  // ros::param::param<float>(ns+"/eta", eta, 0.5);
  // ros::param::param<std::string>(ns+"/map_topic", map_topic, "/map");
  // ros::param::param<std::string>(ns+"/map_frame", map_frame_id, "/map");
  // ros::param::param<std::string>(ns+"/robot_frame", base_frame_topic, "/base_link");

  // node->declare_parameter<float>(ns+"/eta", 0.5);
  // node->declare_parameter<std::string>(ns+"/map_topic", "/map");
  // node->declare_parameter<std::string>(ns+"/map_frame", "/map");
  // node->declare_parameter<std::string>(ns+"/robot_frame", "/base_link");

  node->declare_parameter(ns+"/eta",rclcpp::ParameterValue(float(0.5)));
  node->declare_parameter(ns+"/map_topic", rclcpp::ParameterValue(std::string("/map")));
  node->declare_parameter(ns+"/map_frame",rclcpp::ParameterValue(std::string("map")));
  node->declare_parameter(ns+"/robot_frame", rclcpp::ParameterValue(std::string("base_link")));

  node->get_parameter(ns+"/eta", eta);
  node->get_parameter(ns+"/map_topic", map_topic);
  node->get_parameter(ns+"/map_frame", map_frame_id);
  node->get_parameter(ns+"/robot_frame", base_frame_topic);

  // ros::Subscriber sub       = nh.subscribe(map_topic, 100 ,mapCallBack);
  // ros::Subscriber rviz_sub  = nh.subscribe("init_points", 100 ,initCallBack);

  // auto sub= node->create_subscription<nav_msgs::msg::OccupancyGrid>("GridMapper_node/rectified_map", 100 ,mapCallBack);
  auto sub= node->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 100 ,mapCallBack);
  auto rviz_sub= node->create_subscription<visualization_msgs::msg::Marker>("init_points", 100 ,initCallBack);
  
  auto targetspub = node->create_publisher<geometry_msgs::msg::PointStamped>("detected_points", 10);
  auto pub        = node->create_publisher<visualization_msgs::msg::Marker>(ns+"_shapes", 10);

  double param_1;
  // nh.getParam(ns+"/rate", param_1);
  node->get_parameter(ns+"/rate", param_1);
  // ros::Rate rate(param_1);
  rclcpp::Rate rate(param_1);

  // wait until map is received, when a map is received, mapData.header.seq will not be < 1
  // while (mapData.header.seq<1 or mapData.data.size()<1)  {  rclcpp::spin_some(node);  ros::Duration(0.1).sleep();}
  // while (mapData.data.size()<1)  {  rclcpp::spin_some(node);  ros::Duration(0.1).sleep();}
  while (mapData.data.size()<1)  {  rclcpp::spin_some(node);  std::this_thread::sleep_for(std::chrono::seconds(1));}
  RCLCPP_INFO(node->get_logger()," Local: map received");
  //visualizations  points and lines..
  points.header.frame_id = mapData.header.frame_id;
  line.header.frame_id   = mapData.header.frame_id;
  points.header.stamp    = rclcpp::Time(0);
  line.header.stamp      = rclcpp::Time(0);
  points.ns   = line.ns  = "markers";
  points.id   = 0;
  line.id     = 1;
  points.type = points.POINTS;
  line.type   = line.LINE_LIST;

  //Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  points.pose.orientation.w = 1.0;
  line.pose.orientation.w   = 1.0;
  points.action   = points.ADD;
  line.action     = line.ADD;
  line.scale.x    = 0.03;
  // line.scale.y    = 0.03;
  points.scale.x  = 0.3;
  points.scale.y  = 0.3;
  line.color.r    = 255.0/255.0;
  line.color.g    = 0.0/255.0;
  line.color.b    = 0.0/255.0;
  points.color.r  = 255.0/255.0;
  points.color.g  = 255.0/255.0;
  points.color.b  = 0.0/255.0;
  points.color.a  = 1.0;
  line.color.a    = 1.0;
  // points.lifetime = ros::Duration();
  // line.lifetime   = ros::Duration();
  points.lifetime = rclcpp::Duration::from_seconds(0);
  line.lifetime   = rclcpp::Duration::from_seconds(0);

  geometry_msgs::msg::Point p;

  while(points.points.size()<5){
    rclcpp::spin_some(node);
    pub->publish(points) ;
  }

  std::vector<float> temp1;
  temp1.push_back(points.points[0].x);
  temp1.push_back(points.points[0].y);

  std::vector<float> temp2;
  temp2.push_back(points.points[2].x);
  temp2.push_back(points.points[0].y);

  init_map_x=Norm(temp1,temp2);
  temp1.clear();
  temp2.clear();

  temp1.push_back(points.points[0].x);
  temp1.push_back(points.points[0].y);

  temp2.push_back(points.points[0].x);
  temp2.push_back(points.points[2].y);

  init_map_y = Norm(temp1,temp2);
  temp1.clear();
  temp2.clear();

  Xstartx = (points.points[0].x + points.points[2].x) * .5;
  Xstarty = (points.points[0].y + points.points[2].y) * .5;

  geometry_msgs::msg::Point trans;
  trans = points.points[4];
  std::vector< std::vector<float> > V;
  std::vector<float> xnew;
  xnew.push_back(trans.x);
  xnew.push_back(trans.y);
  V.push_back(xnew);

  points.points.clear();
  pub->publish(points);

  std::vector<float> frontiers;
  int i=0;
  float xr,yr;
  std::vector<float> x_rand,x_nearest,x_new;

  // tf2_ros::TransformListener *listener;
  auto tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto listener =std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  visualization_msgs::msg::Marker points;
  points.header.frame_id = mapData.header.frame_id;
  points.header.stamp    = rclcpp::Time(0);
  points.ns   = line.ns  = "markers";
  points.id   = 0;
  points.type = points.POINTS;
  points.pose.orientation.w = 1.0;
  points.action   = points.ADD;
  points.scale.x  = 0.3;
  points.scale.y  = 0.3;
  points.color.r  = 255.0/255.0;
  points.color.g  = 0.0/255.0;
  points.color.b  = 0.0/255.0;
  points.color.a  = 1.0;
  points.lifetime = rclcpp::Duration::from_seconds(0);

  //TODO stop working when black pixels in map
  // Main loop
  while (rclcpp::ok()){
    // Sample free inside the map!
    bool outside_map = true;
    while (outside_map == true){
      x_rand.clear();
      xr = (drand()*init_map_x) - (init_map_x*0.5) + Xstartx;
      yr = (drand()*init_map_y) - (init_map_y*0.5) + Xstarty;
      if (xr<mapData.info.origin.position.x || xr>(mapData.info.origin.position.x+mapData.info.width * mapData.info.resolution) || \
          yr<mapData.info.origin.position.y || yr>(mapData.info.origin.position.y+mapData.info.height * mapData.info.resolution)){
        outside_map = true;
      } else{
        outside_map = false;
      }
    }
    x_rand.push_back(xr);
    x_rand.push_back(yr);
    // Nearest
    x_nearest = Nearest(V,x_rand);
    // Steer
    x_new = Steer(x_nearest,x_rand,eta);
    if (x_new[0]>mapData.info.origin.position.x && x_new[0]<(mapData.info.origin.position.x+mapData.info.width * mapData.info.resolution) && \
        x_new[1]>mapData.info.origin.position.y && x_new[1]<(mapData.info.origin.position.y+mapData.info.height * mapData.info.resolution)){      
      // ObstacleFree    1:free     -1:unkown (frontier region)      0:obstacle
      char checking = ObstacleFree(x_nearest,x_new,mapData);

  	  if (checking == -1){
  	    exploration_goal.header.stamp=node->get_clock()->now();
      	exploration_goal.header.frame_id=mapData.header.frame_id;
      	exploration_goal.point.x = x_new[0];
      	exploration_goal.point.y = x_new[1];
      	exploration_goal.point.z = 0.0;
      	p.x = x_new[0];
        p.y = x_new[1];
        p.z = 0.0;

      	points.points.push_back(p);
      	pub->publish(points) ;
      	targetspub->publish(exploration_goal);
  	  	points.points.clear();
  	  	V.clear();

  			// tf2::StampedTransform transform;
        geometry_msgs::msg::TransformStamped transform;
  			int  temp=0;
  			while (temp == 0){
    			try{
      			temp = 1;
      			transform = tf_buffer_->lookupTransform(map_frame_id, base_frame_topic , rclcpp::Time(0));
    			}
    			catch (tf2::TransformException ex){
      			temp = 0;
      			// ros::Duration(0.1).sleep();
            std::this_thread::sleep_for(std::chrono::seconds(1));
  		    }
        }
        
  			// x_new[0] = transform.getOrigin().x();
  			// x_new[1] = transform.getOrigin().y();
        x_new[0] = transform.transform.translation.x;
  			x_new[1] = transform.transform.translation.y;

      	V.push_back(x_new);
      	line.points.clear();
        // Avoid "Points should not be empty for specified marker type." error message by adding a line between 0,0,0 and 0,0,0
        p.x = 0.0;
        p.y = 0.0;
        p.z = 0.0;
        line.points.push_back(p);
        p.x = 0.0;
        p.y = 0.0;
        p.z = 0.0;
        line.points.push_back(p);
      }
      else if (checking == 1){
        V.push_back(x_new);
        p.x = x_new[0];
        p.y = x_new[1];
        p.z = 0.0;
        line.points.push_back(p);
        p.x = x_nearest[0];
        p.y = x_nearest[1];
        p.z = 0.0;
        line.points.push_back(p);
      }

      pub->publish(line);
    }

    rclcpp::spin_some(node);
    // rate.sleep();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  } // endof ros loop

  return 0;
} //endof main
