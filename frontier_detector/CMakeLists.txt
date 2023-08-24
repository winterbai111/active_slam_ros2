cmake_minimum_required(VERSION 3.5)
project(frontier_detector)

# Add support for C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

set(CMAKE_CXX_STANDARD 20) #20

# include(CheckCXXCompilerFlag)
# check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
# if(COMPILER_SUPPORTS_CXX11)
#   add_compile_options(-std=c++11)
# else()
#   message(ERROR
#     "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()

# add_compile_options(-std=c++14)

add_compile_options(-std=c++20 -g)
# add_compile_options(-std=c++11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)



find_package(OpenCV 4.0 QUIET)
include_directories ("/usr/include/opencv4/")
if(NOT OpenCV_FOUND)
   find_package(OpenCV 3.0 QUIET)
   if(NOT OpenCV_FOUND)
      find_package(OpenCV 2.4.3 QUIET)
      if(NOT OpenCV_FOUND)
         message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
     endif()
   endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(orb_slam2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
# find_package(Boost COMPONENTS program_options signals thread system)
# find_package(Threads)

include_directories(
  include
  # ${colcon_INCLUDE_DIRS}
  /opt/ros/humble/include
  /opt/ros/humble/include/sensor_msgs
  /opt/ros/humble/include/nav_msgs
  )

  #add_service_files(
#		DIRECTORY srv
#		FILES GetOctomap.srv
#)


# generate_dynamic_reconfigure_options(cfg/informationGain.cfg)


# Frontier detectors
add_executable(${PROJECT_NAME}_global_rrt src/GlobalRRTDetector.cpp
  src/Functions.cpp src/Mtrand.cpp)


add_executable(${PROJECT_NAME}_local_rrt src/LocalRRTDetector.cpp
  src/Functions.cpp src/Mtrand.cpp)

ament_target_dependencies(${PROJECT_NAME}_global_rrt
  nav_msgs
  octomap_msgs
  visualization_msgs
  rclcpp
  tf2_ros
  tf2
)
ament_target_dependencies(${PROJECT_NAME}_local_rrt
  nav_msgs
  octomap_msgs
  visualization_msgs
  rclcpp
  tf2_ros
  tf2
)

# Octomapper
add_executable(${PROJECT_NAME}_octomapper src/OctoMapper.cpp)

ament_target_dependencies(${PROJECT_NAME}_octomapper
  nav_msgs
  octomap_msgs
  tf2_ros
  octomap_ros
  orb_slam2_ros
)

target_link_libraries(${PROJECT_NAME}_octomapper 
  ${Boost_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
)

# Gridmapper
add_executable(${PROJECT_NAME}_gridmapper src/GridMapper.cpp)

ament_target_dependencies(${PROJECT_NAME}_gridmapper
  nav_msgs
  octomap_msgs
  tf2_ros
  octomap_ros
  tf2_geometry_msgs
  cv_bridge
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_global_rrt
  ${PROJECT_NAME}_local_rrt
  ${PROJECT_NAME}_octomapper
  ${PROJECT_NAME}_gridmapper
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/PointArray.msg"
#   DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
# )

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  src/python/Filter.py
  src/python/Functions.py
  src/python/Initializer.py
  src/python/OpenCV_detector.py
  src/python/SimpleRobot.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
