#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <light_painting_demo/parse_image_waypoints.h>

namespace light_painting_demo
{

namespace
{
  static const std::string WAYPOINT_FRAME_ID = "base_link";
  static constexpr double PIXEL_TO_WORLD_SCALE = 0.001;  // unitless, converts pixels to meters
}

bool ParseImageWaypoints::loadWaypointsFromFile(const std::string& yaml_file)
{
  YAML::Node config = YAML::LoadFile(yaml_file);

  // Fill the pixel waypoints vector
  const YAML::Node &waypoints = config["waypoints"];
  std::pair<int, int> pixel_coordinate; 
  for (size_t waypoint_idx = 0; waypoint_idx < waypoints.size(); ++waypoint_idx)
  {
    pixel_coordinate.first = waypoints[waypoint_idx]["x"].as<int>();
    pixel_coordinate.second = waypoints[waypoint_idx]["y"].as<int>();
    pixel_waypoints_x_y_.push_back(pixel_coordinate);
  }
  std::cout << "Number of waypoints parsed: " << waypoints.size() << std::endl;
  std::cout << "First pixel x,y: " << pixel_waypoints_x_y_.at(0).first << "  " << pixel_waypoints_x_y_.at(0).second << std::endl;

  return true;
}

bool ParseImageWaypoints::transformPixelCoordinatesToRobotPose()
{
  // (0,0) is the upper-left pixel coordinate in the image
  // (500,500) is the center pixel of the image

  // (0.5, 0, 0.5) meters is roughly the center of the robot workspace

  // So, to transform from pixel coordinates to robot coordinates, do...
  // 1. scale by 0.001
  // 2. rotate 180* about X

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = WAYPOINT_FRAME_ID;

  Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());

  for (size_t waypoint_idx = 0; waypoint_idx < pixel_waypoints_x_y_.size(); ++waypoint_idx)
  {
    transform = Eigen::Isometry3d::Identity();
  }

  return true;
}
}  // namespace light_painting_demo
