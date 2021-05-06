#include <Eigen/Core>
#include <Eigen/Geometry>
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

std::vector<geometry_msgs::msg::PoseStamped> ParseImageWaypoints::transformPixelCoordinatesToRobotPose()
{
  // (0,0) is the upper-left pixel coordinate in the image
  // (500,500) is the center pixel of the image

  // (0.5, 0, 0.5) meters is roughly the center of the robot workspace

  // So, to transform from pixel coordinates to robot coordinates, do...
  // 1. scale by 0.001
  // 2. translate by (-0.5, -0.5)
  // 3. do the inverse of (rotate (90* about X) then (90* about Z))

  std::vector<geometry_msgs::msg::PoseStamped> target_poses;
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = WAYPOINT_FRAME_ID;

  Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());

  Eigen::Quaterniond rot_90_about_x(0.7071, 0.7071, 0, 0);
  Eigen::Quaterniond rot_90_about_z(0.7071, 0, 0, 0.7071);
  Eigen::Quaterniond net_rotation_from_pixel_to_world = (rot_90_about_x * rot_90_about_z).inverse();

  for (size_t waypoint_idx = 0; waypoint_idx < pixel_waypoints_x_y_.size(); ++waypoint_idx)
  {
    transform = Eigen::Isometry3d::Identity();
    transform.translation().x() = pixel_waypoints_x_y_[waypoint_idx].first * PIXEL_TO_WORLD_SCALE - 0.5;
    transform.translation().y() = pixel_waypoints_x_y_[waypoint_idx].second * PIXEL_TO_WORLD_SCALE - 1.0;

    transform = net_rotation_from_pixel_to_world * transform;

    target_pose.pose.position.x = 0;
    target_pose.pose.position.y = transform.translation().y();
    target_pose.pose.position.z = transform.translation().z();
    target_poses.push_back(target_pose);

    std::cout << "Waypoint (x,y,z) in frame " << WAYPOINT_FRAME_ID << ":  "
              << 0
              << "  " << target_pose.pose.position.y
              << "  " << target_pose.pose.position.z
              << std::endl;
  }

  return target_poses;
}
}  // namespace light_painting_demo
