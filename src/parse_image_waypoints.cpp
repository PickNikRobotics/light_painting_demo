#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <light_painting_demo/parse_image_waypoints.h>

namespace light_painting_demo
{

namespace
{
  static const std::string WAYPOINT_FRAME_ID = "base_link";
  static constexpr double PIXEL_TO_WORLD_SCALE = 0.002;  // unitless, converts pixels to meters
  static constexpr double PAINT_PLANE_X_COORDINATE = 0.5;  // meters
  static constexpr size_t IMAGE_WIDTH_IN_PIXELS = 1000;  // pixel width and height
  static constexpr double HEIGHT_TO_WORKSPACE_CENTER = 0.5;  // meters
}

bool ParseImageWaypoints::loadWaypointsFromFile(const std::string& yaml_file)
{
  std::cout << "Loading waypoints file: " << yaml_file << std::endl;
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

  std::vector<geometry_msgs::msg::PoseStamped> target_poses;
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = WAYPOINT_FRAME_ID;

  Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());

  Eigen::Quaterniond rot_90_about_x(0.7071, 0.7071, 0, 0);
  Eigen::Quaterniond rot_90_about_z(0.7071, 0, 0, 0.7071);
  Eigen::Quaterniond net_rotation_from_pixel_to_world = (rot_90_about_x * rot_90_about_z).inverse();

  // Mirror Y-coordinates
  Eigen::Isometry3d mirror(Eigen::Isometry3d::Identity());
  mirror(1,1) = -1;

  for (size_t waypoint_idx = 0; waypoint_idx < pixel_waypoints_x_y_.size(); ++waypoint_idx)
  {
    // Translate the pixel to the center of the robot workspace
    transform = Eigen::Isometry3d::Identity();
    transform.translation().x() = -(0.5 * IMAGE_WIDTH_IN_PIXELS - pixel_waypoints_x_y_[waypoint_idx].first);
    transform.translation().y() = -(0.5 * IMAGE_WIDTH_IN_PIXELS - pixel_waypoints_x_y_[waypoint_idx].second);

    // Rotate to match WAYPOINT_FRAME_ID axes
    transform = mirror * net_rotation_from_pixel_to_world * transform;

    // X (depth) is constant
    transform.translation().x() = PAINT_PLANE_X_COORDINATE;

    // Scale
    transform.translation().y() *= PIXEL_TO_WORLD_SCALE;
    transform.translation().z() *= PIXEL_TO_WORLD_SCALE;

    // Translate, since origin is at the base of the robot
    transform.translation().z() += HEIGHT_TO_WORKSPACE_CENTER;

    std::cout << "x4: " << transform.translation().x() << std::endl;
    std::cout << "y4: " << transform.translation().y() << std::endl;
    std::cout << "z4: " << transform.translation().z() << std::endl;
    std::cout << " -- " << std::endl;
  }

  return target_poses;
}
}  // namespace light_painting_demo
