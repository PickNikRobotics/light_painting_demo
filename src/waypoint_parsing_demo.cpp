// light_painting_demo
#include <light_painting_demo/parse_image_waypoints.h>

// ROS
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  light_painting_demo::ParseImageWaypoints parser;
  std::string yaml_path = "/home/andy/ws_light_demo/src/light_painting_demo/resources/simple_devel_image.yaml";
  parser.loadWaypointsFromFile(yaml_path);

  return 0;
}
