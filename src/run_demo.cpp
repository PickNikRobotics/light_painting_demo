/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Inc
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Andy Zelenak, Boston Cleek
   Desc: Follow a series of waypoints from image file.
*/

#include <light_painting_demo/parse_image_waypoints.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("run_demo");
static const std::string GROUP_NAME = "ur_manipulator";
static const std::string EEF_NAME = "tool0";

static const double path_tolerance = 0.1;
static const double resample_dt = 0.1;
static const double min_angle_change = 0.001;

static const double max_velocity_scaling_factor = 1.0;
static const double max_acceleration_scaling_factor = 1.0;


// TODO(andyz): do not hard-code this
static const std::string YAML_PATH = "/home/andy/ws_light_demo/src/light_painting_demo/resources/simple_devel_image.yaml";
//static const std::string YAML_PATH = "/home/boston/light_ws/src/light_painting_demo/resources/simple_devel_image.yaml";

class RunDemo
{
public:

  RunDemo(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  , robot_state_publisher_(node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
  {
  }

  void run()
  {
    if (!parser_.loadWaypointsFromFile(YAML_PATH))
    {
      RCLCPP_ERROR(LOGGER, "Failed to load image waypoint file %s: ",  YAML_PATH.c_str());
      return;
    }

    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    moveit::planning_interface::PlanningComponent arm(GROUP_NAME, moveit_cpp_);

    // Waypoints
    const std::vector<geometry_msgs::msg::PoseStamped> waypoints = parser_.transformPixelCoordinatesToRobotPose();

    // Combined trajectory
    auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(moveit_cpp_->getRobotModel(), GROUP_NAME);

    // A little delay before planning
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Get current robot state
    moveit::core::RobotStatePtr current_state = arm.getStartState();
    RCLCPP_INFO(LOGGER, "Arm start state: ");
    current_state->printStatePositions();
    arm.setStartState(*current_state);

    // Plan to each waypoint
    for (const auto& waypoint :  waypoints)
    {
      if (!arm.setGoal(waypoint, EEF_NAME))
      {
        RCLCPP_ERROR(LOGGER, "Failed to set goal");
        return;
      }

      const auto plan = arm.plan();
      if (plan.error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      {
        RCLCPP_ERROR(LOGGER, "Planning Failed");
        return;
      }

      // Combine trajectories
      trajectory->append(*plan.trajectory.get(), 0.1 /* placeholder dt, timestamps are recomputed */);

      // Update robot start state
      if (!arm.setStartState(plan.trajectory->getLastWayPoint()))
      {
        RCLCPP_ERROR(LOGGER, "Failed to set start state");
        return;
      }
    }

    // TOTG
    trajectory_processing::TimeOptimalTrajectoryGeneration totg(path_tolerance, resample_dt, min_angle_change);
    if (!totg.computeTimeStamps(*trajectory.get(), max_velocity_scaling_factor, max_acceleration_scaling_factor))
    {
      RCLCPP_ERROR(LOGGER, "TOTG Failed");
      return;
    }

    if (!moveit_cpp_->execute(GROUP_NAME, trajectory))
    {
      RCLCPP_ERROR(LOGGER, "Failed to execute trajectory");
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  light_painting_demo::ParseImageWaypoints parser_;
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;
};

int main(int argc, char** argv)
{
  RCLCPP_INFO(LOGGER, "Initializing node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_demo", "", node_options);

  RunDemo demo(node);
  std::thread run_demo([&demo]() {
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();

  return 0;
}
