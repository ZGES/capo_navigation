// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_PLANNER__PLANNER_SERVER_HPP_
#define NAV2_PLANNER__PLANNER_SERVER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/global_planner.hpp"
#include "builtin_interfaces/msg/duration.hpp"

namespace nav2_planner
{
/**
 * @class nav2_planner::PlannerServer
 * @brief An action server implements the behavior tree's ComputePathToPose
 * interface and hosts various plugins of different algorithms to compute plans.
 */
class PlannerServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_planner::PlannerServer
   */
  PlannerServer();
  /**
   * @brief A destructor for nav2_planner::PlannerServer
   */
  ~PlannerServer();

  using PlannerMap = std::unordered_map<std::string, nav2_core::GlobalPlanner::Ptr>;

  /**
   * @brief Method to get plan from the desired plugin
   * @param start starting pose
   * @param goal goal request
   * @return Path
   */
  nav_msgs::msg::Path getPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id);

protected:
  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using ActionT = nav2_msgs::action::ComputePathToPose;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  // Our action server implements the ComputePathToPose action
  std::unique_ptr<ActionServer> action_server_;

  /**
   * @brief The action server callback which calls planner to get the path
   */
  void computePlan();

  /**
   * @brief Publish a path for visualization purposes
   * @param path Reference to Global Path
   */
  void publishPlan(const nav_msgs::msg::Path & path);

  /**
   * @brief Publish a duration for evaluation purposes
   * @param duration Reference to path planning duration
   */
  void publishDuration(const builtin_interfaces::msg::Duration & duration);

  // Planner
  PlannerMap planners_;
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> gp_loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> planner_ids_;
  std::vector<std::string> planner_types_;
  double max_planner_duration_;
  std::string planner_ids_concat_;

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Global Costmap
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
  nav2_costmap_2d::Costmap2D * costmap_;

  // Publishers for the path
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<builtin_interfaces::msg::Duration>::SharedPtr duration_publisher_;
};

}  // namespace nav2_planner

#endif  // NAV2_PLANNER__PLANNER_SERVER_HPP_