#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

struct Parameters {
  std::vector<double> pick;
  std::vector<double> place;
  std::vector<double> tool_offset;
  std::vector<double> orientation;
  double pregrasp_clearance;
  double grasp_clearance;
  double retreat_distance;
  double eef_step;
  double jump_threshold;
  double min_fraction;
  double velocity_scaling;
  double acceleration_scaling;
  double gripper_open;
  double gripper_closed;
};

geometry_msgs::msg::Pose make_tool_pose(
    const std::vector<double>& xyz, const Parameters& p, double clearance) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = xyz.at(0) + p.tool_offset.at(0);
  pose.position.y = xyz.at(1) + p.tool_offset.at(1);
  pose.position.z = xyz.at(2) + 0.025 + p.tool_offset.at(2) + clearance;
  pose.orientation.x = p.orientation.at(0);
  pose.orientation.y = p.orientation.at(1);
  pose.orientation.z = p.orientation.at(2);
  pose.orientation.w = p.orientation.at(3);
  return pose;
}

bool execute_pose(moveit::planning_interface::MoveGroupInterface& group,
                  const geometry_msgs::msg::Pose& pose) {
  group.setStartStateToCurrentState();
  group.setPoseTarget(pose, "gripper_base");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool planned = static_cast<bool>(group.plan(plan));
  group.clearPoseTargets();
  return planned && static_cast<bool>(group.execute(plan));
}

bool execute_cartesian(moveit::planning_interface::MoveGroupInterface& group,
                       const std::vector<geometry_msgs::msg::Pose>& waypoints,
                       double eef_step, double jump_threshold, double min_fraction,
                       double velocity_scaling, double acceleration_scaling) {
  moveit_msgs::msg::RobotTrajectory message;
  const double fraction = group.computeCartesianPath(
      waypoints, eef_step, jump_threshold, message, true);
  if (fraction < min_fraction) {
    RCLCPP_ERROR(rclcpp::get_logger("fixed_pick_place"),
                 "Cartesian path rejected: %.3f < %.3f", fraction, min_fraction);
    return false;
  }
  robot_trajectory::RobotTrajectory trajectory(group.getRobotModel(), "arm");
  trajectory.setRobotTrajectoryMsg(*group.getCurrentState(), message);
  trajectory_processing::IterativeParabolicTimeParameterization timing;
  if (!timing.computeTimeStamps(trajectory, velocity_scaling, acceleration_scaling)) {
    return false;
  }
  trajectory.getRobotTrajectoryMsg(message);
  return static_cast<bool>(group.execute(message));
}

void command_gripper(
    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& publisher,
    double command) {
  std_msgs::msg::Float64 message;
  message.data = command;
  publisher->publish(message);
  std::this_thread::sleep_for(1200ms);
}

void add_world_objects(moveit::planning_interface::PlanningSceneInterface& scene) {
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = "base";
  table.id = "work_table";
  shape_msgs::msg::SolidPrimitive table_shape;
  table_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  table_shape.dimensions = {0.60, 0.50, 0.05};
  geometry_msgs::msg::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x = 0.14;
  table_pose.position.z = -0.025;
  table.primitives.push_back(table_shape);
  table.primitive_poses.push_back(table_pose);
  table.operation = moveit_msgs::msg::CollisionObject::ADD;

  moveit_msgs::msg::CollisionObject cube;
  cube.header.frame_id = "base";
  cube.id = "target_object";
  shape_msgs::msg::SolidPrimitive cube_shape;
  cube_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  cube_shape.dimensions = {0.035, 0.035, 0.05};
  geometry_msgs::msg::Pose cube_pose;
  cube_pose.orientation.w = 1.0;
  cube_pose.position.x = 0.18;
  cube_pose.position.y = 0.08;
  cube_pose.position.z = 0.025;
  cube.primitives.push_back(cube_shape);
  cube.primitive_poses.push_back(cube_pose);
  cube.operation = moveit_msgs::msg::CollisionObject::ADD;
  scene.applyCollisionObjects({table, cube});
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
      "fixed_pick_place", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor] { executor.spin(); });

  Parameters p{
      node->get_parameter("pick_xyz").as_double_array(),
      node->get_parameter("place_xyz").as_double_array(),
      node->get_parameter("tool_offset_xyz").as_double_array(),
      node->get_parameter("tool_orientation_xyzw").as_double_array(),
      node->get_parameter("pregrasp_clearance").as_double(),
      node->get_parameter("grasp_clearance").as_double(),
      node->get_parameter("retreat_distance").as_double(),
      node->get_parameter("eef_step").as_double(),
      node->get_parameter("jump_threshold").as_double(),
      node->get_parameter("cartesian_fraction_threshold").as_double(),
      node->get_parameter("velocity_scaling").as_double(),
      node->get_parameter("acceleration_scaling").as_double(),
      node->get_parameter("gripper_open").as_double(),
      node->get_parameter("gripper_closed").as_double(),
  };

  moveit::planning_interface::MoveGroupInterface group(node, "arm");
  RCLCPP_INFO(node->get_logger(), "GROUP_READY");
  moveit::planning_interface::PlanningSceneInterface scene;
  auto gripper = node->create_publisher<std_msgs::msg::Float64>("/mecharm/gripper_command", 10);
  group.setEndEffectorLink("gripper_base");
  group.setMaxVelocityScalingFactor(p.velocity_scaling);
  group.setMaxAccelerationScalingFactor(p.acceleration_scaling);
  group.setPlanningTime(5.0);
  group.setNumPlanningAttempts(5);
  if (!group.getCurrentState(5.0)) {
    RCLCPP_ERROR(node->get_logger(), "NO_JOINT_STATE");
    executor.cancel(); spin_thread.join(); rclcpp::shutdown(); return 2;
  }
  RCLCPP_INFO(node->get_logger(), "STATE_READY");
  add_world_objects(scene);
  std::this_thread::sleep_for(500ms);
  command_gripper(gripper, p.gripper_open);
  RCLCPP_INFO(node->get_logger(), "SCENE_READY");

  group.setNamedTarget("home");
  RCLCPP_INFO(node->get_logger(), "HOME_START");
  if (!static_cast<bool>(group.move())) {
    RCLCPP_ERROR(node->get_logger(), "HOME_FAILED");
    executor.cancel(); spin_thread.join(); rclcpp::shutdown(); return 3;
  }
  RCLCPP_INFO(node->get_logger(), "HOME_DONE");

  const auto pick_high = make_tool_pose(p.pick, p, p.pregrasp_clearance);
  const auto pick_low = make_tool_pose(p.pick, p, p.grasp_clearance);
  if (!execute_pose(group, pick_high) ||
      !execute_cartesian(group, {pick_low}, p.eef_step, p.jump_threshold,
                         p.min_fraction, p.velocity_scaling, p.acceleration_scaling)) {
    RCLCPP_ERROR(node->get_logger(), "PICK_APPROACH_FAILED");
    executor.cancel(); spin_thread.join(); rclcpp::shutdown(); return 4;
  }
  command_gripper(gripper, p.gripper_closed);
  group.attachObject("target_object", "gripper_base");

  if (!execute_cartesian(group, {pick_high}, p.eef_step, p.jump_threshold,
                         p.min_fraction, p.velocity_scaling, p.acceleration_scaling)) {
    RCLCPP_ERROR(node->get_logger(), "LIFT_FAILED");
    executor.cancel(); spin_thread.join(); rclcpp::shutdown(); return 5;
  }

  const auto place_high = make_tool_pose(p.place, p, p.pregrasp_clearance);
  const auto place_low = make_tool_pose(p.place, p, p.grasp_clearance);
  if (!execute_pose(group, place_high) ||
      !execute_cartesian(group, {place_low}, p.eef_step, p.jump_threshold,
                         p.min_fraction, p.velocity_scaling, p.acceleration_scaling)) {
    RCLCPP_ERROR(node->get_logger(), "PLACE_APPROACH_FAILED");
    executor.cancel(); spin_thread.join(); rclcpp::shutdown(); return 6;
  }
  command_gripper(gripper, p.gripper_open);
  group.detachObject("target_object");
  scene.removeCollisionObjects({"target_object"});
  if (!execute_cartesian(group, {place_high}, p.eef_step, p.jump_threshold,
                         p.min_fraction, p.velocity_scaling, p.acceleration_scaling)) {
    RCLCPP_ERROR(node->get_logger(), "RETREAT_FAILED");
    executor.cancel(); spin_thread.join(); rclcpp::shutdown(); return 7;
  }
  group.setNamedTarget("home");
  const bool success = static_cast<bool>(group.move());
  RCLCPP_INFO(node->get_logger(), success ? "PICK_PLACE_SUCCESS" : "RETURN_HOME_FAILED");
  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return success ? 0 : 8;
}
