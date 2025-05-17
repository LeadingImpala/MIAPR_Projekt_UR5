#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  robot_model_loader::RobotModelLoader robot_model_loader(node);
  const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
  const auto &LOGGER = node->get_logger();
  RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());

  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("ur_manipulator");

  // Manually set joint configuration (6 values for UR robot)
  std::vector<double> joint_values = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};  // Example configuration
  
  // Set the joint values
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
  
  // Print the joint values
  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // Get the end effector pose for this configuration
  const Eigen::Isometry3d &end_effector_state = kinematic_state->getGlobalLinkTransform("tool0");

  /* Print end-effector pose */
  RCLCPP_INFO_STREAM(LOGGER, "Current end effector position:\n"
                             << end_effector_state.translation() << "\n");
  RCLCPP_INFO_STREAM(LOGGER, "Current end effector orientation:\n"
                             << end_effector_state.rotation() << "\n");

  // Now try to find IK solution for this pose
  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

  if (found_ik)
  {
    RCLCPP_INFO(LOGGER, "Found IK solution!");
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Did not find IK solution");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}