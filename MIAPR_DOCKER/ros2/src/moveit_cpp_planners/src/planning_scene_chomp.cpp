#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/kinematic_constraints/utils.hpp>
#include <thread>

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  moveit::planning_interface::MoveGroupInterface::Options options("ur_manipulator");
  //options.planning_pipeline_id = "chomp";

  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, options);


  RCLCPP_ERROR(logger, "test");
  RCLCPP_ERROR(logger, "getDefaultPlanningPipelineId: %s", move_group_interface.getDefaultPlanningPipelineId().c_str());

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                             move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closure for updating the text in rviz
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;  // Place text 1m above the base link
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };
  
  auto const prompt = [&moveit_visual_tools](auto text) { moveit_visual_tools.prompt(text); };

  RCLCPP_INFO(logger, "Available Planning Groups:");
  for (const auto& str : move_group_interface.getJointModelGroupNames()) {
    RCLCPP_INFO(logger, "%s", str.c_str());
  }
  
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur_manipulator")](
          auto const trajectory)
  { return moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

  // Set a target Pose
  auto const target_pose = []
  {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = -1.0;
    msg.position.x = -0.18;
    msg.position.y = 0.3;
    // change to values when executed:
    msg.position.x = 0.3;
    msg.position.y = -0.3;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create two collision objects for the robot to avoid
auto const collision_objects = [frame_id = move_group_interface.getPlanningFrame()] {
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  
  // Common box dimensions (smaller than original)
  const double box_x = 0.1;  // 30 cm
  const double box_y = 0.1;  // 10 cm
  const double box_z = 0.1;  // 30 cm
  
  // Common box orientation
  geometry_msgs::msg::Quaternion box_orientation;
  box_orientation.w = 1.0;

  // First box (original position but smaller)
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {box_x, box_y, box_z};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation = box_orientation;
    box_pose.position.x = -0.3;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.5;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    
    collision_objects.push_back(collision_object);
  }

  // Second box (30 cm to the left of the first box)
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box2";
    
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {box_x, box_y, box_z};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation = box_orientation;
    box_pose.position.x = -0.2;  // 30 cm to the left
    box_pose.position.y = 0.2 - 0.35;
    box_pose.position.z = 0.5;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    
    collision_objects.push_back(collision_object);
  }

  return collision_objects;
}();

// Add the collision objects to the scene
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
planning_scene_interface.applyCollisionObjects(collision_objects);

  // Create a plan to that target pose
  prompt("Press 'next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  RCLCPP_ERROR(logger, "Planning");

  auto const [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    auto draw_success = draw_trajectory_tool_path(plan.trajectory);
    if(draw_success) {
      RCLCPP_ERROR(logger, "Trajectory drawn");
    } else {
      RCLCPP_ERROR(logger, "Trajectory not drawn");
    }
    moveit_visual_tools.trigger();
    draw_title("Executing");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Executing");
    move_group_interface.execute(plan);
  }
  else
  {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  
  rclcpp::shutdown();
  spinner.join();
  return 0;
}