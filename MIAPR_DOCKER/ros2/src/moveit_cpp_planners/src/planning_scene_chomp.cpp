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
////////////////////////////////////////
#include <cmath>
#include <fstream>  
#include <iomanip>

#include <string>
#include <cstdlib>
#include <chrono>

void start_monitor() {
    std::cout << "Starting system load monitor...\n";

    pid_t pid = fork();
    if (pid == 0) {
        // Child process: uruchamiamy skrypt Pythona
        execlp("python3", "python3", "src/moveit_cpp_planners/system_load.py", "start", (char *)nullptr);
        perror("Failed to exec Python script");
        exit(1);
    } else if (pid > 0) {
        // Parent process
        std::this_thread::sleep_for(std::chrono::seconds(2)); // daj Pythonowi czas na wystartowanie
    } else {
        std::cerr << "Fork failed!" << std::endl;
        exit(1);
    }
}

void stop_monitor() {
    std::cout << "Stopping system load monitor...\n";
    int result = system("python3 src/moveit_cpp_planners/system_load.py stop");
    if (result != 0) {
        std::cerr << "Failed to stop monitor\n";
        exit(1);
    }
}

double effector_distance(std::vector<Eigen::Vector3d>& effector_positions){
  auto distance = 0.0;
  Eigen::Vector3d prev_pos;
  bool first = true;

  for (const auto& position : effector_positions)
  {

    if (first){
      prev_pos = position;
      first=false;
    }
    else{
      auto dist=std::sqrt(std::pow((position.x()-prev_pos.x()), 2) + std::pow((position.y()-prev_pos.y()), 2) + std::pow((position.z()-prev_pos.z()), 2));
      prev_pos=position;
      distance+=dist;
    }

  }
  return distance;
}

std::vector<Eigen::Vector3d> effector_positions_kartezian(auto& traj, auto& move_group_interface){
  auto group_name=move_group_interface.getName();
  auto robot_model=move_group_interface.getRobotModel();
  moveit::core::RobotState robot_state(robot_model);

  std::vector<Eigen::Vector3d> efector_positions;  

  for (size_t i = 0; i < traj.points.size(); ++i){
    auto positions = traj.points[i].positions;
    robot_state.setJointGroupPositions(group_name, positions);
    robot_state.updateLinkTransforms();
    auto efector_link = move_group_interface.getEndEffectorLink();
    const Eigen::Isometry3d& eef_transform = robot_state.getGlobalLinkTransform(efector_link);
    const Eigen::Vector3d& position = eef_transform.translation();
    efector_positions.push_back(position);
  }
  return efector_positions;
}

double joint_distance(auto& trajectory){
  std::vector<double> prev_position;
  bool first = true;
  auto distance = 0.0;
  auto points = trajectory.points;
  for(auto& point : points){

    auto position=point.positions;

    if(first){
      prev_position=position;
      first=false;
    }
    else{
      auto dist=0.0;
      for (size_t i=0; i<position.size();i++){
        dist+= std::abs(prev_position[i]-position[i]);
      }
      dist/=position.size();
      prev_position=position;
      distance+=dist;
    }
  }
  return distance;
}

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
      
  node->declare_parameter<std::string>("move_group.planning_pipeline", "chomp");
  node->set_parameter(rclcpp::Parameter("move_group.planning_pipeline", "chomp"));

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

  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, options);
  
  move_group_interface.setPlanningPipelineId("chomp");


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
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10.0);
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("ur_manipulator");

  std::vector<double> joint_group_positions;
  bool found_ik = current_state->setFromIK(joint_model_group, target_pose, 0.1);


  if (found_ik) {
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group_interface.setJointValueTarget(joint_group_positions);
  } else {
    RCLCPP_ERROR(logger, "Inverse kinematics failed for the target pose");
    return 1;
  }
/*
  // Create two collision objects for the robot to avoid
auto const collision_objects = [frame_id = move_group_interface.getPlanningFrame()] {
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  
  // Common box dimensions (smaller than original)
  const double box_x = 0.4;  
  const double box_y = 0.1;  
  const double box_z = 1.1;  
  
  // Common box orientation
  geometry_msgs::msg::Quaternion box_orientation;
  box_orientation.w = 1.0;

 
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {box_x, box_y, box_z};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation = box_orientation;
    box_pose.position.x = 0.2;
    box_pose.position.y = -0.8;
    box_pose.position.z = 0.5;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    
    collision_objects.push_back(collision_object);
  }


  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box2";
    
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.4, 0.4, 0.4};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation = box_orientation;
    box_pose.position.x = -0.3;  
    box_pose.position.y = -0.7;
    box_pose.position.z = 0.2;

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
*/

//otwieranie pliku do zapisu
std::ofstream file("CHOMP.csv");

// Nagłówki kolumn
file << "czas_planowania,il_pkt_w_trajektorii,droga_koncowki,droga_jointow,\n";

//ilosc znalezionych sciezek
int path_succes=0;
for (size_t idx=1; idx<11 ;idx ++){

  // Create a plan to that target pose
  prompt("Press 'next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  RCLCPP_ERROR(logger, "Planning");

  //Starting load measurement
  start_monitor();
  std::this_thread::sleep_for(std::chrono::seconds(2));


  auto const [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  //Stopping load measurement
  stop_monitor();
  std::this_thread::sleep_for(std::chrono::seconds(2));
  

  // Execute the plan
  if (success)
  {
   //////////////////////////////////////////////////////
    path_succes+=1;
    // czas planowania
    auto czas_planowania =plan.planning_time;
    RCLCPP_INFO(logger, "Planowanie trwało: %f s", czas_planowania);
    // trajektoria robota
    auto robot_trajectory=plan.trajectory.joint_trajectory;

    // ilość punktów w trajektorii
    size_t points_count = robot_trajectory.points.size();
    RCLCPP_INFO(logger, "Liczba punktów w trajektorii: %zu", points_count);
    
    // obliczanie pozycji końcówki robota w przestrzeni kartezjanskiej
    auto efector_positions = effector_positions_kartezian(robot_trajectory, move_group_interface);
    
    // obliczanie drogi dla koncowki 
    auto distance = effector_distance(efector_positions);
    RCLCPP_INFO(logger,"Długość drogi przebytej przez końcówkę: %.3f m\n", distance);

    //obliczanie drogi dla jointow (srednia z roznicy obrotow)
    auto distance_joints=joint_distance(robot_trajectory);
    RCLCPP_INFO(logger,"Długość drogi jointow: %.3f\n rad", distance_joints);

    file << std::fixed << std::setprecision(4) << czas_planowania << "," << points_count << "," << distance << "," << distance_joints<< "\n";
 
    ///////////////////
  }
  else
  {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }
}
  RCLCPP_INFO(logger, "Ilość znalezionych sciezek: %d", path_succes);
  file << "\nLiczba udanych prób: " << path_succes << "\n";
  file.close();

  // Shutdown ROS
  
  rclcpp::shutdown();
  spinner.join();
  return 0;
}