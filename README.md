# Projekt Metody I Algorytmy Planowania Ruchu
## Pobranie obrazu Dockera
docker pull osrf/ros:jazzy-desktop-full

## Uruchomienie kontenera Dockera (przykład, dostosuj wg własnego skryptu lub środowiska)
./run_container_ros2.sh

## Po wejściu do kontenera wykonaj:
apt update
export COLCON_WS=~/Shared/ros2_ws
mkdir -p $COLCON_WS/src
apt-get install ros-jazzy-ur
source /opt/ros/jazzy/setup.bash

## Uruchamianie sterowników robota UR5e:
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=127.0.0.1 use_mock_hardware:=true launch_rviz:=false initial_joint_controller:=scaled_joint_trajectory_controller

## Uruchamianie MoveIt dla robota UR5e:
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true use_mock_hardware:=true

## Uruchamianie planerów ruchu (wybierz jeden z planerów: ompl, stomp, chomp):
ros2 launch moveit_cpp_planners planning_scene<ompl|stomp|chomp>.py

