#!/bin/bash

# --- CONFIGURATION RÉSEAU ---
export ROS_DOMAIN_ID=0

# --- FIX INTERFACE GRAPHIQUE ---
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export GZ_GUI_RENDER_ENGINE_GUESS=ogre
export GZ_RENDERING_ENGINE_GUESS=ogre

# --- CONFIGURATION DES CHEMINS ---
WORKSPACE_DIR=~/ros2_ws
SDF_PATH=$WORKSPACE_DIR/src/drone_binome/worlds/monde_binome.sdf
export GZ_SIM_RESOURCE_PATH=$WORKSPACE_DIR/src/drone_binome/models:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH

# --- NETTOYAGE ---
echo "Nettoyage des processus..."
killall -9 gz-sim-server gz-sim-gui ruby parameter_bridge drone_node joy_node 2>/dev/null
sleep 2

# --- 1. LANCEMENT DE GAZEBO ---
echo "Lancement de Gazebo..."
gz sim -r $SDF_PATH &

# Attente pour s'assurer que le serveur Gazebo a généré les topics
sleep 10

# --- 2. LANCEMENT DU BRIDGE (Version Complète) ---
echo "Lancement du Bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  "/model/drone_leader/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
  "/model/drone_follower/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
  "/model/drone_leader/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist" \
  "/model/drone_follower/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist" \
  "/model/drone_leader/camera@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/model/drone_follower/camera@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/model/drone_leader/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
  "/model/drone_follower/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
  --ros-args -p use_sim_time:=true &

sleep 5

# --- 3. VÉRIFICATION ---
echo "--- Topics ROS 2 détectés ---"
ros2 topic list
echo "----------------------------"

# --- 4. LANCEMENT JOY ET INTERFACE ---
echo "Lancement de Joy..."
ros2 run joy joy_node &

sleep 2

echo "Lancement de l'interface Drone Station..."
source $WORKSPACE_DIR/install/setup.bash
# On lance le node
ros2 run drone_binome drone_node