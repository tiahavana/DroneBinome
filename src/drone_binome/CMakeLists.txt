cmake_minimum_required(VERSION 3.8)
project(drone_binome)

# Compilation en C++ 17 pour la compatibilité ROS 2 Humble/Iron/Jazzy
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 1. RECHERCHE DES DÉPENDANCES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Dépendances pour l'interface graphique Qt5
find_package(Qt5 REQUIRED COMPONENTS Widgets Gui Core UiTools)

# Activation de l'automatisation Qt (MOC et UI)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# 2. CONFIGURATION DE L'EXÉCUTABLE
# On compile drone_main.cpp qui contient ton int main()
add_executable(drone_node src/drone_main.cpp)

# Liens avec les bibliothèques Qt5
target_link_libraries(drone_node 
    Qt5::Widgets 
    Qt5::Gui 
    Qt5::Core 
    Qt5::UiTools
)

# Liens avec les dépendances ROS 2
ament_target_dependencies(drone_node 
    rclcpp 
    geometry_msgs 
    sensor_msgs 
    nav_msgs
)

# 3. INSTALLATION (Pour que 'ros2 run' et Gazebo trouvent les fichiers)
# Installe l'exécutable dans le dossier lib du workspace
install(TARGETS drone_node
  DESTINATION lib/${PROJECT_NAME})

# Installe les dossiers worlds et models dans le dossier share
# C'est ce qui permet à Gazebo de trouver monde_binome.sdf
install(DIRECTORY worlds models
  DESTINATION share/${PROJECT_NAME})

# 4. EXPORT DU PACKAGE
ament_package()