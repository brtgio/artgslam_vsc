cmake_minimum_required(VERSION 3.0.2)
project(artgslam_vsc)

## Habilitar C++ moderno (recomendado)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Encuentra ROS y dependencias
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  rosaria
  joy
  roslib
)

## Encuentra SFML
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

## Encuentra TGUI
find_package(TGUI REQUIRED)

## tinyfiledialogs (archivo C puro)
set(TINYFILEDIALOG_SRC src/tinyfiledialogs.c)

## Configura Catkin
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs tf rosaria joy
  DEPENDS SFML TGUI
  INCLUDE_DIRS include
)

## Incluye directorios
include_directories(
  include  # <- Aquí es más limpio y correcto
  ${catkin_INCLUDE_DIRS}
  ${SFML_INCLUDE_DIRS}
  ${TGUI_INCLUDE_DIRS}
)

## Lista de archivos fuente
set(SOURCES
  src/main.cpp
  src/MenuBar.cpp
  src/MapViewer.cpp
  src/FileManager.cpp
  src/GridMap.cpp
  src/ViewController.cpp
  src/RosHandler.cpp
  src/RobotCreator.cpp
  ${TINYFILEDIALOG_SRC}
)

## Crea el ejecutable
add_executable(${PROJECT_NAME}_node ${SOURCES})

## Enlazar librerías
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  sfml-graphics
  sfml-window
  sfml-system
  TGUI::TGUI
)

## Instalar ejecutable (opcional pero recomendado)
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Instalar headers (opcional pero recomendado si compartes el paquete)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
