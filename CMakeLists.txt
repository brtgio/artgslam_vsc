cmake_minimum_required(VERSION 3.0.2)
project(artgslam_vsc)

## C++ moderno (RECOMENDADO)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Encuentra ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  rosaria
  joy
)

## Encuentra SFML
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

## Encuentra TGUI
find_package(TGUI REQUIRED)

## tinyfiledialogs
set(TINYFILEDIALOG_SRC src/tinyfiledialogs.c)

## Configura Catkin
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs tf rosaria joy
  DEPENDS SFML TGUI
)

## Incluye directorios
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${SFML_INCLUDE_DIRS}
  ${TGUI_INCLUDE_DIRS}  # <-- Esto te faltaba
  ${CMAKE_CURRENT_SOURCE_DIR}/src  # Incluye los headers de tu proyecto
)

## Lista de archivos fuente
set(SOURCES
  src/main.cpp
  src/MenuBar.cpp
  src/MapViewer.cpp
  src/FileManager.cpp
  src/GridMap.cpp          # <-- Faltaba
  src/ViewController.cpp   # <-- Faltaba
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
