# Artgslam Visualizer

**Artgslam Visualizer** is a grid map viewer designed for use with the **Artgslam backend**, a sonar-based SLAM system developed for the Adept MobileRobots Amigobot platform.

### ✨ Features

- Grid map base visualization
- Path planing simulation (A star as for right now)
- Real time map creation via Ros 1 sonar data
- Simulating unicicle wmr (wheled mobile robot)
- loading/Saving map data   

### 🔧 Dependencies

| Dependency          | Description                                                                                                                                                   |
|---------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **SFML**            | A simple, fast, cross-platform, and object-oriented multimedia API. Provides access to windowing, graphics, audio, and network.                               |
| **TGUI**            | A cross-platform, modern C++ GUI library built on top of SFML.                                                                                                 |
| **TinyFileDialog**  | A native file dialog library for Windows, macOS, GTK+, Qt, and console/SSH (via fallback to console mode or X11 forwarding).                                  |
| **ROS**             | The Robot Operating System (ROS) is a set of software libraries and tools to help you build robot applications.                                               |


## Instalation Guide 

### Prerequisites

- [SFML](https://www.sfml-dev.org/tutorials/2.5/)
- [TGUI](https://tgui.eu/tutorials/0.9/)
- [TinyFileDialogs](https://sourceforge.net/projects/tinyfiledialogs/)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation) (tested on Ubuntu 20.04)
