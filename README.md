# Artgslam Visualizer

**Artgslam Visualizer** is a grid map viewer designed for use with the **Artgslam backend**, a sonar-based SLAM system developed for the Adept MobileRobots Amigobot platform. Full code documentation can be found here https://brtgio.github.io/artgslam_vsc/html/index.html.

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

Ensure the following libraries and tools are installed:
- [SFML](https://www.sfml-dev.org/tutorials/2.5/)
- [TGUI](https://tgui.eu/tutorials/0.9/)
- [TinyFileDialogs](https://sourceforge.net/projects/tinyfiledialogs/)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation) (tested on Ubuntu 20.04)

Optional – required **only** for integration with Adept MobileRobots (e.g., Amigobot):
- [Aria](https://github.com/cinvesrob/Aria) (tested on ubuntu 20.04 and debian 10)
- [RosAria](https://github.com/amor-ros-pkg/rosaria) (tested on ubuntu 20.04 and debian 10)

> 🧠 **Note**: The ROS topics for receiving sonar data are hardcoded by default, see RosHandler(https://brtgio.github.io/artgslam_vsc/html/classRosHandler.html) API documentation. Dynamic configuration is being considered for future versions.


### 🛠️ Set Up ROS Workspace

If you don’t already have a catkin workspace set up:

```bash
# Example: ROS workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make

```

### Build the project

```bash

# Navigate to the 'src' directory of your workspace
cd ~/catkin_ws/src

# Clone this repository
git clone https://github.com/brtgio/artgslam_vsc.git

# Go back to the workspace root and build
cd ~/catkin_ws
catkin_make
```
## 🧪 **Usage**


```bash

# In one terminal, start the ROS core
roscore
```
```bash
# In a new terminal, run the Artgslam Visualizer node
rosrun artgslam_vsc artgslam_vsc_node
```


## 📚 Licenses

### Third-Party Libraries

This project incorporates the following open-source libraries. Please refer to their respective licenses for more details:

- **SFML** – [zlib/libpng License](https://www.sfml-dev.org/license.php)
- **TGUI** – [zlib/libpng License](https://tgui.eu/license/)
- **TinyFileDialogs** – [WTFPL or MIT License](https://sourceforge.net/projects/tinyfiledialogs/)
- **ROS** – [BSD License](https://www.ros.org/reps/rep-0003.html)

All credit is given to the original authors and maintainers of these libraries.

This project is provided for personal, academic, and non-commercial use only.  
For the full terms and conditions, please refer to the `LICENSE.txt` file included in this repository.
