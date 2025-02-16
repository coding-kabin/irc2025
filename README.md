
# irc2025 (ROS2 Migration)

Migration of Project Kratos' irc2025 repo from ROS1 Noetic to ROS2 Humble.

ROS2 Installation instructions : https://docs.ros.org/en/humble/Installation.html




## File Structure and New File Locations

To make ROS2 work with both C++ as well as Python follow this file structure :
```bash
irc2025/
# --> package info, configuration, and compilation
├── CMakeLists.txt
├── package.xml
--> Python files
├── irc2025
│   ├── __init__.py
│   └── module_to_import.py
├── scripts
│   └── py_node.py
--> Cpp stuff
├── include
│   └── irc2025
│       └── cpp_header.hpp
└── src
    └── cpp_node.cpp
```

* New python scripts should be added to **irc2025/scripts** 
* Python modules should be added to **irc2025/irc2025**
* New C++ programs should be added to **irc2025/src**
* New C++ header files should be added to **irc2025/include/irc2025**

Once you have written new scripts, follow the next section to see what changes are to be made to the CMakeLists.txt as well as package.xml files.


## Changes in CMakeLists and package.xml

#### C++ Files:
Once you have written a C++ file, you need to add executables and install them. This is done through the CMakeLists.txt file.

Considering a file drive.cpp which imports std_msgs and rclcpp, you need to add the following:

```bash
add_executable(drive_executable src/drive.cpp)
ament_target_dependencies(drive_executable rclcpp std_msgs)
```
and add the executable name to the install as follows
```bash
install(TARGETS
  .
  .
  drive_executable
  DESTINATION lib/${PROJECT_NAME}
)
```

#### Python Scripts:
Once written the python files must also be mentioned in its own install.

Considering a file drive.py, we need to add the following :
```bash
install(PROGRAMS
  .
  .
  scripts/drive.py
  DESTINATION lib/${PROJECT_NAME}
)
```




## How to Run Programs

Clone the repo within your ros2 workspace's *src* and run the following command from within the workspace

```bash
colcon build
```
This is equivalent to catkin make. After this source the worspace similar to ROS using
```bash
source install/setup.bash
```
then running programs is just
```bash
ros2 run irc2025 drive.py
ros2 run irc2025 drive_executable
```

Note: ROS2 supports launch files in the form of python scripts that have not yet been written
