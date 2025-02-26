
# irc2025 (ROS2 Migration)

Migration of Project Kratos' irc2025 repo from ROS1 Noetic to ROS2 Humble.

ROS2 Installation instructions : [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)

- [File Structure and New File Locations](#file-structure-and-new-file-locations)
- [Changes in CMakeLists and package.xml](#changes-in-cmakelists-and-packagexml)
- [How to Run Programs](#how-to-run-programs)
- [How to Use MicroROS](#how-to-use-microros)
  - [MicroROS Header Files and Variables](#whats-necessary-for-a-microros-program)
  - [Making Code Disconnection Proof (Code Structure)](#dealing-with-disconnection)


## File Structure and New File Locations

To make ROS2 work with both C++ as well as Python follow this file structure :
```
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

```cmake
add_executable(drive_executable src/drive.cpp)
ament_target_dependencies(drive_executable rclcpp std_msgs)
```
and add the executable name to the install as follows
```cmake
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
```cmake
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

## How to Use MicroROS

At the time of writing the current README, none of the included microROS codes have been tested on the rover. That being said they compile and give feedback with an ESP32 attached.

Follow microROS installation instructions [here](https://micro.ros.org/docs/tutorials/core/first_application_linux/).

Install the microros Arduino IDE library from [here](https://github.com/micro-ROS/micro_ros_arduino).

### What's Necessary for A MicroROS Program

These are the packages that are usually required for a microROS program

```cpp
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
```

- ```micro_ros_arduino.h``` : Sets up the Arduino for Micro-ROS.
- ```rcl/rcl.h``` : Provides the core ROS 2 functionality.
- ```rcl/error_handling.h``` : Helps you handle errors in ROS 2 operations. Its usage will be expanded on [later]().
- ```rclc/rclc.h``` : Simplifies ROS 2 API usage for microcontrollers. 
- ```rclc/executor.h``` : Manages the execution of subscription or timer driven callback functions.
- ```rmw_microros/rmw_microros.h``` : Handles the communication layer between the microcontroller and the ROS 2 network

Subsequntly the various required variables are initialised

- ```rcl_allocator_t allocator``` : Provides memory allocation for all ROS 2 entities.
- ```rclc_support_t support``` : Initializes the ROS 2 context and holds the allocator.
- ```rcl_node_t node``` : Creates a ROS 2 node, which acts as the main container for publishers, subscribers, etc.
- ```rcl_publisher_t publisher``` : Publishes messages to a topic.
- ```rcl_subscription_t subscriber``` : Listens for messages on a topic and triggers a callback.
- ```rclc_executor_t executor``` : Manages and executes callbacks for subscriptions, timers, and other events.

### Dealing With Disconnection

When you normally write code using microROS, you will realise you need to keep reflashing the microcontroller before running the agent. This problem can be solved by changing the structure of the code as elaborated below. (refer [this](https://github.com/micro-ROS/micro_ros_arduino/blob/jazzy/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino))

```cpp
//Header Files
//Variable declaration
.
.
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

.
void subscription_callback(){
.
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "sample_node", "", &support)); //node creation
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "sample_publisher")); //publisher creation

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "sample_subscriber")); //subscriber creation

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  return true;
}
void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  .
  .
  state = WAITING_AGENT;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        //THIS IS WHERE YOUR MAIN CODE WILL GO
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}
```
```create_entities()``` initializes the Micro-ROS node, publisher, timer, and executor.

```destroy_entities()``` cleans up the Micro-ROS entities when the agent is disconnected.

```loop()``` handles the state machine for managing the connection with the Micro-ROS agent.

- ```WAITING_AGENT```: Pings the agent every 500ms. If the agent is available, transitions to ```AGENT_AVAILABLE```.
- ```AGENT_AVAILABLE```: Attempts to create Micro-ROS entities. If successful, transitions to ```AGENT_CONNECTED```; otherwise, goes back to ```WAITING_AGENT```.
- ```AGENT_CONNECTED```: If the agent is connected runs the code and spins as per user requirement.
- ```AGENT_DISCONNECTED```: Destroys the Micro-ROS entities and transitions back to WAITING_AGENT.

