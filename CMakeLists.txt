cmake_minimum_required(VERSION 3.0.2)
project(switch)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  hippocampus_common
  hippocampus_msgs
  mavros_msgs
)

add_message_files(
  FILES
  BatteryState.msg
  Button.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# catkin_python_setup()

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  nodes/switch_node.py
  nodes/switch_node_px4.py
  nodes/button.py
  nodes/sound.py
  nodes/led.py
  nodes/button_handler.py
  nodes/battery_watcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
