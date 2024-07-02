cmake_minimum_required(VERSION 3.5)
project(stretch_web_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbridge_server REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(tf2_web_republisher REQUIRED)

##############################
## Generate Custom Messages ##
##############################
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPregrasp.action"
  "msg/TextToSpeech.msg"
  DEPENDENCIES geometry_msgs
)


#############
## Install ##
#############

ament_python_install_package(${PROJECT_NAME}_helpers)

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  nodes/compressed_image_visualizer.py
  nodes/configure_video_streams.py
  nodes/move_to_pregrasp.py
  nodes/navigation_camera.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY certificates
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY nodes
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
