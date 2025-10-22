cmake_minimum_required(VERSION 3.0.2)
project(pepper_apps)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

# Install Python nodes so `rosrun pepper_apps <script>` works
catkin_install_python(PROGRAMS
  scripts/pepper_forward.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
