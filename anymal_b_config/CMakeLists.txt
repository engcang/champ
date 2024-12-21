cmake_minimum_required(VERSION 3.5)
project(anymal_b_config)

find_package(ament_cmake REQUIRED)

# Install the launch directory
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install the urdf directory
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install the config directory
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  champ_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
