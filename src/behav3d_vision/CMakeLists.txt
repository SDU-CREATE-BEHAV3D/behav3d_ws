cmake_minimum_required(VERSION 3.8)
project(behav3d_vision)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the Python package (module)
ament_python_install_package(${PROJECT_NAME})

# Install Python executables (wrappers are just the scripts themselves)
install(PROGRAMS
  behav3d_vision/image_processor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME image_processor
)

install(PROGRAMS
  behav3d_vision/image_processor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME image_processor
)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
