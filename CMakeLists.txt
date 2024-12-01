cmake_minimum_required(VERSION 3.5)
project(robot_manager)

# Encontrar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Instalar archivos del backend y frontend
install(DIRECTORY backend frontend launch
  DESTINATION share/${PROJECT_NAME}
)

# Instalar scripts
install(PROGRAMS
  backend/server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

