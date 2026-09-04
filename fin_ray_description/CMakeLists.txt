cmake_minimum_required(VERSION 3.8)
project(fin_ray_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Encontrar dependências
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(linkattacher_msgs REQUIRED)

# 2. Criar o executável C++
add_executable(gripper_mover src/gripper_mover.cpp)

# 3. Vincular as dependências AO executável criado acima
ament_target_dependencies(gripper_mover rclcpp std_msgs sensor_msgs linkattacher_msgs)

# 4. Instalar o executável para o "ros2 run" encontrar
install(TARGETS
  gripper_mover
  DESTINATION lib/${PROJECT_NAME}
)

# 5. Instalar pastas de recursos (launch, urdf, meshes, config, objects)
install(DIRECTORY launch config urdf meshes worlds objects
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
