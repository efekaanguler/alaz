cmake_minimum_required(VERSION 3.8)
project(ros2_can_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python package'ı (ros2_can_bridge/__init__.py dahil) site-packages'a kurar
ament_python_install_package(${PROJECT_NAME})


# launch dosyaları
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# resource index (colcon/ament index için genelde şart)
install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()