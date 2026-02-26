cmake_minimum_required(VERSION 3.8)
project(ros2_can_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python package'ı (ros2_can_bridge/__init__.py dahil) site-packages'a kurar
ament_python_install_package(${PROJECT_NAME})



# launch dosyalarını recursive olarak kopyala (sadece dosyalar)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# install node script so launch can find executables under install/lib/<package>
install(
  PROGRAMS
    ros2_can_bridge/bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# resource dosyasını kopyala (varsa)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resource/${PROJECT_NAME}")
  install(
    FILES resource/${PROJECT_NAME}
    DESTINATION share/ament_index/resource_index/packages
  )
endif()

# package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()