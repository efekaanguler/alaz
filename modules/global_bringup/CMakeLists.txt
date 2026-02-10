cmake_minimum_required(VERSION 3.8)
project(global_bringup)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 bağımlılıkları
find_package(ament_cmake REQUIRED)

# ========================================
# DOSYA KURULUMU (INSTALL)
# ========================================
# ROS2 paketi çalıştığında bu dosyalara erişebilmek için
# share dizinine kopyalanması gerekir

# Launch dosyalarını kur
# YENİ LAUNCH DOSYASI EKLEDİĞİNİZDE:
# - Otomatik olarak kurulur (DIRECTORY kullanıldığı için)
# - launch/ klasörüne .launch.py uzantılı dosya eklemeniz yeterli
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Config dosyalarını kur
# YENİ CONFIG DOSYASI EKLEDİĞİNİZDE:
# - Otomatik olarak kurulur (DIRECTORY kullanıldığı için)
# - config/ klasörüne .yaml uzantılı dosya eklemeniz yeterli
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ========================================
# YENİ MODÜL/PAKET EKLEDİĞİNİZDE NE YAPMALI?
# ========================================
# 1. EĞER SADECE LAUNCH/CONFIG DOSYASI EKLİYORSANIZ:
#    - Bu CMakeLists.txt'yi DEĞİŞTİRMENİZE GEREK YOK
#    - Dosyayı launch/ veya config/ klasörüne ekleyin
#    - colcon build ile yeniden derleyin
#
# 2. EĞER YENİ BİR C++ NODE EKLİYORSANIZ:
#    - src/ klasörü oluşturun
#    - add_executable() ve install(TARGETS...) komutları ekleyin
#    - Örnek:
#    # add_executable(my_node src/my_node.cpp)
#    # ament_target_dependencies(my_node rclcpp std_msgs)
#    # install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})
#
# 3. EĞER YENİ BİR PYTHON NODE EKLİYORSANIZ:
#    - scripts/ klasörü oluşturun
#    - install(PROGRAMS...) komutu ekleyin
#    - Örnek:
#    # install(PROGRAMS
#    #   scripts/my_python_node.py
#    #   DESTINATION lib/${PROJECT_NAME}
#    # )
#
# 4. EĞER YENİ BAĞIMLILIK EKLİYORSANIZ:
#    - package.xml'e <depend> veya <exec_depend> ekleyin
#    - Eğer C++ kodunda kullanılıyorsa:
#      find_package(new_package REQUIRED) ekleyin
# ========================================

ament_package()