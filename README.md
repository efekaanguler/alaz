# ege_alaz

ROS 2 Humble ve Autoware Universe tabanli otonom arac entegrasyon workspace'i.

## Build Ve Launch

```bash
./build_pkgs.sh --clean
source source_pkgs.sh
ros2 launch global_bringup global.launch.py
```

Route, localization, sensor safety kontrolleri ve hardware autonomous-mode
feedback hazir olduktan sonra engage:

```bash
ros2 service call /system/operation_mode/change_operation_mode autoware_system_msgs/srv/ChangeOperationMode "{mode: 2}" && ros2 service call /system/operation_mode/change_autoware_control autoware_system_msgs/srv/ChangeAutowareControl "{autoware_control: true}"
```

## Sahadan Tamamlanmasi Gerekenler

1. Elektronik ekip yeni aracin DBC/CAN matrix dosyasini vermeli. ID, DLC,
   endian, scale/offset, unit, min/max, counter/CRC, timeout, autonomous-mode
   request/feedback, e-stop ve fault alanlari eksiksiz olmali.
2. `modules/vehicle/my_vehicle_interface/config/vehicle_interface.param.yaml`
   yeni protokole gore doldurulmali. Hardware mode handshake tamamlanmadan
   `can_command_output_enabled` acilmamali.
3. Wheelbase, wheel radius, steering ratio, maksimum steering angle ve govde
   olculeri `modules/vehicle/rdw_vehicle_description/config/vehicle_info.param.yaml`
   icinde gercek aracla dogrulanmali.
4. Kamera intrinsics, sensor extrinsics ve frame adlari yeni montajda yeniden
   kalibre edilmeli. Sonuclar `modules/sensor/rdw_sensor_kit_description/config/`
   altindaki kalibrasyona islenmeli.
5. YabLoc modeli `/autoware_data/yabloc_pose_initializer/` altina kurulup saha
   kamerasi ile localization initialization testi yapilmali.
6. `maps/map.osm` icindeki eksik traffic-light govde geometrileri duzeltilmeli.
   Regulatory element `1157` ve `1218` yalniz `light_bulbs` iceriyor; Lanelet2
   formatinin zorunlu `refers -> type=traffic_light` geometrisi eksik.
7. Jetson'da kamera/LiDAR device passthrough, CUDA provider, inference FPS,
   sicaklik ve uzun sureli mesaj gecikmesi gercek sensorlerle olculmeli.
8. `sllidar_ros2` Humble arm64 binary paketi bulunmadigi icin Jetson image'inda
   uyumlu surum kaynaktan derlenmeli ve `/dev/sllidar` udev kurali dogrulanmali.

Kod degisiklikleri, testler ve calistirma adimlari `README_FIX.md` dosyasindadir.
