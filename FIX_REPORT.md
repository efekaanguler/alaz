# ege_alaz - Gün Sonu Fix Raporu

Bu çalışma aracın kalkmasını engelleyebilecek Autoware/ROS 2 entegrasyon kopukluklarını temizlemek için yapıldı. Repo Autoware Universe/Humble hattına göre kontrol edildi ve Docker içinde build doğrulaması alındı.

## Kapatılan Kritik Sorunlar

- Control launch yanlış executable çağırıyordu. Autoware image içinde `autoware_trajectory_follower_node` executable adı `controller_node_exe`; repo eski/olmayan adı çağırıyordu. Ayrıca executable vermeyen `autoware_mpc_lateral_controller` node'u ve dependency'si kaldırıldı. (`modules/control/launch/control.launch.py`, `modules/control/CMakeLists.txt`, `modules/control/package.xml`)
- TF hattında sensör static transform'ları namespace altında kalma riski taşıyordu. `robot_state_publisher` artık global `/tf` ve `/tf_static` yayınlarına remap ediliyor. (`modules/sensor/rdw_sensor_kit_launch/launch/sensing.launch.xml`)
- Odometry `/odom` yayınlıyordu ama `odom -> base_link` TF üretmiyordu. TF broadcaster eklendi ve Autoware vehicle status topiclerinden hız/direksiyon okuyacak hat güçlendirildi. (`modules/odometry/include/odometry_node.hpp`, `modules/odometry/src/odometry_node.cpp`, `modules/odometry/CMakeLists.txt`, `modules/odometry/package.xml`)
- Mission Control topic sözleşmeleri ana repo standardıyla hizalandı: trajectory topic `/planning/scenario_planning/trajectory`, emergency topic `/mission_control/emergency_stop`. Legacy `/autoware/engage` korunurken Autoware AD API operation-mode servis çağrısı da eklendi. (`modules/mission_control/include/mission_control/mode_run.hpp`, `modules/mission_control/src/mode_run.cpp`, `modules/mission_control/include/mission_control/mode_park.hpp`, `modules/mission_control/src/mode_park.cpp`)
- StartMode lokalizasyonu mesaj gelmeden otomatik başarılı sayıyordu. Bu davranış düzeltildi; localization state geldiyse gerçek durum kontrol ediliyor, gelmiyorsa sensör/odometry kontrolleriyle devam ediliyor. (`modules/mission_control/include/mission_control/mode_start.hpp`, `modules/mission_control/src/mode_start.cpp`)
- `my_vehicle_interface` ile `ros2_can_bridge` ayrımı netleştirildi: vehicle interface Autoware control command'larını CAN frame'e çevirir, Python CAN bridge sadece `/to_can_bus` ve `/from_can_bus` taşır. (`modules/vehicle/my_vehicle_interface/src/vehicle_interface_node.cpp`, `modules/ros2_can_bridge/ros2_can_bridge/bridge_node.py`)
- Eski top-level `modules/detection` hattı kaldırıldı; aktif algılama hattı `modules/perception` altında kaldı. Build/install/log çıktıları ve geçici dosyalar ignore kapsamına alındı. (`modules/detection/`, `modules/perception/`, `.gitignore`, `build_pkgs.sh`, `clean_pkgs.sh`)

## Hala Donanım Ekibinden Gerekenler

- Yeni araca ait DBC/CAN matrix: ID, DLC, endian, scale/offset, min/max, checksum/counter, heartbeat/watchdog ve safe-state kuralları.
- Steering, throttle, brake, gear, e-stop, autonomous/manual mode ve fault feedback mesajları.
- Gerçek araç geometri ölçüleri: wheelbase, wheel radius, steering ratio, max steering angle, gövde ölçüleri ve `base_link` referansı.
- Kamera/lidar/IMU/GNSS device path, calibration, extrinsic ve timestamp davranışı.

Bu bilgiler gelmeden `can_utils.cpp` içindeki eski araç CAN şemasını yeni araca uydurmak doğru olmaz. (`modules/vehicle/my_vehicle_interface/src/can_utils.cpp`, `modules/vehicle/my_vehicle_interface/include/my_vehicle_interface/can_utils.hpp`, `modules/vehicle/my_vehicle_interface/config/vehicle_interface.param.yaml`)

## Doğrulama

- Docker image: `autoware_backup:latest`
- Full workspace build: `17 packages finished`
- Launch parse kontrolleri geçti: `global_bringup`, `perception`, `rdw_sensor_kit_launch`, `control`
- Executable kontrolü yapıldı: `controller_node_exe`, `vehicle_interface_node`, `mission_control_launch`, `odom_launch`, `bridge_node.py`
- `git diff --check` temiz

## Push Notu

Önerilen commit mesajı:

```text
fix(integration): align Autoware control, TF, mission state and perception pipeline
```

Gerçek araçta son kabul testi için düşük hızda; önce CAN feedback, sonra `/control/command/control_cmd`, `/to_can_bus`, `/from_can_bus`, `/odom`, `/tf`, `/sensing/image_raw`, `/sensing/scan` ve `/rois0` canlı izlenmeli.
