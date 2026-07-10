# ege_alaz - Next Steps

Bu repo yazilim zincirini toparlanmis halde tutar. Bundan sonraki hedef, gercek aracta hareketi garantiye almak icin eksik donanim bilgilerini almak ve modulleri sirayla sahada dogrulamaktir.

## 1. Elektronik Ekibinden Alinacaklar

`my_vehicle_interface` ana vehicle interface katmanidir. `ros2_can_bridge` sadece CAN frame tasimalidir.

Elektronik ekipten gereken bilgiler:

- DBC dosyasi veya CAN matrix tablosu.
- CAN interface adi, bitrate, standard/extended ID ve Classical CAN/CAN-FD bilgisi.
- Steering, throttle, brake, gear, enable, heartbeat mesaj ID'leri.
- Her mesaj icin DLC, byte layout, endian, scale, offset, min/max ve unit bilgisi.
- Feedback mesajlari: hiz, steering angle, throttle/brake feedback, gear, e-stop, fault, autonomous/manual mode.
- Watchdog timeout, counter, checksum veya CRC kurali.
- Komut kesilince ECU'nun safe-state davranisi.

Guncellenecek ana dosyalar:

- `modules/vehicle/my_vehicle_interface/src/can_utils.cpp`
- `modules/vehicle/my_vehicle_interface/include/my_vehicle_interface/can_utils.hpp`
- `modules/vehicle/my_vehicle_interface/config/vehicle_interface.param.yaml`
- `modules/vehicle/my_vehicle_interface/src/vehicle_interface_node.cpp`

## 2. Arac Parametreleri

Mekanik ekipten olculmeden eski arac degerleri kullanilmamali.

Gereken bilgiler:

- Wheelbase.
- Wheel radius veya lastik capi.
- Steering ratio.
- Maksimum teker direksiyon acisi.
- Maksimum hiz, ivme ve frenleme limitleri.
- Arac uzunluk, genislik, yukseklik.
- `base_link` referans noktasi.

Guncellenecek ana dosyalar:

- `modules/vehicle/rdw_vehicle_description/config/vehicle_info.param.yaml`
- `modules/odometry/src/odometry_node.cpp`
- `modules/control/launch/control.launch.py`

## 3. Sensor ve Kalibrasyon

Gercek perception basarisi icin kamera/lidar/imu/gnss bilgileri yeni aracta tekrar dogrulanmali.

Gereken bilgiler:

- Kamera modeli, device path, cozunurluk, FPS, pixel format.
- Kamera intrinsics ve distortion parametreleri.
- Lidar modeli, port, baudrate, 2D scan veya 3D pointcloud bilgisi.
- IMU/GNSS varsa topic, frame, baudrate, covariance ve timestamp davranisi.
- Kamera/lidar/imu icin `base_link`e gore x/y/z ve roll/pitch/yaw extrinsic olculeri.

Guncellenecek ana dosyalar:

- `modules/sensor/rdw_sensor_kit_launch/launch/sensing.launch.xml`
- `modules/sensor/rdw_sensor_kit_launch/launch/camera.launch.xml`
- `modules/sensor/rdw_sensor_kit_launch/launch/lidar.launch.xml`
- `modules/sensor/rdw_sensor_kit_launch/config/logitech_720p_camera_info.yaml`
- `modules/sensor/rdw_sensor_kit_description/config/sensor_kit_calibration.yaml`
- `modules/global_bringup/config/topics.yaml`

## 4. Perception Sahada Dogrulanacaklar

Aktif hat `modules/perception` altindadir. Eski top-level `modules/detection` kaldirildi. Modeller `modules/perception/models` altindan okunur.

Sahada kontrol:

- Jetson/NUC image icinde ROS 2, Autoware ve perception bagimliliklari var mi?
- Jetson kullanilacaksa NVIDIA Container Runtime, CUDA/TensorRT ve device passthrough hazir mi?
- ONNX Runtime CPU yeterli FPS veriyor mu, yoksa TensorRT engine gerekli mi?
- Kamera `/sensing/image_raw` yayinliyor mu?
- CameraInfo `/sensing/camera_info` yayinliyor mu veya fallback dogru mu?
- Lidar `/sensing/scan` veya `/points_raw` yayinliyor mu?
- `/rois0`, `/perception/object_recognition/detection/objects`, `/perception/occupancy_grid` canli mi?
- ROI fusion icin kamera/lidar extrinsic dogru mu?

Temel komutlar:

```bash
ros2 topic hz /sensing/image_raw
ros2 topic echo /sensing/camera_info --once
ros2 topic hz /sensing/scan
ros2 topic hz /rois0
ros2 topic echo /perception/object_recognition/detection/objects --once
```

## 5. Araci Yurutme Sirasi

1. CAN hattinda pasif `candump can0` logu al.
2. Feedback mesajlarini decode edip hiz, steering, gear ve fault topiclerini dogrula.
3. Tekerler yerde degilken dusuk limitli steering/throttle/brake komutlarini test et.
4. `/to_can_bus` ve `/from_can_bus` topiclerini ROS tarafinda izle.
5. Odometry `/odom` ciktisini gercek teker/hiz verisiyle karsilastir.
6. Kamera/lidar topiclerini ve TF agacini dogrula.
7. Perception smoke testini gercek sensorlerle calistir.
8. Mission Control run/emergency gecislerini dusuk hizda test et.
9. Ilk hareket testini e-stop hazir, dusuk hiz ve limitli komutlarla yap.

## 6. Push Oncesi Temiz Kontrol

```bash
git status --short
git diff --check
```

Commit'e alinmamasi gerekenler:

- `build/`, `install/`, `log/`
- `.DS_Store`
- `.venv/`, `venv/`
- `modules/sensor/ros2_v4l2_camera/`
- `modules/sensor/sllidar_ros2/`

Son Docker dogrulamasi:

```bash
MAKEFLAGS=-j1 colcon build --base-paths modules --build-base build --install-base install --symlink-install --executor sequential
```

Bu dogrulamada 17 ROS paketi build aldi. Control launch Autoware Universe icindeki `controller_node_exe` ile hizalandi; sensor TF'leri global `/tf` ve `/tf_static` hattina, odometry de `odom -> base_link` TF yayina cekildi.

## Kisa Sonuc

Kod mimarisi `my_vehicle_interface + ros2_can_bridge + perception` ayrimina cekildi. Aracin yurumesi icin artik kritik is, elektronik CAN protokolu ve yeni arac sensor/geometry kalibrasyonlarini dogru dosyalara isleyip sahada dusuk riskli test sirasi ile dogrulamaktir.
