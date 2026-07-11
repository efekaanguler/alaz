# Integration Fix And Verification

## Yapilan Duzeltmeler

- Mission Control'un `/autoware/engage` mesaj tipi ve operation-mode servisleri
  Autoware arayuzleriyle hizalandi. (`modules/mission_control/`)
- Planning, trajectory follower, shift decider, vehicle command gate ve operation
  mode manager resmi Tier4 launch zincirine baglandi. (`modules/planning/`,
  `modules/control/`)
- Vehicle interface varsayilan MANUAL ve NEUTRAL fail-safe davranisina alindi.
  Test modu disinda hardware mode feedback olmadan AUTONOMOUS kabul edilmiyor.
  (`modules/vehicle/my_vehicle_interface/`)
- Python CAN bridge yalniz `/to_can_bus` ve `/from_can_bus` frame transportu
  yapiyor. (`modules/ros2_can_bridge/`)
- Odometry Autoware velocity/steering reportlarini kullaniyor. Dinamik TF'nin tek
  sahibi localization oldugu icin odometry TF yayini varsayilan olarak kapali.
  (`modules/odometry/`)
- Kamera/LiDAR QoS profilleri sensor-data ile hizalandi. Piksel koordinatlarini
  metre gibi yayinlayan 2D->3D bridge varsayilan olarak kapatildi. Metric planner
  nesneleri LiDAR detection -> official tracker -> map prediction hattindan gelir.
  (`modules/perception/`, `modules/alaz_lidar_clustering/`)
- Camera-info launch crash'i, dummy TF zinciri ve gereksiz map reload giderildi.
  (`modules/perception/scripts/camera_info_fallback_publisher.py`,
  `modules/global_bringup/launch/includes/`, `modules/localization/`)
- Global package launch'lari scope edildi; `config_file` gibi genel argumanlarin
  vehicle, sensing, planning ve control arasinda sizmasi engellendi. Sensor
  suruculeri ve CAN transportu yazilim testinde acikca secilebilir hale getirildi.
  (`modules/global_bringup/launch/global.launch.py`,
  `modules/sensor/rdw_sensor_kit_launch/launch/sensing.launch.xml`)
- Perception gercek LiDAR yokken artik kendiliginden dummy veri basmiyor. Dummy
  kamera/LiDAR yalniz acik test bayraklariyla baslar; gercek sensor discovery icin
  startup bekleme penceresi eklendi. (`modules/perception/scripts/nuc_docker_perception.sh`)
- Eski 2D->3D bridge varsayimini, gecersiz topic'leri ve onceki sensor-kit adlarini
  test eden ad-hoc perception scriptleri paket kurulumundan ve repodan kaldirildi.
  Guncel kanit Docker smoke ve ROS launch testleridir. (`modules/perception/CMakeLists.txt`,
  `modules/perception/test_scripts/`)
- Build, install, log, rosbag ve pcap ciktilari Git disina alindi. (`.gitignore`)

## Kanitlanan Testler

- Ana workspace: 17/17 paket build PASS.
- Ana workspace: 92 test, 0 error, 0 failure (22 skip).
- Perception child workspace: 5/5 paket build PASS.
- LiDAR perception launch testi: best-effort LaserScan alindi ve metre cinsinden
  `autoware_perception_msgs/DetectedObjects` uretildi.
- Perception Docker smoke testi: dummy camera + LiDAR ile ONNX Runtime, YOLOv8,
  ByteTrack, CameraInfo, PointCloud2 ve OccupancyGrid gercek mesaj uretti.
- Control E2E: Mission Control route request -> AUTONOMOUS operation mode ->
  trajectory follower -> gate -> vehicle interface -> pozitif DRIVE ve steering
  CAN frame zinciri PASS.

```bash
./build_pkgs.sh
source source_pkgs.sh

colcon test --base-paths modules --packages-select alaz_lidar_clustering control
colcon test-result --verbose

# Donanimsiz global startup denetimi (fiziksel actuator cikisi yine kapali kalir)
ros2 launch global_bringup global.launch.py \
  launch_sensor_drivers:=false \
  can_interface:=ege_test can_channel_type:=virtual

docker build -t ege_alaz-perception:latest \
  -f modules/perception/docker/Dockerfile .
docker run --rm -v "$PWD:/workspace" -w /workspace \
  ege_alaz-perception:latest bash -lc '
    source /opt/ros/humble/setup.bash
    source /opt/autoware/setup.bash
    source /workspace/install/setup.bash
    bash modules/perception/scripts/nuc_docker_perception.sh \
      --dummy-camera --dummy-lidar --no-fusion --smoke-test --device cpu
  '
```

## Engage Akisi

1. Sensor, odometry ve localization initialization verileri canli olmali.
2. Mission Control `/planning/set_waypoint_route` servisine hedef yollar.
3. `/planning/trajectory` bos olmayan trajectory yayinlamali.
4. Mission Control system operation-mode servisleriyle AUTONOMOUS ve Autoware
   control ister.
5. Vehicle interface hardware tarafindan AUTONOMOUS mode feedback aldiktan sonra
   control komutlarini CAN'e cevirmelidir.

Software testinde 5. adim `software_test_mode=true` ile mock edilir. Bu ayar fiziksel
aracta kullanilmaz.

## Fiziksel Arac Blocker'lari

- Yeni aracin DBC/CAN matrix'i ve autonomous-mode handshake'i gelmedi. Bu nedenle
  fiziksel config'de `can_command_output_enabled=false` kalir.
- Jetson icin `sllidar_ros2` arm64 kaynaktan derlenmeli; test image'i bu harici
  donanim surucusunu icermiyor.
- Eski CAN ID ve payload degerleri yalniz software test kanitidir; yeni aracta
  dogrulanmadan kullanilamaz.
- YabLoc modeli `/autoware_data/yabloc_pose_initializer/saved_model/` altinda
  bulunmali. Model yoksa localization hazir kabul edilmez.
- `maps/map.osm` traffic-light relation `1157` ve `1218` icin gercek
  `type=traffic_light` geometry/refers kaydi harita ekibi tarafindan eklenmeli.
- Kamera intrinsics ve `base_link -> camera/lidar` extrinsics yeni montajda
  olculmeden ROI fusion saha sonucu garanti edilemez.

Jetson'da Dockerfile CPU ONNX Runtime fallback'ini saglar. GPU icin JetPack ile
uyumlu ONNX Runtime GPU/TensorRT kurulup `CUDAExecutionProvider`, FPS ve latency
gercek cihazda dogrulanmalidir.

## Commit Onerisi

```text
fix(integration): align Autoware engage, control, vehicle and perception pipelines
```
