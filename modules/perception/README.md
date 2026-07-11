# Perception

ROS 2 Humble ve Autoware Universe icin kamera ve 2D LiDAR perception paketi.

## Runtime Mimarisi

```text
/sensing/scan
  -> LaserScan to PointCloud2
  -> metric LiDAR detection
  -> Autoware multi-object tracker
  -> map-based prediction
  -> /perception/object_recognition/objects

/sensing/image_raw
  -> YOLOv8
  -> ByteTrack
  -> /rois0 (vision_msgs/Detection2DArray)

/perception/lidar/pointcloud
  -> occupancy fallback
  -> /perception/occupancy_grid
  -> /perception/obstacle/pointcloud
```

Kamera bounding box koordinatlari metre olmadigi icin 2D detection bridge'i
planner nesnesi uretmez. Planner icin 3D konumlu nesneler metric LiDAR hattindan
gelir. Kamera ROI'leri yalniz goruntu takibi ve kalibrasyonlu ROI fusion icindir.

## Baslatma

Ana sistem:

```bash
ros2 launch perception perception.launch.xml
```

Container icinde perception pipeline:

```bash
bash /workspace/modules/perception/scripts/nuc_docker_perception.sh
```

Script gercek sensor topic'lerini bekler. Sensor yoksa kendiliginden dummy veri
basmaz ve ilgili ciktilar pasif kalir.

## Software Smoke Test

Dummy sensorler yalniz acik test bayraklariyla baslatilir:

```bash
bash /workspace/modules/perception/scripts/nuc_docker_perception.sh \
  --dummy-camera --dummy-lidar --no-fusion --smoke-test --device cpu
```

Smoke testi CameraInfo, PointCloud2, OccupancyGrid ve `/rois0` uzerinde gercek
mesaj alinmadan PASS vermez.

LiDAR ROS launch testi:

```bash
colcon test --packages-select alaz_lidar_clustering
colcon test-result --verbose
```

## Deployment

- `docker/Dockerfile` CPU ONNX Runtime fallback ve ROS `vision_msgs` bagimliligini
  icerir. Jetson GPU kullanimi icin JetPack ile uyumlu provider ayrica kurulur.
- `config/nuc/camera_info_logitech_720p.yaml` yalniz fallback degeridir; yeni
  kameranin intrinsic kalibrasyonu ile degistirilmelidir.
- `base_link -> camera_center_link` ve `base_link -> lidar_link` extrinsic
  donusumleri yeni sensor montajinda olculmelidir.
- ROI fusion gercek CameraInfo, zaman senkronizasyonu ve extrinsic kalibrasyon
  dogrulanmadan saha icin hazir kabul edilmez.

## Onemli Dosyalar

- `launch/perception.launch.xml`: Autoware tracking/prediction ve AI pipeline.
- `scripts/nuc_docker_perception.sh`: sensor discovery ve runtime orchestration.
- `detection_ws/src/`: YOLOv8, ByteTrack ve traffic-light paketleri.
- `test_scripts/dummy_camera_publisher.py`: yalniz software test kamerasi.
- `test_scripts/dummy_lidar_publisher.py`: yalniz software test LiDAR'i.
