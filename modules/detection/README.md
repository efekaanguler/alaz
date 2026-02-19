# Detection Module (Docker Sonrasi Kisa Runbook)

Bu modulu Docker icinde ayaga kaldirdiginda:
- Kamera girdisini alir (`/sensing/camera/camera0/image_raw`)
- Detection + tracking + traffic light classifier calisir
- Ciktilari **Autoware-native topiclere** bridge eder

## 1) Terminal 1 - Build + Launch

```bash
export BASE=/workspace/modules/detection
source /opt/ros/humble/setup.bash

cd ${BASE}
bash scripts/install_dependencies.sh
bash scripts/download_or_export_model.sh

cd ${BASE}/detection_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch tier4_perception_launch perception_with_traffic_light.launch.xml \
  image_number:=1 \
  use_bytetrack:=true \
  enable_traffic_light_classifier:=true \
  traffic_light_detector/class_allowlist:=9, \
  camera_2d_detector/model_path:=${BASE}/models/yolov8n.onnx \
  camera_2d_detector/label_path:=${BASE}/models/labels.txt \
  camera_2d_detector/color_map_path:=${BASE}/models/color_map.json \
  traffic_light_classifier/model_path:=${BASE}/models/traffic_light_classifier/ped_traffic_light_classifier_mobilenetv2_batch_6.onnx \
  traffic_light_classifier/label_path:=${BASE}/models/traffic_light_classifier/lamp_labels_ped.txt
```

## 2) Terminal 2 - Kamera (Gercek yoksa Dummy)

```bash
export BASE=/workspace/modules/detection
source /opt/ros/humble/setup.bash
source ${BASE}/detection_ws/install/setup.bash

python3 ${BASE}/scripts/publish_dummy_camera.py \
  --topic /sensing/camera/camera0/image_raw \
  --width 640 \
  --height 640 \
  --fps 5
```

## 3) Terminal 3 - Kontrol

```bash
export BASE=/workspace/modules/detection
source /opt/ros/humble/setup.bash
source ${BASE}/detection_ws/install/setup.bash

ros2 topic hz /sensing/camera/camera0/image_raw

# Modulin kendi ciktilari
ros2 topic hz /rois0
ros2 topic hz /traffic_light/rois0
ros2 topic echo /classified/traffic_signals --once

# Autoware'a verilen bridge ciktilari
ros2 topic type /perception/object_recognition/detection/objects
ros2 topic type /perception/object_recognition/tracking/objects
ros2 topic type /perception/object_recognition/objects
ros2 topic type /perception/traffic_light_recognition/traffic_signals
```

## Autoware Entegrasyonunda Bu Modulin Verdigi Topicler

- `/perception/object_recognition/detection/objects` (`autoware_perception_msgs/msg/DetectedObjects`)
- `/perception/object_recognition/tracking/objects` (`autoware_perception_msgs/msg/TrackedObjects`)
- `/perception/object_recognition/objects` (`autoware_perception_msgs/msg/PredictedObjects`)
- `/perception/traffic_light_recognition/traffic_signals` (`autoware_perception_msgs/msg/TrafficLightGroupArray`)

## Sik Hata (2 satirlik)

- `No module named vision_msgs`

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-vision-msgs
```

- `BASE` bos oldugu icin `/detection_ws/install/setup.bash` bulunamiyor

```bash
export BASE=/workspace/modules/detection
```

- Eski overlay/copy kalintisi yuzunden build bozulduysa

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
cd /workspace/modules/detection/detection_ws
rm -rf build install log
colcon build --symlink-install
```
