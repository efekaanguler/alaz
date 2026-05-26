# 🚗 SDC 2026 — Perception Module

## Architecture

```
┌─────────────────────────────── PERCEPTION ──────────────────────────────┐
│                                                                         │
│  ┌─────────┐    ┌──────────┐    ┌───────────┐    ┌──────────────────┐   │
│  │ Camera  │───▶│ YOLOv8   │───▶│ ByteTrack │───▶│ Autoware Bridge  │   │
│  │ (Image) │    │ Detector │    │ Tracker   │    │                  │   │
│  └─────────┘    │ (rois0)  │    │ (tracked  │    │ DetectedObjects  │──▶ Obstacle Avoidance
│                 └──────────┘    │  rois0)   │    │ TrackedObjects   │──▶ Behavior Velocity
│                                 └───────────┘    │ PredictedObjects │──▶ Crosswalk
│                                                  └──────────────────┘   │
│  ┌─────────┐    ┌──────────┐    ┌───────────┐    ┌──────────────────┐   │
│  │ Camera  │───▶│ YOLOv8   │───▶│ TL        │───▶│ Autoware Bridge  │   │
│  │ (Image) │    │ TL ROI   │    │ Classifier│    │ TrafficLightGroup│──▶ Traffic Light Stop
│  └─────────┘    │ Detector │    │ (HSV/ONNX)│    │ Array            │   │
│                 └──────────┘    └───────────┘    └──────────────────┘   │
│                                                                         │
│  ┌─────────┐    ┌──────────┐    ┌───────────┐                           │
│  │ LiDAR   │───▶│ LaserScan│───▶│ Occupancy │──────────────────────────▶ Costmap
│  │ (/scan) │    │ → PCL    │    │ Grid      │                           │
│  └─────────┘    └──────────┘    └───────────┘                           │
│                                                                         │
│  LiDAR yoksa ───▶ Dummy Lidar Publisher (fallback)                      │
└─────────────────────────────────────────────────────────────────────────┘
```

## Autoware Planner Topics

| Topic | Format | Kullanan Modül |
|---|---|---|
| `/perception/object_recognition/detection/objects` | `DetectedObjects` | Obstacle Avoidance, Crosswalk |
| `/perception/object_recognition/tracking/objects` | `TrackedObjects` | Behavior Velocity |
| `/perception/object_recognition/objects` | `PredictedObjects` | Behavior Path Planner |
| `/perception/traffic_light_recognition/traffic_signals` | `TrafficLightGroupArray` | Traffic Light Stop |

## Dosya Yapısı

```
modules/perception/
├── scripts/
│   ├── webcam_detect_standalone.py    # Mac'te standalone detection + TL (webcam)
│   ├── dummy_lidar_publisher.py       # Sahte LaserScan (lidar yokken)
│   ├── dummy_camera_publisher.py      # Sahte kamera (test için)
│   ├── mjpeg_to_ros.py                # Mac webcam → ROS Image bridge
│   ├── visualize_detections.py        # Detection görüntüleyici
│   ├── mac_docker_start.sh            # Mac'te test icin docker baslatici
│   ├── test_detection_e2e.sh          # E2E test
│   ├── test_planner_integration.sh    # Planner entegrasyon testi
│   ├── test_perception_pipeline.sh    # Pipeline testi
│   ├── test_sensor_fusion_autoware.py # Sensor fusion + Autoware testi
│   └── comprehensive_test.py          # 90 testlik kapsamlı test
├── models/
│   ├── yolov8n.onnx                   # YOLOv8 Nano model (12 MB)
│   ├── labels.txt                     # COCO 80 sınıf
│   └── color_map.json                 # Renk haritası
├── launch/
│   ├── perception.launch.xml          # Ana ROS 2 launch
│   └── laserscan_to_pcl_and_occ.launch.xml  # LaserScan → Costmap
├── detection_ws/src/                  # Autoware ROS 2 paketleri
│   ├── autoware_tensorrt_yolox/       #   YOLOv8 detector node (435 satır)
│   ├── autoware_bytetrack/            #   ByteTrack tracker (376 satır)
│   ├── autoware_traffic_light_classifier/  # TL classifier (331 satır)
│   ├── autoware_detection_autoware_bridge/ # Detection → Autoware format (346 satır)
│   └── tier4_perception_launch/       #   Master launch (300 satır)
├── run.sh                             # Hızlı başlatıcı
├── package.xml
├── CMakeLists.txt
└── README.md                          # Bu dosya
```

---

## 🖥️ Mac'te Kullanım (Geliştirme / Test)

### 1. Standalone Detection (en basit — sadece webcam + algılama)

```bash
cd ~/alaz/modules/perception

# İlk kurulum
python3 -m venv venv
source venv/bin/activate
pip install opencv-python onnxruntime numpy

# Çalıştır (webcam açılır, tespit gösterir, q ile çık)
python3 scripts/webcam_detect_standalone.py
```

Parametreler:
```bash
python3 scripts/webcam_detect_standalone.py --score-thr 0.35   # eşik ayarı
python3 scripts/webcam_detect_standalone.py --device 1          # farklı kamera
python3 scripts/webcam_detect_standalone.py --no-tl             # TL algılamasız
```

### 2. Docker Pipeline (Mac + Docker = tam Autoware pipeline)

```bash
# 1. Docker Desktop'u aç
# 2. Pipeline'ı başlat (lidar yoksa otomatik dummy başlar)
./scripts/mac_docker_start.sh

# Sadece sahte lidar (Dummy Lidar) ile test etmek istersen:
./scripts/mac_docker_start.sh --dummy-lidar

# Lidar olmadan (sadece kamera) test etmek istersen:
./scripts/mac_docker_start.sh --no-lidar

# Gorsellestirmeyi (viz) kapatmak istersen (daha hizli calisir):
./scripts/mac_docker_start.sh --no-viz
```

Pipeline otomatik olarak:
1. Mac webcam'i MJPEG stream olarak yayınlar
2. Docker içinde MJPEG → ROS Image bridge'i başlatır
3. `/scan` topic'ini kontrol eder → varsa gerçek LiDAR, yoksa dummy
4. YOLOv8 → ByteTrack → Autoware Bridge zincirini başlatır
5. Trafik ışığı ROI detector → TL classifier → TrafficLightGroupArray

### 3. Testleri Çalıştır

```bash
# Kapsamlı test (90 test)
python3 test_scripts/comprehensive_test.py

# Spesifik testler:
python3 test_scripts/test_sensor_fusion_autoware.py
python3 scripts/mjpeg_to_ros.py  # Doğrudan kamerayı test etmek için

bash test_scripts/test_perception_pipeline.sh --local
```

---

## 🚀 NUC'da Kullanım (Yarışma Günü — Docker İçinde)

> **Sensörler zaten Docker'da açık olacak** (arkadaşlar sensör launch'unu halletti).
> Perception tarafında **tek bir script** çalıştırmak yeterli.

### Yarışma Günü — Tek Komut

```bash
# Docker container'ın içinde:
bash /workspace/modules/perception/scripts/nuc_docker_perception.sh
```

**Bu kadar.** Script otomatik olarak:
1. ✅ `/scan` topic'ini arar → bulursa gerçek 2D LiDAR kullanır, yoksa dummy başlatır
2. ✅ **LaserScan → PointCloud2** dönüşümü yapar (ara adım — ikisini de besler)
3. ✅ **Paralel yol A:** PointCloud2 → Clustering → **ROI Cluster Fusion** (kamera ROI'leri ile)
4. ✅ **Paralel yol B:** PointCloud2 → **OccupancyGrid** → Costmap (engel mesafesi)
5. ✅ YOLOv8 → ByteTrack → Autoware Bridge (nesne tespiti + sınıflandırma)
6. ✅ Trafik ışığı algılama + sınıflandırma başlatır
7. ✅ `detection_ws` build edilmediyse otomatik `colcon build` çalıştırır

### Opsiyonel Flaglar

```bash
# 2 kamera ile:
bash nuc_docker_perception.sh --cameras=2

# Sadece kamera, LiDAR fusion'sız:
bash nuc_docker_perception.sh --no-fusion

# Debug görselleştirme açık:
bash nuc_docker_perception.sh --viz

# Help:
bash nuc_docker_perception.sh --help
```

### İlk Kurulum (NUC'da bir kez — sadece build)

```bash
# Docker'ın içinde:
cd /workspace/modules/perception/detection_ws
colcon build --symlink-install
```
> Script bunu otomatik yapar ama ilk sefer uzun sürer. Önceden yapılması önerilir.

### Kontrol ve Debug

```bash
# Topic'ler çalışıyor mu?
ros2 topic list | grep perception

# Detection çıktısı (tek mesaj)
ros2 topic echo /perception/object_recognition/detection/objects --once

# Trafik ışığı
ros2 topic echo /perception/traffic_light_recognition/traffic_signals --once

# Hz kontrolü (en az 5-10 Hz olmalı)
ros2 topic hz /perception/object_recognition/detection/objects

# LiDAR → Costmap (PointCloud2 çıkıyor mu?)
ros2 topic hz /perception/obstacle/pointcloud

# LaserScan ham veri
ros2 topic echo /scan --once
```

---

## Sensör Fusion Stratejisi (2D LiDAR + Camera)

Bizim LiDAR **2D** (LaserScan). Önce PointCloud2'ye dönüştürülür,
sonra **paralel** olarak hem ROI fusion'a hem costmap'e beslenir:

```
                                         ┌─ Euclidean Clustering ─┐
                                         │                        ├─ ROI Cluster Fusion → Fused Objects
2D LiDAR → LaserScan → PointCloud2 ─────┤  Camera → YOLOv8 ROIs ─┘
                                         │
                                         └─ OccupancyGrid → Costmap → Planner

Camera → YOLOv8 → ByteTrack → Bridge → DetectedObjects  ──→ Planner
                                      → TrackedObjects   ──→ Planner
                                      → PredictedObjects  ──→ Planner

Camera → YOLOv8 TL → TL Classifier → TrafficLightGroupArray → Planner
```

**3 paralel yol:**
1. **ROI Cluster Fusion**: PointCloud2 → clustering → 2D ROI'lerle IoU eşleme → fused objects
2. **Costmap**: PointCloud2 → OccupancyGrid → planner engel haritası
3. **Camera detection**: YOLOv8 → ByteTrack → Autoware Bridge → planner nesne tespiti

> **LaserScan → PointCloud2 dönüşümü her zaman gerekli** (ara adım).
> Script bunu `laserscan_to_pointcloud` ile otomatik yapar.

---

## ByteTrack Tracker (2D ROI Tracking)

YOLOv8'in ürettiği ROI'ler **ByteTrack** ile frame'ler arası takip edilir:

```
YOLOv8 → rois0 → ByteTrack → tracked_rois0 → Bridge → Autoware
                      │
                      └─ İki aşamalı IoU eşleme:
                         1. Yüksek skor (>0.5) → mevcut track'lerle eşle
                         2. Düşük skor (0.1-0.5) → eşlenememiş track'lerle eşle
```

| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `track_high_thresh` | 0.5 | Yüksek güven eşiği |
| `track_low_thresh` | 0.1 | Düşük güven eşiği |
| `new_track_thresh` | 0.6 | Yeni track oluşturma eşiği |
| `match_thresh` | 0.7 | IoU eşleme eşiği |
| `track_buffer` | 30 | Kayıp track silme süresi (frame) |

---

## COCO → Autoware Sınıf Eşlemesi

| COCO ID | COCO Adı | Autoware Label | Planner Kullanımı |
|---|---|---|---|
| 0 | person | PEDESTRIAN (7) | Crosswalk durdurma |
| 1 | bicycle | BICYCLE (6) | Yol paylaşımı |
| 2 | car | CAR (1) | Obstacle avoidance |
| 3 | motorbike | MOTORCYCLE (5) | Obstacle avoidance |
| 5 | bus | BUS (3) | Obstacle avoidance |
| 7 | truck | TRUCK (2) | Obstacle avoidance |

## Trafik Işığı → Planner

| Renk | Autoware Color | Planner Kararı |
|---|---|---|
| 🔴 red | TL_COLOR_RED (1) | **STOP** |
| 🟡 yellow | TL_COLOR_AMBER (2) | **CAUTION** |
| 🟢 green | TL_COLOR_GREEN (3) | **PROCEED** |

---

## Model İndirme

YOLOv8n model (12 MB) → `models/yolov8n.onnx`:

```bash
pip install ultralytics
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='onnx', opset=17)"
mv yolov8n.onnx ~/alaz/modules/perception/models/
```
