# Perception Module

Alaz otonom araç projesi için perception modülü.
2D LiDAR → Occupancy Grid üretimi (obstacle avoidance).

## Nasıl Çalışıyor?

```
/scan (LaserScan) → [occupancy_grid_map_node] → /perception/occupancy_grid_map/map (OccGrid)
                     ↑ Autoware standalone node
                     ↑ PointCloud'a dönüşüm YOK
```

## Pipeline Seçenekleri

| Pipeline | Akış | Durum |
|----------|------|-------|
| `direct` (varsayılan) | LaserScan → OccGrid | ✅ Çalışıyor |
| `pointcloud` | LaserScan → PCL → OccGrid | ⚠️ Docker'da plugin yok | olsa çalışcak kıvamda

## Kullanım

```bash
# Build
cd ~/workspace && colcon build --packages-select perception_module && source install/setup.bash

# Varsayılan (direct — önerilen)
ros2 launch perception_module perception.launch.xml

# Simülasyon
ros2 launch perception_module perception.launch.xml use_sim_time:=true

# PCL pipeline (farklı Docker image gerekebilir)
ros2 launch perception_module perception.launch.xml occ_pipeline:=pointcloud
```

## Topic'ler

| Yön | Topic | Tip |
|-----|-------|-----|
| Sub | `/scan` | sensor_msgs/LaserScan |
| Pub | `/perception/occupancy_grid_map/map` | nav_msgs/OccupancyGrid |

## Dosya Yapısı

```
perception_module/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   ├── perception.launch.xml              # Ana launch
│   └── laserscan_to_pcl_and_occ.launch.xml # Opsiyon B (PCL)
└── test/
    └── test_perception.sh                  # Test scripti
```

## Launch Argümanları

| Argüman | Varsayılan | Açıklama |
|---------|-----------|----------|
| `occ_pipeline` | `direct` | `direct` veya `pointcloud` |
| `use_sim_time` | `false` | Simülasyon zamanı |
| `input_scan_topic` | `/scan` | LiDAR topic |

## Autoware Entegrasyonu

OccupancyGrid Autoware planner'a gider:
- `obstacle_velocity_limiter` — hız sınırlama
- `behavior_path_planner` — yol planlama
- `freespace_planner` — park/manevra

## Bağımlılıklar

- autoware_probabilistic_occupancy_grid_map
- pointcloud_to_laserscan (sadece pointcloud pipeline)

## Ekip
Ege · Cengo · İzzet

## Lisans
Apache-2.0
