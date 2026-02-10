# Global Bringup

Global bringup, tüm sistemin merkezi başlatma paketidir. Autoware ve diğer tüm modülleri tek bir noktadan başlatmayı sağlar.

## 📋 İçindekiler

- [Paket Nedir?](#paket-nedir)
- [Dosya Yapısı](#dosya-yapısı)
- [Nasıl Kullanılır?](#nasıl-kullanılır)
- [Yeni Paket/Modül Ekleme](#yeni-paketmodül-ekleme)
- [Konfigürasyon Dosyaları](#konfigürasyon-dosyaları)
- [Örnekler](#örnekler)

---

## 📦 Paket Nedir?

`global_bringup` paketi, sisteminizin tüm bileşenlerini tek bir komutla başlatmanızı sağlayan merkezi bir launch sistemidir. 

### Ana Özellikler:
- ✅ Tek komutla tüm sistemi başlatma
- ✅ YAML dosyaları ile kolay konfigürasyon
- ✅ Modüler yapı - istediğiniz paketi açıp kapatabilme
- ✅ Autoware entegrasyonu
- ✅ Kolay genişletilebilirlik

---

## 📁 Dosya Yapısı

```
global_bringup/
├── CMakeLists.txt          # ROS2 build konfigürasyonu
├── package.xml             # Paket bağımlılıkları ve meta bilgiler
├── README.md               # Bu dosya
├── config/                 # Konfigürasyon dosyaları
│   ├── autoware_args.yaml  # Autoware parametreleri
│   ├── bringup.yaml        # Hangi paketlerin başlatılacağı
│   ├── frames.yaml         # TF frame konfigürasyonu
│   └── topics.yaml         # Topic remapping
└── launch/
    ├── global.launch.py    # Ana launch dosyası
    └── includes/           # Her paket için ayrı launch dosyaları (opsiyonel)
        └── (paket_launch_dosyaları.launch.py)
```

---

## 🚀 Nasıl Kullanılır?

### 1. Paketi Derleyin

```bash
cd ~/ros2_ws  # veya workspace'inizin kök dizini
colcon build --packages-select global_bringup
source install/setup.bash
```

### 2. Sistemi Başlatın

```bash
ros2 launch global_bringup global.launch.py
```

### 3. Konfigürasyonu Değiştirin

Sistemi başlatmadan önce konfigürasyon dosyalarını ihtiyacınıza göre düzenleyin:

```bash
cd ~/ros2_ws/src/alaz/modules/global_bringup/config
nano autoware_args.yaml  # Autoware ayarlarını düzenle
nano bringup.yaml        # Paket ayarlarını düzenle
```

---

## ➕ Yeni Paket/Modül Ekleme

Yeni bir paketi sisteme entegre etmek için 3 basit adım:

### Adım 1: `bringup.yaml` Dosyasını Düzenleyin

`config/bringup.yaml` dosyasına yeni paketinizi ekleyin:

```yaml
packages:
  # Yeni paketiniz
  my_package:
    enabled: true  # true: başlat, false: başlatma
    launch_file: "my_package.launch.py"
    arguments:
      param1: "value1"
      param2: "value2"
```

### Adım 2: Launch Dosyası Oluşturun (Opsiyonel)

Eğer paketiniz için özel bir wrapper launch dosyası oluşturmak istiyorsanız:

```bash
touch launch/includes/my_package.launch.py
chmod +x launch/includes/my_package.launch.py
```

`launch/includes/my_package.launch.py` içeriği:

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch argümanlarını al
    param1 = LaunchConfiguration('param1', default='default_value')
    param2 = LaunchConfiguration('param2', default='default_value')
    
    # Paket launch dosyasını bul
    pkg_share = get_package_share_directory('my_package')
    pkg_launch = os.path.join(pkg_share, 'launch', 'main.launch.py')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(pkg_launch),
            launch_arguments={
                'param1': param1,
                'param2': param2,
            }.items()
        )
    ])
```

### Adım 3: `global.launch.py` Dosyasını Güncelleyin

`launch/global.launch.py` dosyasındaki `# ========== PACKAGES ==========` bölümüne yeni paketinizi ekleyin:

```python
# ========== PACKAGES ==========

# my_package'i başlat
if packages_cfg.get("packages", {}).get("my_package", {}).get("enabled", False):
    pkg_cfg = packages_cfg["packages"]["my_package"]
    pkg_launch = os.path.join(includes_dir, pkg_cfg.get("launch_file", "my_package.launch.py"))
    if os.path.exists(pkg_launch):
        pkg_args = pkg_cfg.get("arguments", {})
        actions.append(IncludeLaunchDescription(
            AnyLaunchDescriptionSource(pkg_launch),
            launch_arguments=pkg_args.items(),
        ))
```

### Adım 4: Bağımlılıkları Ekleyin

Eğer yeni paketiniz bir ROS2 bağımlılığı gerektiriyorsa, `package.xml` dosyasına ekleyin:

```xml
<!-- Sadece çalışma zamanında gerekli -->
<exec_depend>my_package</exec_depend>

<!-- Hem derleme hem çalışma zamanında gerekli -->
<depend>my_package</depend>
```

### Adım 5: Yeniden Derleyin ve Başlatın

```bash
colcon build --packages-select global_bringup
source install/setup.bash
ros2 launch global_bringup global.launch.py
```

---

## ⚙️ Konfigürasyon Dosyaları

### `autoware_args.yaml`

Autoware'in başlatma parametrelerini içerir.

```yaml
autoware:
  enabled: true              # Autoware'i başlat/başlatma
  use_sim_time: false        # Simülasyon zamanı kullan
  pose_source: "yabloc"      # Lokalizasyon kaynağı: yabloc, ndt, ekf
  map_path: "/path/to/map"   # Harita dizini
  vehicle_model: "sample_vehicle"  # Araç modeli
  sensor_model: "sample_sensor"    # Sensör modeli
```

**Ne zaman düzenlenir?**
- Autoware'i açıp kapatmak için
- Harita yolu değiştiğinde
- Araç veya sensör modeli değiştiğinde
- Lokalizasyon yöntemi değiştiğinde

---

### `bringup.yaml`

Hangi paketlerin başlatılacağını belirler.

```yaml
packages:
  my_sensor_driver:
    enabled: true
    launch_file: "sensor_driver.launch.py"
    arguments:
      device: "/dev/ttyUSB0"
      baud_rate: "115200"
  
  my_perception:
    enabled: false  # Bu paket başlatılmayacak
    launch_file: "perception.launch.py"
    arguments:
      model_path: "/path/to/model"
```

**Ne zaman düzenlenir?**
- Yeni paket eklerken
- Bir paketi geçici olarak devre dışı bırakmak için
- Paket parametrelerini değiştirirken

---

### `frames.yaml`

TF frame ID'lerini yapılandırır.

```yaml
frames:
  map_frame: "map"
  base_link_frame: "base_link"
  sensor_frame: "velodyne_top"
  camera_frame: "camera_front"
```

**Ne zaman düzenlenir?**
- Sensör frame isimleri değiştiğinde
- Yeni sensör eklendiğinde
- TF ağacı yapısı değiştiğinde

---

### `topics.yaml`

Topic isimlerini yeniden eşleştirir (remapping).

```yaml
topics:
  # Input topics
  input_pointcloud: "/sensing/lidar/concatenated/pointcloud"
  input_image: "/sensing/camera/front/image_raw"
  
  # Output topics
  output_odometry: "/localization/kinematic_state"
  output_pose: "/localization/pose_estimator/pose"
```

**Ne zaman düzenlenir?**
- Topic isimleri standardize edilirken
- Farklı paketler arasında topic bağlantısı kurarken
- Başka bir sistemle entegrasyon yaparken

---

## 📚 Örnekler

### Örnek 1: Sadece Autoware Başlatma

`config/bringup.yaml`:
```yaml
packages: {}  # Boş - sadece Autoware başlatılacak
```

`config/autoware_args.yaml`:
```yaml
autoware:
  enabled: true
  use_sim_time: false
  pose_source: "yabloc"
  map_path: "/home/user/maps/my_map"
  vehicle_model: "sample_vehicle"
  sensor_model: "sample_sensor"
```

Çalıştır:
```bash
ros2 launch global_bringup global.launch.py
```

---

### Örnek 2: Autoware + Sensör Sürücüleri

`config/bringup.yaml`:
```yaml
packages:
  lidar_driver:
    enabled: true
    launch_file: "velodyne.launch.py"
    arguments:
      device_ip: "192.168.1.201"
      port: "2368"
  
  camera_driver:
    enabled: true
    launch_file: "camera.launch.py"
    arguments:
      device: "/dev/video0"
      fps: "30"
```

---

### Örnek 3: Bir Paketi Geçici Olarak Devre Dışı Bırakma

Eğer bir pakette sorun varsa veya test etmek istemiyorsanız:

```yaml
packages:
  lidar_driver:
    enabled: false  # ❌ Başlatılmayacak
    launch_file: "velodyne.launch.py"
  
  camera_driver:
    enabled: true  # ✅ Başlatılacak
    launch_file: "camera.launch.py"
```

---

## 🔧 Gelişmiş Kullanım

### CMakeLists.txt'de Değişiklik

**Ne zaman gerekir?**
- Yeni C++ node eklediğinizde
- Yeni Python script eklediğinizde
- Yeni bağımlılık eklediğinizde (C++ için)

**Örnek: C++ Node Ekleme**
```cmake
# CMakeLists.txt içine ekleyin
find_package(rclcpp REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs)
install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})
```

---

### package.xml'de Değişiklik

**Ne zaman gerekir?**
- Yeni ROS2 paketi kullanacağınızda
- Python kütüphanesi gerektiğinde
- Build/runtime bağımlılığı eklerken

**Bağımlılık Türleri:**
| Tür | Ne Zaman Kullanılır | Örnek |
|-----|---------------------|-------|
| `<depend>` | Hem build hem runtime | `<depend>sensor_msgs</depend>` |
| `<build_depend>` | Sadece derleme zamanı | `<build_depend>rosidl_generator_cpp</build_depend>` |
| `<exec_depend>` | Sadece çalışma zamanı | `<exec_depend>python3-numpy</exec_depend>` |
| `<test_depend>` | Sadece test zamanı | `<test_depend>ament_cmake_pytest</test_depend>` |

---

## ❓ Sık Sorulan Sorular

### Paket derlenmiyor, ne yapmalıyım?

```bash
# Önce temiz build deneyin
rm -rf build/ install/ log/
colcon build --packages-select global_bringup --symlink-install
source install/setup.bash
```

### Konfigürasyon değişikliklerimi nasıl test edebilirim?

```bash
# Sadece syntax kontrolü
python3 -c "import yaml; yaml.safe_load(open('config/bringup.yaml'))"

# Launch dosyasını test et (başlatmadan)
ros2 launch global_bringup global.launch.py --show-args
```

### Hangi paketlerin başlatıldığını nasıl görebilirim?

```bash
# Çalışan node'ları listele
ros2 node list

# Aktif topic'leri listele
ros2 topic list
```

---

## 📝 Notlar

- ⚠️ **YAML Dosyaları:** Girinti (indentation) önemlidir! 2 veya 4 boşluk kullanın, tab kullanmayın.
- 🔄 **Değişiklikler:** Config dosyalarındaki değişiklikler için yeniden build gerekmez, sadece launch'ı yeniden başlatın.
- 🗂️ **Launch Dosyaları:** Launch dosyalarında değişiklik yaptıysanız `colcon build` yapın.
- 🔗 **Bağımlılıklar:** Yeni bağımlılık ekledikten sonra mutlaka `colcon build` çalıştırın.

---

## 📞 Destek

Sorun yaşarsanız:
1. Log dosyalarını kontrol edin: `~/.ros/log/`
2. Verbose mode ile çalıştırın: `ros2 launch global_bringup global.launch.py --debug`
3. ROS2 ortamının doğru source edildiğinden emin olun

---

**Son Güncelleme:** 26 Ocak 2026
