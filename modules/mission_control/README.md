# Mission Control Modülü

Bu paket, **RDW Self-Driving Challenge** için kullanılan Go-Kart aracının otonom sürüş misyonunu yönetir. Aracın farklı çalışma modları arasında geçişini kontrol eder ve sensör hazırlığını, konumlandırmayı ve hedef yönetimini sağlar.

Tüm ROS 2 node adları ve topic isimleri bu modülde merkezi olarak yönetilir, böylece sistem genelinde herhangi bir değişiklik yapılması kolaylaşır.


## Paket İçeriği

* **`mission_control/config/`**: Node adları ve topic isimleri için konfigürasyon dosyaları (gelecek sürümlerde).
* **`mission_control/include/mission_control/`**: 
  - **`mode_base.hpp`**: Tüm modların kalıtıp aldığı temel sınıf.
  - **`mode_start.hpp`**: Başlangıç modu (sensör kontrolü ve konumlandırma).
  - **`mode_pause.hpp`**: Bekleme modu (belirli süre duraklatma).
  - **`mode_run.hpp`**: Çalışma modu (hedef takibi ve otonom sürüş).
  - **`mode_park.hpp`**: Park modu (aracı konumlandırma).
  - **`mode_emergency.hpp`**: Acil durum modu (acil durdurma).
* **`mission_control/src/`**: Yukarıdaki modların uygulama dosyaları (.cpp).
* **`mission_control_launch.cpp`**: Ana ana launch dosyası ve node örneklemeleri.


## Çalışma Modları (Modes)

### 1. **START Mode** (`mode_start`)
Aracı başlatmadan önce tüm gereksinimleri kontrol eder:
- Lidar, GNSS, IMU, Kamera ve Odometry sensörleri veri alıyor mu?
- Konumlandırma (Localization) başarıyla yapıldı mı?

✅ Tüm kontroller başarılıysa → **PAUSE Mode**'e geçer.
❌ Herhangi bir sorun varsa → Hata kaydeder ve START Mode'de kalır.

**Önemli Topic'ler:**
```cpp
LIDAR_TOPIC = ""                    // LIDAR sensöründen veri
GNSS_TOPIC = ""                     // GPS/GNSS sensöründen veri
IMU_TOPIC = ""                      // IMU sensöründen veri
CAMERA_TOPIC = ""                   // Kamera sensöründen veri
ODOM_TOPIC = ""                     // Odometry sensöründen veri
"/localization/initialization_state" // Konumlandırma durumu
```

### 2. **PAUSE Mode** (`mode_pause`)
Aracı 5 saniye boyunca duraklatır. Bu modda belirli görevler gerçekleştirilebilir.

✅ Bekleme süresi tamamlanırsa → **RUN Mode**'e geçer.

**Yapılandırılabilir Parametre:**
```cpp
PAUSE_DURATION_SECONDS = 5.0  // Bekleme süresi (saniye)
```

### 3. **RUN Mode** (`mode_run`)
Aracı otonom olarak çalıştırır. Hedef noktalarına gitmesini sağlar.

- Hedef dizisini alır ve bir bir işler.
- Autoware'i `engage` komutuyla aktive eder.
- Her hedefe ulaşıldığında bir sonrakine geçer.
- Acil durum algılanırsa **EMERGENCY Mode**'e geçer.
- Tüm hedefler tamamlanırsa **PAUSE Mode**'e döner.

**Önemli Topic'ler:**
```cpp
GOAL_PUBLISHER_TOPIC = "/planning/mission_planning/goal"
ENGAGE_PUBLISHER_TOPIC = "/autoware/engage"
GOAL_ARRAY_SUBSCRIBER_TOPIC = "/mission_control/goal_array"
ROUTE_SUBSCRIBER_TOPIC = "/api/routing/route"
TRAJECTORY_SUBSCRIBER_TOPIC = "/planning/scenario_planning/lane_driving/trajectory"
KINEMATICS_SUBSCRIBER_TOPIC = "/api/vehicle/kinematics"
EMERGENCY_SUBSCRIBER_TOPIC = "/api/autoware/get/emergency"
```

### 4. **PARK Mode** (`mode_park`)
Aracı güvenli bir şekilde park etme prosedürünü gerçekleştirir.

*(Henüz tam olarak uygulanmadı)*

### 5. **EMERGENCY Mode** (`mode_emergency`)
Acil durum gerçekleştiğinde aracı acilen durdurur.

*(Henüz tam olarak uygulanmadı)*


## Node Adları ve Topic'leri Yönetme (Merkezi Konfigürasyon)

Tüm node adları ve topic isimleri, her modun `.hpp` (header) dosyasında string sabitleri olarak tanımlanmıştır. Bu sayede bir yerde değiştirip tüm yerden etkili olur.

### Örnek: START Mode'deki Sabitleri Değiştirmek

Dosya: `include/mission_control/mode_start.hpp`

```cpp
class StartMode : public ModeBase {
public:
    std::string LIDAR_TOPIC = "";
    std::string GNSS_TOPIC = "";
    std::string IMU_TOPIC = "";
    std::string CAMERA_TOPIC = "";
    std::string ODOM_TOPIC = "";
    // ... diğer sabitleri burada tanımlayın
};
```

### Örnek: RUN Mode'deki Sabitleri Değiştirmek

Dosya: `include/mission_control/mode_run.hpp`

```cpp
class RunMode : public ModeBase {
public:
    std::string GOAL_PUBLISHER_TOPIC = "/planning/mission_planning/goal";
    std::string ENGAGE_PUBLISHER_TOPIC = "/autoware/engage";
    std::string GOAL_ARRAY_SUBSCRIBER_TOPIC = "/mission_control/goal_array";
    // ... diğer topic'ler
};
```

| Sabit | Nerede Tanımlanır? | Ne Zaman Değişir? | Neden Önemli? |
| :--- | :--- | :--- | :--- |
| **`LIDAR_TOPIC`** | `mode_start.hpp` | LIDAR sensörü veya driver değişirse | Sensör kontrolü başarısız olur. |
| **`GOAL_PUBLISHER_TOPIC`** | `mode_run.hpp` | Routing sistemi değişirse | Hedefler Autoware'e iletilmez. |
| **`ENGAGE_PUBLISHER_TOPIC`** | `mode_run.hpp` | Autoware iletişim protokolü değişirse | Araç otonom sürüşe başlamaz. |
| **`PAUSE_DURATION_SECONDS`** | `mode_pause.hpp` | Duraklatma süresi ayarlanmak istenirse | Modlar arasında geçiş zamanlaması etkilenir. |

---

## Topic İsimleri Değişirse Ne Yapılmalı?

Eğer Autoware veya başka sistemlerle iletişim kurulan ROS 2 topic isimleri değişirse:

### 1. İlgili Mod Dosyasını Açın
Örneğin, `/planning/mission_planning/goal` topic ismi değişirse:
- Dosya: `include/mission_control/mode_run.hpp`
- Sabit: `GOAL_PUBLISHER_TOPIC`

### 2. Yeni Topic İsmini Atayın
```cpp
class RunMode : public ModeBase {
public:
    std::string GOAL_PUBLISHER_TOPIC = "/planning/new_goal_topic";  // Yeni isim
    // ...
};
```

### 3. Paketi Tekrar Derleyin
```bash
colcon build --packages-select mission_control --cmake-clean-cache
```

---

## Sensör Yoksa Ne Yapılmalı? (START Mode)

Eğer bazı sensörler aracınızda yüklü değilse, START Mode'deki ilgili topic isimlerini **boş string** yapın:

Dosya: `include/mission_control/mode_start.hpp`

```cpp
class StartMode : public ModeBase {
public:
    std::string LIDAR_TOPIC = "";          // Lidar yok → kontrol atlanır
    std::string GNSS_TOPIC = "/gnss/fix";  // GNSS var
    std::string IMU_TOPIC = "";            // IMU yok → kontrol atlanır
    std::string CAMERA_TOPIC = "";         // Kamera yok → kontrol atlanır
    std::string ODOM_TOPIC = "/odom";      // Odometry var
};
```

START Mode, boş olmayan topic'ler için otomatik olarak kontrol gerçekleştirir.

---

## Nasıl Derlenir? (Build)

Node adlarında veya topic isimlerinde herhangi bir değişiklik yaptıktan sonra paketi **mutlaka** tekrar derlemelisiniz. Aksi takdirde eski konfigürasyon kullanılmaya devam eder.

```bash
cd ~/autoware

# 1. Eski paket kalıntılarını temizleyin (Önerilir)
colcon build --packages-select mission_control --cmake-clean-cache

# 2. Yeni ayarlarla derleyin
colcon build --symlink-install --packages-select mission_control

# 3. Ortamı güncelleyin
source install/setup.bash
```

---

## Kütüphaneler ve Bağımlılıklar

* **`rclcpp`**: ROS 2 C++ client library
* **`sensor_msgs`**: Sensör verileri (PointCloud2, NavSatFix, Imu, Image, Odometry)
* **`geometry_msgs`**: Geometrik veriler (PoseStamped, PoseArray, Twist)
* **`nav_msgs`**: Navigasyon verileri (Odometry)
* **`autoware_planning_msgs`**: Autoware planlama mesajları (LaneletRoute, Trajectory)
* **`autoware_adapi_v1_msgs`**: Autoware API mesajları
* **`std_msgs`**: Temel mesaj türleri (Bool, Float64)

---

## Sık Sorulan Sorular (FAQ)

**S: Araç START Mode'da kalıp ilerlemiyorsa?**
- A: Logları kontrol edin: `ros2 run mission_control mission_control_launch --ros-args --log-level info`. Hangi sensör veri almıyorsa bulun ve konfigürasyonunda boş string yapın veya sensörü bağlayın.

**S: Topic isimleri nerede değiştirilir?**
- A: Her modun `.hpp` (header) dosyasında. Derledikten sonra değişiklikler uygulanır.

**S: Pause Mode'nin süresi nasıl değiştirilir?**
- A: `include/mission_control/mode_pause.hpp` dosyasında `PAUSE_DURATION_SECONDS` değerini değiştirin.

**S: Araç hedeflere gitmiyorsa?**
- A: `GOAL_PUBLISHER_TOPIC` ve `ENGAGE_PUBLISHER_TOPIC` isimlerinin Autoware konfigürasyonu ile eşleşip eşleşmediğini kontrol edin.

