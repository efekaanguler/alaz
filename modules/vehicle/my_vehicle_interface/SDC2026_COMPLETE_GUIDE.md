# SDC 2026 Kart Vehicle Interface - Kapsamlı Teknik Dokümantasyon

Bu dokümanda, Self Driving Challenge 2026 yarışması için oluşturduğumuz vehicle interface paketinin **baştan sona tüm detayları** anlatılmaktadır.

---

# 📚 İÇİNDEKİLER

1. [Giriş ve Arka Plan](#1-giriş-ve-arka-plan)
2. [Neden Bu Kodu Yazdık?](#2-neden-bu-kodu-yazdık)
3. [Referans Aldığımız Belgeler](#3-referans-aldığımız-belgeler)
4. [Kart Donanım Analizi](#4-kart-donanım-analizi)
5. [Kontrol Tipi Kararı (MIXED)](#5-kontrol-tipi-kararı-mixed)
6. [ROS 2 ve Autoware Mimarisi](#6-ros-2-ve-autoware-mimarisi)
7. [CAN Bus Sistemi](#7-can-bus-sistemi)
8. [Tüm Dosyaların Detaylı Açıklaması](#8-tüm-dosyaların-detaylı-açıklaması)
9. [Docker ve Build Süreci](#9-docker-ve-build-süreci)
10. [Test ve Doğrulama](#10-test-ve-doğrulama)
11. [Sonraki Adımlar](#11-sonraki-adımlar)

---

# 1. GİRİŞ VE ARKA PLAN

## 1.1 Problem Neydi?

Self Driving Challenge 2026 yarışmasında, bir go-kart'ı otonom olarak sürmemiz gerekiyor. Bunun için:

1. **Autoware** - Açık kaynak otonom sürüş yazılımı kullanıyoruz
2. **Go-Kart** - Organizasyon tarafından sağlanan donanım platformu
3. **Köprü** - Autoware'in çıktılarını kart'ın anlayacağı formata çeviren yazılım

Bu "köprü" yazılımına **Vehicle Interface** deniyor.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              GENEL MİMARİ                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌─────────────┐         ┌─────────────────────┐         ┌─────────────┐  │
│   │             │         │                     │         │             │  │
│   │  AUTOWARE   │ ──────► │  VEHICLE INTERFACE  │ ──────► │   GO-KART   │  │
│   │  (Planlama, │  ROS 2  │  (Bizim yazdığımız) │   CAN   │  (Motorlar, │  │
│   │  Kontrol)   │ Topics  │                     │   Bus   │  Frenler)   │  │
│   │             │         │                     │         │             │  │
│   └─────────────┘         └─────────────────────┘         └─────────────┘  │
│                                                                             │
│   ◄───────────────────────────────────────────────────────────────────────► │
│              ROS 2 Mesajları              CAN Frame'leri                    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 1.2 Vehicle Interface'in Görevi

Vehicle Interface şu işleri yapar:

### Autoware → Kart (Komutlar)
- Direksiyon açısı komutu al → CAN frame olarak servo'ya gönder
- Hız/ivme komutu al → CAN frame olarak motor/fren'e gönder

### Kart → Autoware (Durum)
- Hız sensöründen CAN frame al → ROS 2 topic olarak yayınla
- Direksiyon pozisyonundan CAN frame al → ROS 2 topic olarak yayınla

---

# 2. NEDEN BU KODU YAZDIK?

## 2.1 Autoware'in Beklentileri

Autoware, aşağıdaki ROS 2 topic'lerini bekler:

### Autoware'in Publish Ettiği (Komutlar):
| Topic | Mesaj Tipi | Açıklama |
|-------|-----------|----------|
| `/control/command/control_cmd` | `Control` | Direksiyon açısı, hız, ivme |
| `/control/command/gear_cmd` | `GearCommand` | Vites (D, R, P, N) |
| `/control/command/turn_indicators_cmd` | `TurnIndicatorsCommand` | Sinyal |
| `/control/command/hazard_lights_cmd` | `HazardLightsCommand` | Dörtlü |

### Autoware'in Subscribe Olduğu (Durum):
| Topic | Mesaj Tipi | Açıklama |
|-------|-----------|----------|
| `/vehicle/status/velocity_status` | `VelocityReport` | Anlık hız |
| `/vehicle/status/steering_status` | `SteeringReport` | Direksiyon açısı |
| `/vehicle/status/gear_status` | `GearReport` | Mevcut vites |
| `/vehicle/status/control_mode` | `ControlModeReport` | Otonom/Manuel |

## 2.2 Kart'ın Gerçekliği

Kart sadece CAN bus üzerinden iletişim kurabilir. Yani:
- ROS 2 bilmiyor
- Autoware mesajlarını anlamıyor
- Sadece belirli CAN ID'lerine belirli formatda veri bekliyor

**İşte tam bu noktada Vehicle Interface devreye giriyor!**

---

# 3. REFERANS ALDIĞIMIZ BELGELER

## 3.1 Autoware Dokümantasyonu

İki önemli Autoware sayfasını inceledik:

### Sayfa 1: Vehicle Interface Overview
**URL**: https://autowarefoundation.github.io/autoware-documentation/main/tutorials/integrating-autoware/creating-vehicle-interface-package/vehicle-interface/

Bu sayfadan öğrendiklerimiz:
- Vehicle interface'in ne olduğu
- İki kontrol tipi: Type A ve Type B
- CAN veya Serial haberleşme seçenekleri
- ros2_socketcan paketi önerisi

### Sayfa 2: Creating Vehicle Interface
**URL**: https://autowarefoundation.github.io/autoware-documentation/main/tutorials/integrating-autoware/creating-vehicle-interface-package/creating-vehicle-interface/

Bu sayfadan öğrendiklerimiz:
- Paket yapısı nasıl olmalı
- Hangi topic'lere subscribe olmalı
- Hangi topic'leri publish etmeli
- Örnek kod yapısı

### ros2_socketcan Repository
**URL**: https://github.com/autowarefoundation/ros2_socketcan

Bu sayfadan öğrendiklerimiz:
- Linux SocketCAN wrapper'ı
- `/from_can_bus` ve `/to_can_bus` topic'leri
- CAN frame mesaj formatı

## 3.2 SDC 2026 Yarışma Kuralları

Sen bana yarışma kurallarını gönderdin. Bu belgeden çıkarttığımız kritik bilgiler:

### Sensörler (T 2)
```
T 2.1  : 3 kamera, 1 LiDAR, 1 hız sensörü
T 2.8  : Hız sensörü hektometre/saat cinsinden raporluyor
T 2.9  : Minimum algılama: 15 hektometre/saat
T 2.10 : Hız sensörü CAN bus üzerinden erişilebilir
```

### Aktüatörler (T 3)
```
T 3.1  : Direksiyon için servomotor
         Fren için lineer aktüatör
         Hızlanma için elektrik motoru
T 3.2  : Aktüatörler CAN bus üzerinden kontrol edilebilir
T 3.3  : Detaylar wiki'de (A 3.3)
```

### Bilgisayar (T 4)
```
T 4.1  : Intel NUC mini bilgisayar
T 4.4  : Sensörlerle USB üzerinden iletişim
T 4.5  : CAN bus ile USB üzerinden iletişim
```

---

# 4. KART DONANIM ANALİZİ

## 4.1 Donanım Şeması

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          SDC 2026 GO-KART                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│                          ┌───────────────────┐                              │
│                          │    INTEL NUC      │                              │
│                          │   (Bilgisayar)    │                              │
│                          └─────────┬─────────┘                              │
│                                    │                                        │
│                    ┌───────────────┼───────────────┐                        │
│                    │               │               │                        │
│                   USB             USB             USB                       │
│                    │               │               │                        │
│              ┌─────▼─────┐   ┌─────▼─────┐   ┌─────▼─────┐                  │
│              │  Kamera   │   │  LiDAR    │   │ USB-CAN   │                  │
│              │  (3 adet) │   │           │   │ Adaptör   │                  │
│              └───────────┘   └───────────┘   └─────┬─────┘                  │
│                                                    │                        │
│                                              CAN BUS                        │
│                    ┌───────────────────────────────┼───────────────┐        │
│                    │               │               │               │        │
│              ┌─────▼─────┐   ┌─────▼─────┐   ┌─────▼─────┐   ┌─────▼─────┐  │
│              │ DİREKSİYON│   │   FREN    │   │   MOTOR   │   │    HIZ    │  │
│              │  SERVO    │   │ AKTÜATÖR  │   │ (Throttle)│   │  SENSÖRÜ  │  │
│              └───────────┘   └───────────┘   └───────────┘   └───────────┘  │
│                                                                             │
│              Komut alır      Komut alır      Komut alır      Veri gönderir  │
│              (0x100?)        (0x101?)        (0x102?)        (0x200?)       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 4.2 Veri Akışı

### Komut Akışı (Autoware → Kart)
```
Autoware Planner
      │
      ▼ (Yol planı)
Autoware Controller
      │
      ▼ (Control.msg: steering, acceleration)
Vehicle Interface
      │
      ▼ (CAN Frame encode)
ros2_socketcan
      │
      ▼ (SocketCAN)
USB-CAN Adaptör
      │
      ▼ (Fiziksel CAN sinyali)
Aktüatörler (Servo, Motor, Fren)
```

### Durum Akışı (Kart → Autoware)
```
Hız Sensörü
      │
      ▼ (Fiziksel CAN sinyali)
USB-CAN Adaptör
      │
      ▼ (SocketCAN)
ros2_socketcan
      │
      ▼ (CAN Frame: /from_can_bus)
Vehicle Interface
      │
      ▼ (VelocityReport.msg)
Autoware Localization/Control
```

### 🎯 Adım Adım Sinyal Akışı (Özet Tablo)

**Senaryo**: Autoware "15 derece sola dön" diyor

| Adım | Kim | Ne Yapıyor | Çıktı |
|------|-----|------------|-------|
| 1 | **Autoware** | Karar verir, topic publish eder | ROS topic: `/control/command/control_cmd` |
| 2 | **Vehicle Interface** | ROS mesajını CAN frame'e çevirir (encode) | ROS topic: `/to_can_bus` |
| 3 | **ros2_socketcan** | Topic'i Linux can0 interface'ine yazar | Linux SocketCAN |
| 4 | **SocketCAN driver** | can0'dan USB-CAN adaptöre gönderir | USB paketi |
| 5 | **USB-CAN adaptör** | USB'yi elektrik sinyaline çevirir | CAN sinyali |
| 6 | **Servo motor** | Elektrik sinyalini harekete çevirir | Tekerlek döndü! 🎉 |

---

# 5. KONTROL TİPİ KARARI (MIXED)

## 5.1 Autoware'in Tanımladığı Kontrol Tipleri

Autoware dokümantasyonu iki tip kontrol tanımlıyor:

### Type A: Target Interface
```
Direksiyon: Hedef açı (örn: 15 derece sola)
Hız: Hedef hız veya hedef ivme (örn: 5 m/s veya 2 m/s²)
```

Araç kendi içinde bu hedeflere ulaşmak için gerekli motor/fren/direksiyon ayarını yapar.

**Örnekler**: Çoğu modern drive-by-wire araçlar

### Type B: Actuation Interface
```
Direksiyon: Direksiyon torku veya direk açı komutu
Gaz: Gaz pedalı yüzdesi (0-100%)
Fren: Fren pedalı yüzdesi (0-100%)
```

Araç direkt olarak verilen değerleri uygular, kendi kontrolü yoktur.

**Örnekler**: Go-kartlar, basit robotlar

## 5.2 Neden MIXED Tip Seçtik?

Yarışma kurallarını analiz ettiğimizde:

### Direksiyon (SERVO) → Type A
```
T 3.1: "A servomotor for moving the steering wheel"
```
- Servo motorlar genellikle "hedef açıya git" komutu alır
- Bu Type A davranışı
- Biz sadece hedef açıyı göndereceğiz, servo kendisi o açıya gidecek

### Fren (LİNEER AKTÜATÖR) → Type B
```
T 3.1: "A linear actuator for applying the rear brake"
```
- Lineer aktüatör direkt pozisyon/güç komutu alır
- "Şu kadar fren uygula" şeklinde
- Bu Type B davranışı

### Motor (ELEKTRİK MOTOR) → Type B
```
T 3.1: "An electric motor for accelerating the kart"
```
- Elektrik motoru direkt güç komutu alır
- "Şu kadar throttle aç" şeklinde
- Bu Type B davranışı

### Sonuç: MIXED

```
┌─────────────────────────────────────────────────────────────────┐
│                    KONTROL TİPİ KARARI                          │
├───────────────┬─────────────┬───────────────────────────────────┤
│ Bileşen       │ Tip         │ Açıklama                          │
├───────────────┼─────────────┼───────────────────────────────────┤
│ Direksiyon    │ Type A      │ Hedef açı → Servo kendisi gider   │
│ Gaz (Motor)   │ Type B      │ Throttle %  → Direkt motor gücü   │
│ Fren          │ Type B      │ Brake %     → Direkt fren gücü    │
└───────────────┴─────────────┴───────────────────────────────────┘
```

## 5.3 Koddaki Yansıması

```cpp
// vehicle_interface_node.cpp - sendToVehicle() fonksiyonu

void VehicleInterfaceNode::sendToVehicle()
{
  // ================================================================
  // TYPE A: Direksiyon - Hedef açıyı direkt gönder
  // ================================================================
  double steering_angle = control_cmd_ptr_->lateral.steering_tire_angle;
  auto steering_frame = can_utils::encodeSteeringCommand(steering_angle, can_ids_);
  can_frame_pub_->publish(steering_frame);

  // ================================================================
  // TYPE B: Gaz/Fren - İvmeyi throttle/brake yüzdesine çevir
  // ================================================================
  double target_accel = control_cmd_ptr_->longitudinal.acceleration;
  
  double throttle_cmd = 0.0;
  double brake_cmd = 0.0;

  if (target_accel > 0.0) {
    // Pozitif ivme = gaz
    throttle_cmd = std::clamp(target_accel / 3.0, 0.0, 1.0);  // 3 m/s² = %100
  } else if (target_accel < 0.0) {
    // Negatif ivme = fren
    brake_cmd = std::clamp(-target_accel / 5.0, 0.0, 1.0);    // -5 m/s² = %100
  }

  auto throttle_frame = can_utils::encodeThrottleCommand(throttle_cmd, can_ids_);
  auto brake_frame = can_utils::encodeBrakeCommand(brake_cmd, can_ids_);
  can_frame_pub_->publish(throttle_frame);
  can_frame_pub_->publish(brake_frame);
}
```

---

# 6. ROS 2 VE AUTOWARE MİMARİSİ

## 6.1 ROS 2 Nedir?

ROS 2 (Robot Operating System 2) bir robotik middleware'idir. Ana özellikleri:

- **Topic'ler**: Publish/Subscribe pattern ile veri iletişimi
- **Node'lar**: Bağımsız çalışan yazılım birimleri
- **Message'lar**: Standart veri formatları

## 6.2 Bizim Node'un Yapısı

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     VehicleInterfaceNode                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  SUBSCRIBERS (Dinlediğimiz Topic'ler)                                       │
│  ─────────────────────────────────────                                      │
│  /control/command/control_cmd      → Autoware'den kontrol komutu           │
│  /control/command/gear_cmd         → Autoware'den vites komutu             │
│  /control/command/turn_indicators  → Autoware'den sinyal komutu            │
│  /control/command/hazard_lights    → Autoware'den dörtlü komutu            │
│  /from_can_bus                     → ros2_socketcan'den CAN frame          │
│                                                                             │
│  PUBLISHERS (Yayınladığımız Topic'ler)                                      │
│  ────────────────────────────────────                                       │
│  /vehicle/status/velocity_status   → Autoware'e hız durumu                 │
│  /vehicle/status/steering_status   → Autoware'e direksiyon durumu          │
│  /vehicle/status/gear_status       → Autoware'e vites durumu               │
│  /vehicle/status/control_mode      → Autoware'e kontrol modu               │
│  /to_can_bus                       → ros2_socketcan'e CAN frame            │
│                                                                             │
│  TIMER (Periyodik İşlem)                                                    │
│  ───────────────────────                                                    │
│  100Hz (10ms) → sendToVehicle() + publishVehicleStatus()                   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 6.3 Callback'ler ve Veri Akışı

```cpp
// 1. Autoware komut gönderdiğinde
void onControlCmd(const Control::ConstSharedPtr msg) {
  control_cmd_ptr_ = msg;  // Komutu sakla
  last_command_time_ = this->now();  // Timeout için zaman damgası
}

// 2. CAN bus'tan hız sensörü verisi geldiğinde
void onCanFrame(const Frame::ConstSharedPtr msg) {
  if (msg->id == can_ids_.speed_sensor) {
    current_velocity_mps_ = can_utils::decodeSpeedSensor(*msg);
  }
}

// 3. Her 10ms'de bir (100Hz timer)
void onTimer() {
  sendToVehicle();         // CAN komutlarını gönder
  publishVehicleStatus();  // Autoware'e durum bildir
}
```

---

# 7. CAN BUS SİSTEMİ

## 7.1 CAN Bus Nedir?

Controller Area Network (CAN), araçlarda yaygın kullanılan bir haberleşme protokolüdür.

### Temel Özellikler:
- **Diferansiyel sinyal**: Gürültüye dayanıklı
- **Multi-master**: Birden fazla cihaz iletişim kurabilir
- **Broadcast**: Tüm cihazlar tüm mesajları görür
- **ID tabanlı**: Her mesajın benzersiz ID'si var

### CAN Frame Yapısı:
```
┌──────────────────────────────────────────────────────────────┐
│                      CAN FRAME                               │
├──────────┬───────────┬────────────────────────────┬──────────┤
│ ID       │ DLC       │ DATA                       │ CRC      │
│ (11-bit) │ (4-bit)   │ (0-8 bytes)                │          │
├──────────┼───────────┼────────────────────────────┼──────────┤
│ 0x100    │ 8         │ [0x12, 0x34, 0x00, ...]    │ Auto     │
└──────────┴───────────┴────────────────────────────┴──────────┘
```

## 7.2 ros2_socketcan

Linux'ta CAN bus'a erişmek için SocketCAN kullanılır. ros2_socketcan paketi bunu ROS 2'ye köprüler.

### Topic'ler:
```
/from_can_bus  → Kart'tan gelen CAN frame'leri (can_msgs/Frame)
/to_can_bus    → Kart'a gönderilecek CAN frame'leri (can_msgs/Frame)
```

### can_msgs/Frame Yapısı:
```cpp
struct Frame {
  uint32_t id;           // CAN arbitration ID
  bool is_extended;      // 29-bit ID mi?
  bool is_rtr;           // Remote transmission request?
  uint8_t dlc;           // Data length code (0-8)
  std::array<uint8_t, 8> data;  // Veri baytları
};
```

## 7.3 Bizim CAN Kullanımımız

### CAN ID'leri (Placeholder - Wiki'den güncellenmeli):
```yaml
# config/vehicle_interface.param.yaml
steering_can_id: 0x100     # Direksiyon komutu
brake_can_id: 0x101        # Fren komutu
throttle_can_id: 0x102     # Gaz komutu
speed_can_id: 0x200        # Hız sensörü verisi
```

### Veri Encoding Örneği:
```cpp
// can_utils.cpp - encodeSteeringCommand

can_msgs::msg::Frame encodeSteeringCommand(double steering_angle_rad, const CanIds& can_ids)
{
  Frame frame;
  frame.id = can_ids.steering_command;  // 0x100
  frame.dlc = 8;

  // Radyanı dereceye çevir
  double steering_deg = radToDeg(steering_angle_rad);
  
  // -30 ile +30 derece arasında sınırla
  steering_deg = std::clamp(steering_deg, -30.0, 30.0);

  // 0.1 derece çözünürlükle encode et
  int16_t steering_raw = static_cast<int16_t>(steering_deg * 10.0);

  // Little-endian olarak ilk 2 byte'a yaz
  frame.data[0] = steering_raw & 0xFF;         // LSB
  frame.data[1] = (steering_raw >> 8) & 0xFF;  // MSB
  frame.data[2] = 0;
  frame.data[3] = 0;
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;

  return frame;
}
```

### Veri Decoding Örneği:
```cpp
// can_utils.cpp - decodeSpeedSensor

double decodeSpeedSensor(const Frame& frame)
{
  // İlk 2 byte'tan hız değerini oku (little-endian)
  uint16_t speed_hmh = static_cast<uint16_t>(frame.data[0]) |
                       (static_cast<uint16_t>(frame.data[1]) << 8);

  // Hektometre/saat'i metre/saniye'ye çevir
  // 1 hm/h = 100 m/h = 100/3600 m/s = 0.0278 m/s
  double speed_mps = speed_hmh * 0.0278;

  return speed_mps;
}
```

## 7.4 Hız Birimi Dönüşümü

Yarışma kurallarına göre hız sensörü **hektometre/saat** cinsinden veri gönderiyor:

```
1 hektometre = 100 metre

Dönüşüm:
hpm/h → m/s = hpm/h × (100 / 3600) = hpm/h × 0.0278

Örnekler:
15 hm/h (minimum) = 0.42 m/s ≈ 1.5 km/h
50 hm/h           = 1.39 m/s ≈ 5 km/h
100 hm/h          = 2.78 m/s ≈ 10 km/h
```

---

# 8. TÜM DOSYALARIN DETAYLI AÇIKLAMASI

## 8.1 Paket Yapısı

```
my_vehicle_interface/
├── include/my_vehicle_interface/
│   ├── vehicle_interface_node.hpp   # Node sınıf tanımı
│   └── can_utils.hpp                # CAN yardımcı fonksiyonlar
├── src/
│   ├── vehicle_interface_node.cpp   # Node implementasyonu
│   ├── can_utils.cpp                # CAN fonksiyon implementasyonu
│   └── main.cpp                     # Giriş noktası
├── launch/
│   └── vehicle_interface.launch.xml # Launch dosyası
├── config/
│   └── vehicle_interface.param.yaml # Parametreler
├── test/
│   └── test_can_simulation.py       # CAN simülatör
├── CMakeLists.txt                   # Build konfigürasyonu
├── package.xml                      # Paket tanımı
└── README.md                        # Dokümantasyon
```

## 8.2 package.xml - Paket Tanımı

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_vehicle_interface</name>
  <version>1.0.0</version>
  <description>Autoware vehicle interface with CAN support</description>
  <maintainer email="zuhalkoksal@sdc2026.com">Zuhal Koksal</maintainer>
  <license>Apache-2.0</license>

  <!-- Build sistemi -->
  <buildtool_depend>ament_cmake_auto</buildtool_depend>

  <!-- Runtime bağımlılıklar -->
  <depend>rclcpp</depend>                    <!-- ROS 2 C++ client -->
  <depend>rclcpp_components</depend>         <!-- Component registration -->
  <depend>autoware_control_msgs</depend>     <!-- Kontrol mesajları -->
  <depend>autoware_vehicle_msgs</depend>     <!-- Araç mesajları -->
  <depend>can_msgs</depend>                  <!-- CAN frame mesajları -->
  <depend>std_msgs</depend>                  <!-- Standart mesajlar -->

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Neden Bu Bağımlılıklar?

| Bağımlılık | Neden Gerekli |
|------------|---------------|
| `rclcpp` | ROS 2 node oluşturmak için |
| `rclcpp_components` | Component olarak register etmek için |
| `autoware_control_msgs` | Control.msg kullanmak için |
| `autoware_vehicle_msgs` | VelocityReport.msg vb. için |
| `can_msgs` | Frame.msg kullanmak için |

## 8.3 CMakeLists.txt - Build Konfigürasyonu

```cmake
cmake_minimum_required(VERSION 3.14)
project(my_vehicle_interface)

# Bağımlılıkları otomatik bul
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Kütüphane oluştur (shared library)
ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/vehicle_interface_node.cpp
  src/can_utils.cpp
)

# Executable oluştur
ament_auto_add_executable(${PROJECT_NAME}_node
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

# Component olarak register et
rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "my_vehicle_interface::VehicleInterfaceNode"
  EXECUTABLE vehicle_interface_node
)

# Launch ve config dosyalarını install et
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
```

### Build Sistemi Açıklaması

```
                    ┌─────────────────────────────┐
                    │      CMakeLists.txt         │
                    └─────────────┬───────────────┘
                                  │
                    ┌─────────────▼───────────────┐
                    │   ament_cmake_auto          │
                    │   (Otomatik bağımlılık      │
                    │    bulma ve include)        │
                    └─────────────┬───────────────┘
                                  │
              ┌───────────────────┼───────────────────┐
              │                   │                   │
    ┌─────────▼─────────┐ ┌───────▼───────┐ ┌────────▼────────┐
    │ ${PROJECT}_lib    │ │ ${PROJECT}_   │ │ Component       │
    │ (Shared Library)  │ │ node (Exe)    │ │ Registration    │
    └─────────┬─────────┘ └───────┬───────┘ └────────┬────────┘
              │                   │                   │
              └───────────────────┼───────────────────┘
                                  │
                    ┌─────────────▼───────────────┐
                    │   install/                  │
                    │   ├── lib/                  │
                    │   │   └── my_vehicle_...    │
                    │   └── share/                │
                    │       └── my_vehicle_.../   │
                    │           ├── launch/       │
                    │           └── config/       │
                    └─────────────────────────────┘
```

## 8.4 vehicle_interface_node.hpp - Header Dosyası

Bu dosya, VehicleInterfaceNode sınıfının tanımını içerir.

### Sınıf Yapısı:

```cpp
class VehicleInterfaceNode : public rclcpp::Node
{
public:
  explicit VehicleInterfaceNode(const rclcpp::NodeOptions & options);

private:
  // CALLBACK FONKSİYONLARI
  void onControlCmd(...);      // Autoware kontrol komutu geldiğinde
  void onGearCmd(...);         // Autoware vites komutu geldiğinde
  void onCanFrame(...);        // CAN frame geldiğinde
  void onTimer();              // Her 10ms'de bir

  // YARDIMCI FONKSİYONLAR
  void sendToVehicle();        // CAN komutlarını gönder
  void publishVehicleStatus(); // Autoware'e durum bildir

  // SUBSCRIBER'LAR
  rclcpp::Subscription<Control>::SharedPtr control_cmd_sub_;
  rclcpp::Subscription<GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<Frame>::SharedPtr can_frame_sub_;
  // ...

  // PUBLISHER'LAR
  rclcpp::Publisher<VelocityReport>::SharedPtr velocity_report_pub_;
  rclcpp::Publisher<SteeringReport>::SharedPtr steering_report_pub_;
  rclcpp::Publisher<Frame>::SharedPtr can_frame_pub_;
  // ...

  // TIMER
  rclcpp::TimerBase::SharedPtr timer_;

  // DURUM DEĞİŞKENLERİ
  Control::ConstSharedPtr control_cmd_ptr_;   // Son kontrol komutu
  double current_velocity_mps_{0.0};          // Mevcut hız
  double current_steering_angle_rad_{0.0};    // Mevcut direksiyon açısı

  // KONFİGÜRASYON
  can_utils::CanIds can_ids_;                 // CAN ID'leri
  double loop_rate_hz_;                       // Döngü frekansı
  double command_timeout_sec_;                // Timeout süresi
};
```

## 8.5 vehicle_interface_node.cpp - Implementasyon

### Constructor (Yapıcı Fonksiyon)

```cpp
VehicleInterfaceNode::VehicleInterfaceNode(const rclcpp::NodeOptions & options)
: Node("vehicle_interface_node", options)
{
  // 1. PARAMETRELERİ OKU
  loop_rate_hz_ = this->declare_parameter("loop_rate_hz", 100.0);
  command_timeout_sec_ = this->declare_parameter("command_timeout_sec", 0.5);
  
  // CAN ID'lerini parametre olarak al
  can_ids_.steering_command = this->declare_parameter("steering_can_id", 0x100);
  can_ids_.brake_command = this->declare_parameter("brake_can_id", 0x101);
  can_ids_.throttle_command = this->declare_parameter("throttle_can_id", 0x102);
  can_ids_.speed_sensor = this->declare_parameter("speed_can_id", 0x200);

  // 2. SUBSCRIBER'LARI OLUŞTUR
  control_cmd_sub_ = this->create_subscription<Control>(
    "/control/command/control_cmd", rclcpp::QoS{1},
    std::bind(&VehicleInterfaceNode::onControlCmd, this, _1));
  
  can_frame_sub_ = this->create_subscription<Frame>(
    "/from_can_bus", rclcpp::QoS{100},
    std::bind(&VehicleInterfaceNode::onCanFrame, this, _1));

  // 3. PUBLISHER'LARI OLUŞTUR
  velocity_report_pub_ = this->create_publisher<VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS{1});
  
  can_frame_pub_ = this->create_publisher<Frame>(
    "/to_can_bus", rclcpp::QoS{100});

  // 4. TIMER OLUŞTUR (100Hz)
  auto timer_period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
  timer_ = this->create_wall_timer(timer_period,
    std::bind(&VehicleInterfaceNode::onTimer, this));

  RCLCPP_INFO(this->get_logger(), "Vehicle Interface Node initialized!");
}
```

### Ana Döngü Mantığı

```
     ┌──────────────────────────────────────────────────────┐
     │                    NODE BAŞLADI                       │
     └──────────────────────────┬───────────────────────────┘
                                │
     ┌──────────────────────────▼───────────────────────────┐
     │          3 paralel işlem başlar:                      │
     ├───────────────┬───────────────────┬──────────────────┤
     │  Subscriber   │    Subscriber     │      Timer       │
     │  (control)    │    (CAN)          │      (100Hz)     │
     └───────┬───────┴─────────┬─────────┴────────┬─────────┘
             │                 │                   │
             ▼                 ▼                   ▼
     ┌───────────────┐ ┌───────────────┐ ┌──────────────────┐
     │ Autoware'den  │ │ CAN bus'tan   │ │ Her 10ms'de:     │
     │ komut geldi   │ │ frame geldi   │ │                  │
     │               │ │               │ │ 1. sendToVehicle │
     │ control_cmd_  │ │ current_      │ │    - Encode      │
     │ ptr_ = msg    │ │ velocity =    │ │    - Publish CAN │
     │               │ │ decode(msg)   │ │                  │
     │               │ │               │ │ 2. publishStatus │
     │               │ │               │ │    - VelocityRPT │
     │               │ │               │ │    - SteeringRPT │
     └───────────────┘ └───────────────┘ └──────────────────┘
```

## 8.6 can_utils.hpp/cpp - CAN Yardımcı Fonksiyonlar

### CAN ID Yapısı

```cpp
struct CanIds
{
  // Komutlar (Autoware → Kart)
  uint32_t steering_command = 0x100;   // TODO: Wiki'den güncelle
  uint32_t brake_command = 0x101;      // TODO: Wiki'den güncelle
  uint32_t throttle_command = 0x102;   // TODO: Wiki'den güncelle

  // Durum (Kart → Autoware)
  uint32_t speed_sensor = 0x200;       // TODO: Wiki'den güncelle
  uint32_t steering_feedback = 0x201;  // TODO: Wiki'den güncelle
};
```

### Encoding Fonksiyonları

```cpp
// Direksiyon komutunu CAN frame'e çevir
Frame encodeSteeringCommand(double steering_angle_rad, const CanIds& ids);

// Fren komutunu CAN frame'e çevir (0.0 - 1.0 arası)
Frame encodeBrakeCommand(double brake_value, const CanIds& ids);

// Gaz komutunu CAN frame'e çevir (0.0 - 1.0 arası)
Frame encodeThrottleCommand(double throttle_value, const CanIds& ids);
```

### Decoding Fonksiyonları

```cpp
// CAN frame'den hız sensörü verisini çöz (m/s olarak döner)
double decodeSpeedSensor(const Frame& frame);

// CAN frame'den direksiyon feedback'ini çöz (rad olarak döner)
double decodeSteeringFeedback(const Frame& frame);
```

### Yardımcı Fonksiyonlar

```cpp
inline double degToRad(double deg) { return deg * 0.017453292519943295; }
inline double radToDeg(double rad) { return rad * 57.29577951308232; }
inline double hmhToMs(double hmh)  { return hmh * 0.027777777777777776; }
inline double msToHmh(double ms)   { return ms * 36.0; }
```

## 8.7 vehicle_interface.param.yaml - Parametreler

```yaml
/**:
  ros__parameters:
    # Genel Ayarlar
    loop_rate_hz: 100.0           # Kontrol döngüsü frekansı (Hz)
    command_timeout_sec: 0.5      # Komut timeout süresi (saniye)

    # CAN ID'leri - KOMUTLAR (Autoware → Kart)
    # TODO: Wiki'den (A 3.3) güncelle!
    steering_can_id: 0x100        # Direksiyon servo komutu
    brake_can_id: 0x101           # Fren aktüatör komutu
    throttle_can_id: 0x102        # Motor throttle komutu

    # CAN ID'leri - DURUM (Kart → Autoware)
    # TODO: Wiki'den (A 3.3) güncelle!
    speed_can_id: 0x200           # Hız sensörü verisi
    steering_feedback_can_id: 0x201  # Direksiyon pozisyon feedback

    # Kontrol Limitleri
    max_steering_angle_deg: 30.0  # Maksimum direksiyon açısı
    max_throttle_percent: 100.0   # Maksimum gaz
    max_brake_percent: 100.0      # Maksimum fren

    # Kalibrasyon (İvme → Throttle/Brake dönüşümü)
    accel_to_throttle_gain: 0.33  # 3 m/s² = %100 throttle
    decel_to_brake_gain: 0.20     # 5 m/s² = %100 brake
```

## 8.8 vehicle_interface.launch.xml - Launch Dosyası

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <!-- Argümanlar -->
  <arg name="vehicle_id" default="$(env VEHICLE_ID sdc_kart)"/>
  <arg name="config_file" default="$(find-pkg-share my_vehicle_interface)/config/vehicle_interface.param.yaml"/>
  
  <!-- Vehicle Interface Node -->
  <node pkg="my_vehicle_interface" 
        exec="my_vehicle_interface_node" 
        name="vehicle_interface_node" 
        output="screen">
    <param from="$(var config_file)"/>
  </node>
  
  <!-- NOT: ros2_socketcan ayrı başlatılmalı -->
  <!--
  <include file="$(find-pkg-share ros2_socketcan)/launch/socket_can_bridge.launch.xml">
    <arg name="interface" value="can0"/>
  </include>
  -->
</launch>
```

---

# 9. DOCKER VE BUILD SÜRECİ

## 9.1 Docker Nedir ve Neden Kullanıyoruz?

Docker, uygulamaları izole container'larda çalıştırmamızı sağlar.

### Avantajları:
- **Tutarlılık**: Herkesin aynı ortamda çalışması
- **Bağımlılıklar**: ROS 2 + Autoware önceden kurulu
- **İzolasyon**: Mac'te bile Linux ortamı

### Autoware Docker Yapısı:

```
┌─────────────────────────────────────────────────────────────┐
│                         MAC OS                               │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│   ┌────────────────────────────────────────────────────┐    │
│   │                 DOCKER CONTAINER                    │    │
│   ├────────────────────────────────────────────────────┤    │
│   │  Ubuntu 22.04                                       │    │
│   │  ROS 2 Humble                                       │    │
│   │  Autoware Universe                                  │    │
│   │                                                     │    │
│   │  /workspace ◄──────── ~/alaz (mounted)             │    │
│   │      │                                              │    │
│   │      ├── src/                                       │    │
│   │      │   └── vehicle/                               │    │
│   │      │       └── external/                          │    │
│   │      │           └── my_vehicle_interface/          │    │
│   │      │                                              │    │
│   │      ├── build/                                     │    │
│   │      └── install/                                   │    │
│   └────────────────────────────────────────────────────┘    │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## 9.2 Docker Komutları

### Container'a Giriş:
```bash
bash ~/autoware/docker/run.sh --devel --workspace ~/alaz --headless --no-nvidia /bin/bash
```

### Parametrelerin Anlamı:
| Parametre | Açıklama |
|-----------|----------|
| `--devel` | Development modunda çalıştır |
| `--workspace ~/alaz` | ~/alaz klasörünü /workspace olarak mount et |
| `--headless` | GUI olmadan çalıştır |
| `--no-nvidia` | NVIDIA GPU kullanma (Mac için) |
| `/bin/bash` | Bash shell başlat |

### Yeni Terminal'de Aynı Container'a Bağlanma:
```bash
docker exec -it $(docker ps -q) /bin/bash
```

## 9.3 Build Süreci

### Adım 1: Paketi Workspace'e Kopyala (Mac'te)
```bash
mkdir -p ~/alaz/src/vehicle/external/
cp -r /Users/zuhalkoksal/.gemini/antigravity/scratch/my_vehicle_interface \
      ~/alaz/src/vehicle/external/
```

### Adım 2: Container'a Gir
```bash
bash ~/autoware/docker/run.sh --devel --workspace ~/alaz --headless --no-nvidia /bin/bash
```

### Adım 3: Build Et (Container İçinde)
```bash
cd /workspace

# Eksik bağımlılıkları yükle
rosdep install --from-paths src --ignore-src -r -y

# Paketi build et
colcon build --packages-select my_vehicle_interface --symlink-install

# Source et
source install/setup.bash
```

### Build Başarılı Çıktısı:
```
Starting >>> my_vehicle_interface
Finished <<< my_vehicle_interface [12.9s]
Summary: 1 package finished [13.0s]
```

## 9.4 Çalıştırma

### Node'u Başlat:
```bash
ros2 launch my_vehicle_interface vehicle_interface.launch.xml
```

### Beklenen Çıktı:
```
[INFO] Vehicle Interface Node starting...
[INFO] Loop rate: 100.0 Hz
[INFO] Steering CAN ID: 0x100
[INFO] Brake CAN ID: 0x101
[INFO] Throttle CAN ID: 0x102
[INFO] Speed sensor CAN ID: 0x200
[INFO] Vehicle Interface Node initialized successfully!
[INFO] SDC 2026 Kart Vehicle Interface starting...
```

---

# 10. TEST VE DOĞRULAMA

## 10.1 Topic'leri Kontrol Et

```bash
# Yeni terminal aç ve container'a bağlan
docker exec -it $(docker ps -q) /bin/bash
source /workspace/install/setup.bash

# Topic listesi
ros2 topic list | grep -E "(vehicle|can)"

# Beklenen çıktı:
# /vehicle/status/velocity_status
# /vehicle/status/steering_status
# /vehicle/status/gear_status
# /vehicle/status/control_mode
# /to_can_bus
```

## 10.2 Velocity Status'u İzle

```bash
ros2 topic echo /vehicle/status/velocity_status
```

### Beklenen Çıktı:
```yaml
header:
  stamp:
    sec: 1770155195
    nanosec: 946575047
  frame_id: base_link
longitudinal_velocity: 0.0    # Kart bağlı olmadığından 0
lateral_velocity: 0.0
heading_rate: 0.0
---
```

## 10.3 Fake Kontrol Komutu Gönder

```bash
ros2 topic pub -r 100 /control/command/control_cmd autoware_control_msgs/msg/Control \
  "{lateral: {steering_tire_angle: 0.1}, longitudinal: {velocity: 5.0, acceleration: 1.0}}"
```

## 10.4 CAN Frame Çıktısını İzle

```bash
ros2 topic echo /to_can_bus
```

---

# 11. SONRAKİ ADIMLAR

## 11.1 Wiki'den Alınması Gereken Bilgiler

Yarışma wiki'sine (A 3.3) eriştiğinde şunları öğren:

### CAN Message ID'leri:
```yaml
# Dolduracağın tablo:
steering_command_id: 0x???
brake_command_id: 0x???
throttle_command_id: 0x???
speed_sensor_id: 0x???
steering_feedback_id: 0x???
```

### CAN Data Formatları:
Her mesaj için:
- Kaç byte? (DLC)
- Hangi byte'lar hangi anlama geliyor?
- Little-endian mi big-endian mi?
- Scaling faktörü (örn: raw × 0.1 = derece)
- Min/max değerler

### Hız Sensörü Birimi:
```
□ Hız sensörü hangi birimde veri gönderiyor?
  - hm/h (hektometre/saat) → çarpan = 0.0278
  - km/h → çarpan = 0.2778
  - m/s → çarpan = 1.0
  - Başka bir birim? → formülü hesapla
  
Şu an varsayılan: hm/h (yarışma kurallarından)
Dosya: src/can_utils.cpp → decodeSpeedSensor()
```

### Motor/Fren Kalibrasyon Değerleri:
```
□ Motor karakteristiği:
  - %100 throttle ile kart kaç m/s² hızlanıyor?
  - Şu an varsayılan: 3.0 m/s² (PLACEHOLDER!)
  - Dosya: src/vehicle_interface_node.cpp
  
□ Fren karakteristiği:
  - %100 fren ile kart kaç m/s² yavaşlıyor?
  - Şu an varsayılan: 5.0 m/s² (PLACEHOLDER!)
  - Dosya: src/vehicle_interface_node.cpp
  
NOT: Bu değerler GERÇEK KART TESTLERİNDEN sonra kalibre edilmeli!
```

### Örnek Format Dokümantasyonu:
```
Steering Command (ID: 0x100)
├── DLC: 8 bytes
├── Byte 0-1: Target angle (int16, little-endian)
│   └── Scale: 0.1 degrees
│   └── Range: -300 to +300 (-30° to +30°)
├── Byte 2: Reserved
├── Byte 3: Reserved
├── Byte 4-7: Reserved
```

## 11.2 Kod Güncellemeleri

CAN bilgilerini öğrendikten sonra:

1. **config/vehicle_interface.param.yaml** - CAN ID'lerini güncelle
2. **src/can_utils.cpp** - Encoding/decoding fonksiyonlarını güncelle

## 11.3 Test Günü Checklist

```
□ CAN ID'leri doğru mu kontrol et
□ candump can0 ile trafik izle
□ Emergency stop hazır mı?
□ Önce durağan test (motorlar çalışmadan)
□ Her aktüatörü tek tek test et
□ Direksiyon limitleri doğru mu?
□ Fren tepki süresi yeterli mi?
□ Hız sensörü okuma doğru mu?
```

## 11.4 Güvenlik Notları

> ⚠️ **DİKKAT**: Test sırasında her zaman birisinin emergency stop'a yakın durmasını sağla!

1. **Önce sıfır komut gönder** - Kart hareket etmemeli
2. **Küçük değerlerle başla** - %10 throttle, 5° direksiyon
3. **Yavaş artır** - Her şey stabil ise artır
4. **Timeout'u test et** - Komut kesilirse kart durmalı

---

# 🎉 SONUÇ

Bu dokümanda öğrendiklerimiz:

1. **Vehicle Interface** - Autoware ile kart arasındaki köprü
2. **MIXED Kontrol** - Direksiyon Type A, Gaz/Fren Type B
3. **CAN Bus** - Kart ile iletişim protokolü
4. **ROS 2** - Topic publish/subscribe sistemi
5. **Docker** - İzole geliştirme ortamı
6. **Build Süreci** - colcon ile derleme

Başarılar! 🏎️🏆

---

*Oluşturulma tarihi: 4 Şubat 2026*
*SDC 2026 Yarışması için hazırlanmıştır.*
