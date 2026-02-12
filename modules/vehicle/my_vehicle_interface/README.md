# my_vehicle_interface

Autoware vehicle interface for the Self Driving Challenge 2026 kart with CAN support using ros2_socketcan.

## Overview

This package bridges Autoware's control commands with the SDC 2026 kart's CAN-controlled actuators:
- **Steering**: IEEE 754 float encoding → servo motor (CAN ID: `0x220`)
- **Throttle**: 0-100% + gear byte → electric motor (CAN ID: `0x330`)
- **Brake**: 0-100% → linear actuator (CAN ID: `0x110`)

---

## ⚠️ KRİTİK UYARI: Byte Offset (Decoder Hatası İhtimali)

**DİKKAT:** `src/can_utils.cpp` içindeki decoder fonksiyonları, Steering Sensor (0x1E5) hariç, verilerin `data[0]`'dan başladığını varsayar (Wiki standardı).
Ancak, Steering Sensor'de olduğu gibi (Data[1]-Data[2]), diğer ECU'larda da dokümante edilmemiş bir "Status/Counter Byte" (Byte 0) olabilir.

**Eğer arabadan gelen veriler (Hız, Motor Feedback) anlamsızsa (örn: çok yüksek/düşük):**
`src/can_utils.cpp` dosyasını kontrol edin ve byte indekslerini 1 kaydırarak deneyin (örn: `data[0]` yerine `data[1]`).

---

## Wiki Sonrası Yapılan Değişiklikler

### ✅ Tamamlanan İşler

#### CAN ID Düzeltmeleri
| Mesaj | Eski (Yanlış) | Yeni (Wiki) |
|-------|:---:|:---:|
| Steering Command | `0x100` | **`0x220`** |
| Brake Command | `0x101` | **`0x110`** |
| Motor Command | `0x102` | **`0x330`** |
| Speed Sensor | `0x200` | **`0x440`** |

#### Encoding Düzeltmeleri
- **Steering:** Integer (0.1° scale) → **IEEE 754 float** (`memcpy`, range: -1.25 to 1.25)
- **Motor:** Sadece throttle → **throttle (byte 0) + gear (byte 2)** (0=N, 1=F, 2=R)
- **Speed Sensor:** Little-endian → **Big-endian** uint16

#### Yeni Eklenen Feedback Decoder'lar
| CAN ID | İsim | Okunan Veri |
|--------|------|-------------|
| `0x1E5` | Steering Sensor | Direksiyon açısı (int16, bytes 1-2) |
| `0x720` | Steering ECU FB | Mevcut açı + hedef açı + hata flag'i |
| `0x730` | Motor ECU FB | Throttle DAC + fren + vites + idle |
| `0x710` | Brake ECU FB | (Log amaçlı) |

#### Güvenlik Mekanizmaları
- Motor idle algılama (200ms timeout)
- Steering ECU failsafe uyarısı
- Gear clamping (>= 3 mesajı geçersiz kılar)
- Command timeout (0.2s) → sıfır komut gönderme

#### Bulunan ve Düzeltilen Bug'lar
- **Steering Sensor byte offset:** `data[0-1]` → **`data[1-2]`** (`data_recorder.py`'dan doğrulandı)

### ✅ Test Edilenler (79/79 PASS)
Test scripti: `test/test_wiki_verification.py`

- Tüm CAN ID'leri wiki ile eşleşiyor
- IEEE 754 float encoding doğru (wiki örnekleri ile birebir)
- Motor byte layout doğru (motor_demo.py ile birebir)
- Speed sensor big-endian decoding doğru
- Steering sensor byte offset doğru (data_recorder.py ile birebir)
- Timing: 25 Hz loop (wiki: 0.04s), 0.2s timeout (motor ECU: 200ms)
- Autoware topic'leri doğru bağlanmış

---

## ❗ Hala Yapılması Gerekenler

### 1. Docker'da Build ve Test
```bash
cd /workspace
colcon build --packages-select my_vehicle_interface
source install/setup.bash
# Test:
cd src/vehicle/external/my_vehicle_interface
./test/run_wiki_tests.sh --build
```

### 2. Arabada Kalibrasyon (ZORUNLU)

| Parametre | Dosya | Varsayılan | Nasıl Ölçülecek |
|-----------|-------|:---:|------|
| `max_steering_angle_rad` | `param.yaml` | `0.5236` (30°) | Direksiyonu sonuna çevir, açıölçer ile gerçek açıyı ölç |
| `accel_to_throttle_gain` | `param.yaml` | `0.33` | Throttle=50% gönder, hızlanma oranını ölç, gain'i ayarla |
| `decel_to_brake_gain` | `param.yaml` | `0.20` | Brake=50% gönder, yavaşlama oranını ölç, gain'i ayarla |
| Steering sensor sıfır noktası | Kod | `raw / 800` | Düz gittiğinde raw=0 mı kontrol et, değilse offset ekle |

### 3. Arabada CAN Bağlantı Testi
```bash
# 1) CAN interface aç
slcan_attach -f -s6 -o /dev/TinCan
slcand -F /dev/TinCan can0
ip link set up can0

# 2) CAN trafiğini izle
candump can0

# 3) Gelen feedback mesajlarını kontrol et:
#    0x440 → Speed sensor (geliyor mu?)
#    0x1E5 → Steering sensor (geliyor mu?)
#    0x720 → Steering ECU (geliyor mu?)
#    0x730 → Motor ECU (geliyor mu?)
```

### 4. İlk Sürüş Testi Adımları
1. **Mode 1** ile başla (max 10 km/h)
2. Acil stop cihazı (Tyro Indus 1S) el altında olsun
3. Önce sadece direksiyon testi (throttle=0, brake=0)
4. Sonra düşük throttle testi (%10-20)
5. Fren testi (düşük hızda)
6. Tüm feedback mesajlarının geldiğini doğrula

### 5. Opsiyonel İyileştirmeler
- [ ] Steering speed desteği (DLC=8, servo hızı kontrolü)
- [ ] Brake feedback (0x710) decoder eklenmesi
- [ ] PID controller entegrasyonu (daha hassas kontrol)
- [ ] Diagnostik topic'leri (motor DAC, ECU hata durumları)

---

## Prerequisites

- ROS 2 Humble
- Autoware
- [ros2_socketcan](https://github.com/autowarefoundation/ros2_socketcan)

## Usage

### With Real Hardware

```bash
# Terminal 1: Start CAN bridge
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0

# Terminal 2: Start vehicle interface
ros2 launch my_vehicle_interface vehicle_interface.launch.xml
```

## Topics

### Subscribed (from Autoware)
| Topic | Message Type |
|-------|--------------|
| `/control/command/control_cmd` | `Control` |
| `/control/command/gear_cmd` | `GearCommand` |
| `/from_can_bus` | `can_msgs/Frame` |

### Published (to Autoware)
| Topic | Message Type |
|-------|--------------|
| `/vehicle/status/velocity_status` | `VelocityReport` |
| `/vehicle/status/steering_status` | `SteeringReport` |
| `/vehicle/status/gear_status` | `GearReport` |
| `/vehicle/status/control_mode` | `ControlModeReport` |
| `/to_can_bus` | `can_msgs/Frame` |

## CAN Protocol (SDC Wiki)

### Komutlar (Autoware → Kart)
| ID | İsim | Format |
|:---:|------|--------|
| `0x220` | Steering | IEEE 754 float, DLC=4, range: -1.25 to 1.25 |
| `0x330` | Motor | byte[0]=throttle(0-100), byte[2]=gear(0/1/2) |
| `0x110` | Brake | byte[0]=brake(0-100) |

### Feedback (Kart → Autoware)
| ID | İsim | Format |
|:---:|------|--------|
| `0x440` | Speed Sensor | Big-endian uint16, hm/h |
| `0x1E5` | Steering Sensor | Big-endian int16, bytes[1-2], range: -800 to 800 |
| `0x720` | Steering ECU | Current/target angle + error flag |
| `0x730` | Motor ECU | Throttle DAC + brake + gear + idle |

## File Structure

```
my_vehicle_interface/
├── include/my_vehicle_interface/
│   ├── vehicle_interface_node.hpp
│   └── can_utils.hpp
├── src/
│   ├── vehicle_interface_node.cpp
│   ├── can_utils.cpp
│   └── main.cpp
├── launch/
│   └── vehicle_interface.launch.xml
├── config/
│   └── vehicle_interface.param.yaml
├── test/
│   ├── test_wiki_verification.py
│   └── run_wiki_tests.sh
├── CMakeLists.txt
├── package.xml
└── README.md
```

## License

Apache-2.0
