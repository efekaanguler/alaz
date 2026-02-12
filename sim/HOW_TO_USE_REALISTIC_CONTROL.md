# Vehicle Control System (Servo Steering)

Bu modül, direkt kontrol sinyallerini alıp Autoware/CARLA için uygun kontrol mesajlarına çevirir. Direksiyon için **servo-benzeri dinamik davranış** sunar.

## 📋 Dosyalar

- **`pwm_to_control.py`** - Ana kontrol dönüştürücü (servo steering ile)
- **`test_control_publisher.py`** - Test script'i

## 🔌 Kontrol Girişleri

Modül şu topic'leri dinler:

| Topic | Mesaj Tipi | Aralık | Açıklama |
|-------|-----------|--------|----------|
| `/control/throttle` | `std_msgs/Float32` | 0.0 - 1.0 | Gaz pedalı (direkt normalize değer) |
| `/control/brake` | `std_msgs/Float32` | 0.0 - 1.0 | Fren pedalı (direkt normalize değer) |
| `/control/steering` | `std_msgs/Int16` | -255 - +255 | Direksiyon **hız** komutu |

## 📤 Çıkış

| Topic | Mesaj Tipi | Açıklama |
|-------|-----------|----------|
| `/carla/ego_vehicle/vehicle_control_cmd` | `CarlaEgoVehicleControl` | Araç kontrol komutu |

## 🎯 Servo Direksiyon Davranışı

### Önemli: Direksiyon Komutu = Dönüş HIZI

Normal sistemlerden farklı olarak, `/control/steering` topic'i **açı değil, açısal hız** komutu alır:

| Komut Değeri | Davranış |
|--------------|----------|
| **-255** | En hızlı sola dönüş |
| **-100** | Orta hızda sola dönüş |
| **-1** | Çok yavaş sola dönüş |
| **0** | Dönme durur, direksiyon merkeze döner |
| **+1** | Çok yavaş sağa dönüş |
| **+100** | Orta hızda sağa dönüş |
| **+255** | En hızlı sağa dönüş |

### Servo Dinamiği

```
Komut değeri (±255) → Dönüş hızı → Açı entegrasyonu → Gerçek direksiyon açısı
```

- Komut büyüdükçe direksiyon daha **hızlı** döner
- Komut 0 olunca direksiyon yavaşça **merkeze** döner
- Gerçekçi servo motor davranışı simüle edilir

## 🚀 Kullanım

### 1. Dönüştürücüyü Başlat

```bash
cd /home/efekaan/Desktop/alaz/2026/alaz/sim/bridge
python3 pwm_to_control.py
```

### 2. Test Komutları Gönder

**Düz gidiş (yarı gaz):**
```bash
python3 test_control_publisher.py --throttle 0.5 --steering 0
```

**Yavaş sola dönüş:**
```bash
python3 test_control_publisher.py --throttle 0.3 --steering -50
```

**Hızlı sola dönüş:**
```bash
python3 test_control_publisher.py --throttle 0.3 --steering -200
```

**Orta hızda sağa dönüş:**
```bash
python3 test_control_publisher.py --throttle 0.4 --steering 100
```

**Maksimum hızda sağa dönüş:**
```bash
python3 test_control_publisher.py --throttle 0.5 --steering 255
```

**Tam fren:**
```bash
python3 test_control_publisher.py --brake 1.0 --steering 0
```

### 3. Kendi Kodundan Kontrol Gönder

```python
from std_msgs.msg import Float32, Int16

# Throttle gönder (0.0-1.0)
throttle_msg = Float32()
throttle_msg.data = 0.5  # Yarı gaz
throttle_pub.publish(throttle_msg)

# Brake gönder (0.0-1.0)
brake_msg = Float32()
brake_msg.data = 0.0  # Fren yok
brake_pub.publish(brake_msg)

# Steering hız komutu gönder (-255 ile +255)
steering_msg = Int16()
steering_msg.data = -100  # Orta hızda sola dön
steering_pub.publish(steering_msg)
```

## ⚙️ Parametreler

```bash
python3 pwm_to_control.py --ros-args \
  -p max_steering_angle:=1.0 \
  -p steering_speed_scale:=0.02 \
  -p return_to_center_speed:=0.5 \
  -p steering_deadzone:=5 \
  -p timeout:=0.5 \
  -p publish_rate:=50.0
```

| Parametre | Varsayılan | Açıklama |
|-----------|-----------|----------|
| `max_steering_angle` | 1.0 | Maksimum direksiyon açısı (normalize, ±1.0) |
| `steering_speed_scale` | 0.02 | Direksiyon hız ölçekleme faktörü |
| `return_to_center_speed` | 0.5 | Merkeze dönüş hızı (komut 0 olunca) |
| `steering_deadzone` | 5 | Komut dead zone (±5 değerler yok sayılır) |
| `throttle_deadzone` | 0.02 | Gaz için dead zone (2%) |
| `brake_deadzone` | 0.02 | Fren için dead zone (2%) |
| `timeout` | 0.5 | Sinyal timeout süresi (saniye) |
| `publish_rate` | 50.0 | Yayın frekansı (Hz) |

### Parametre Ayarlama İpuçları

- **`steering_speed_scale` artır** → Direksiyon daha hızlı döner
- **`return_to_center_speed` artır** → Merkeze daha hızlı döner
- **`steering_deadzone` artır** → Gürültüye karşı daha dayanıklı

## 🔍 Debug ve İzleme

```bash
# Girişleri izle
ros2 topic echo /control/throttle
ros2 topic echo /control/brake
ros2 topic echo /control/steering

# Çıkışı izle (gerçek direksiyon açısını görmek için)
ros2 topic echo /carla/ego_vehicle/vehicle_control_cmd

# Manuel komut gönder
ros2 topic pub /control/steering std_msgs/msg/Int16 "data: 100"
ros2 topic pub /control/throttle std_msgs/msg/Float32 "data: 0.5"
```

## 🛡️ Güvenlik Özellikleri

- **Timeout koruması**: Sinyal kesilirse değerler otomatik sıfırlanır
- **Dead zone filtreleme**: Küçük gürültü değerleri yok sayılır  
- **Değer sınırlama**: Tüm değerler geçerli aralıklara otomatik sınırlandırılır
- **Merkeze dönüş**: Komut 0 olunca direksiyon güvenli şekilde merkeze döner

## 📊 Karşılaştırma: Eski vs Yeni Sistem

| Özellik | Eski Sistem | Yeni Sistem |
|---------|-------------|-------------|
| Direksiyon girişi | PWM (1000-2000 µs) | Hız komutu (-255 to +255) |
| Throttle/Brake | PWM dönüşümü | Direkt normalize değer |
| Davranış | Anında açı atama | Servo dinamiği (entegrasyon) |
| Gerçekçilik | Düşük (step function) | Yüksek (sürekli geçiş) |

## 💡 Arduino/Mikrodenetleyici Entegrasyonu

```cpp
// micro-ros örneği
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int16.h>

// Throttle gönder
std_msgs__msg__Float32 throttle_msg;
throttle_msg.data = 0.5;
rcl_publish(&throttle_pub, &throttle_msg, NULL);

// Steering hız komutu gönder
std_msgs__msg__Int16 steering_msg;
steering_msg.data = -100;  // Sola dön
rcl_publish(&steering_pub, &steering_msg, NULL);
```

## 🔗 İlgili Dosyalar

- [`keyboard_controls.py`](keyboard_controls.py) - Klavye kontrol sistemi
- [`speed_steer_topics.py`](speed_steer_topics.py) - Hız ve direksiyon republisher

