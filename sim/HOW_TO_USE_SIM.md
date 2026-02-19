# Simülasyon Kullanım Kılavuzu

Bu kılavuz, CARLA simülasyonunu Autoware ile çalıştırmak için adım adım talimatlar içerir.

## 📋 Genel Bakış

1. Container'ları başlat
2. CARLA sunucusunu çalıştır
3. CARLA-ROS Bridge'i kur ve başlat
4. sim_pkg paketini build et ve launch et
5. Simülasyonu kullan
6. Bitirdiğinde container'ları kapat

---

## 🚀 Adım Adım Kullanım

### 1️⃣ Container'ları Başlat

```bash
cd ~/Desktop/alaz/2026/alaz/sim

# Container'ları başlat
./sim_start_container.sh

../dev_run.sh
```

---

### 2️⃣ CARLA Sunucusunu Başlat

```bash
# CARLA container'a gir
docker exec -it carla_container bash

# scenario klasörüne git ve sunucuyu başlat
cd /scenario
./sim_server_start.sh
```

**Bekle**: CARLA sunucusu başlayana kadar bekle (birkaç saniye sürer).

---

### 3️⃣ CARLA-ROS Bridge Kurulumu (Autoware Container)

Yeni bir terminal aç:

```bash
# Autoware container'a gir
docker exec -it autoware_container bash

# bridge klasörüne git
cd /bridge

# Bridge'i kur (ilk sefer için)
./bridge_setup.sh

# ROS workspace'i source et
source sourcing.sh

# Bridge'i başlat
./bridge_start.sh
```

**Başarılı olursa**: Bridge bağlantısı kuruldu ve CARLA ile ROS arasında veri akışı başladı.

---

### 4️⃣ sim_pkg Paketini Build Et (Autoware Container)

Yeni bir terminal aç:

```bash
# Autoware container'a gir
docker exec -it autoware_container bash

# Build script'ini çalıştır
# Sourcela
```
**Build ve source işlemini alaz/ altından yapabilirsin**
**Build başarılı olduysa**: Paket kullanıma hazır.

---

### 5️⃣ sim_pkg Paketini Launch Et

Aynı terminal'de:

```bash
# Default: Keyboard kontrolü OLMADAN
ros2 launch sim_pkg sim_pkg.launch.py
```

**Veya Keyboard kontrolü ile:**

```bash
ros2 launch sim_pkg sim_pkg.launch.py keyboard:=true
```

#### 🎮 Launch Edilen Node'lar

| Node | Açıklama | Keyboard=false | Keyboard=true |
|------|----------|----------------|---------------|
| `pointcloud_to_laserscan` | 3D point cloud → 2D laser scan | ✅ | ✅ |
| `speed_steer_topics` | Hız ve direksiyon bilgisi yayınlar | ✅ | ✅ |
| `realistic_controls` | PWM kontrol dönüştürücü | ✅ | ✅ |
| `keyboard_controls` | Klavye kontrolü (W/A/S/D) | ❌ | ✅ |

#### 🔍 Kontrol Et

```bash
# Çalışan node'ları listele
ros2 node list

# Topic'leri görüntüle
ros2 topic list

# Belirli bir topic'i izle
ros2 topic echo /carla/ego_vehicle/vehicle_status
```

---

### 6️⃣ Simülasyonu Sıfırla (İsteğe Bağlı)

Simülasyonu yeniden başlatmak için yeni terminal:

```bash
# CARLA container'a gir
docker exec -it carla_container bash

# Scenario loader ile sıfırla
cd /scenario
python3 scenario_loader.py
```

**Kullanım**: Simülasyon sırasında her an çalıştırabilirsin.

---

### 7️⃣ Ek Terminal'ler Açma

Geliştirme veya debug için daha fazla terminal açabilirsin:

```bash
# Autoware için yeni terminal
docker exec -it autoware_container bash

# CARLA için yeni terminal
docker exec -it carla_container bash
```

---

## 🛑 Simülasyonu Sonlandırma

### İşiniz Bittiğinde

```bash
cd ~/Desktop/alaz/2026/alaz/sim

# Container'ları kapat ve temizle
./sim_remove_container.sh
```

**Uyarı**: Bu komut çalışan tüm container'ları kapatır ve veriler kaybolur.

---

## 🎯 Hızlı Başlangıç (Özet)

```bash
# 1. Container'ları başlat
./sim_start_container.sh

# Terminal 1 - CARLA Sunucu
docker exec -it carla_container bash
cd /scenario && ./sim_server_start.sh

# Terminal 2 - Bridge
docker exec -it autoware_container bash
cd /bridge && ./bridge_setup.sh && source sourcing.sh && ./bridge_start.sh

# Terminal 3 - sim_pkg
docker exec -it autoware_container bash
./build_simpkg.sh
source ~/Workspace/ros-bridge/install/setup.bash
ros2 launch sim_pkg sim_pkg.launch.py

# İş bitince
./sim_remove_container.sh
```

---

## 🔧 Sorun Giderme

### Bridge Bağlanamıyor

```bash
# CARLA sunucusunun çalıştığını kontrol et
docker exec -it carla_container bash
ps aux | grep CarlaUE4

# Bridge'i yeniden başlat
cd /bridge && ./bridge_start.sh
```

### sim_pkg Build Hatası

```bash
# Workspace'i temizle ve tekrar build et
cd ~/Workspace/ros-bridge
rm -rf build/sim_pkg install/sim_pkg
colcon build --packages-select sim_pkg
source install/setup.bash
```

### Node'lar Başlamıyor

```bash
# ROS workspace source edildi mi kontrol et
echo $AMENT_PREFIX_PATH

# Yoksa tekrar source et
source ~/Workspace/ros-bridge/install/setup.bash
```

---

## 📚 İlgili Dökümanlar

- [Realistic Control Kullanımı](HOW_TO_USE_REALISTIC_CONTROL.md)
- [Bridge README](bridge/README.md)
- [Scenario Konfigürasyonu](scenario/default.yaml)

---

## 💡 İpuçları

- **CARLA sunucusu** her zaman ilk başlatılmalı
- **Bridge** ikinci sırada başlatılmalı
- **sim_pkg** son olarak launch edilmeli
- **Keyboard kontrolü** için `keyboard:=true` parametresini kullan
- **Simülasyonu sıfırla** scenario_loader ile
- **Debug** için `ros2 topic echo` kullan
