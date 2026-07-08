# 📍 Localization Module

Aracın harita üzerindeki konumunu **YabLoc** (Visual Localization) algoritması ile belirler.

---

## ⚠️ ENTEGRASYON - DÜZENLENMELİ !!!

### 1️⃣ localization.yaml - Topic İsimlerini Ayarla

**📍 Dosya:** `config/localization.yaml`

```yaml
localization:
  topics:
    # ⚠️ Kendi kamera topic'inizi yazın
    camera: /sensing/camera/camera0/image_raw              # Örnek: /sensing/camera/camera0/image_raw
    
    # ⚠️ Kendi odometri topic'inizi yazın  
    wheel_odom: /vehicle/odometry              # Örnek: /localization/kinematic_state
```

### 2️⃣ Frame İsimlerini Ayarla

```yaml
  frames:
    map: map              # Harita frame
    odom: odom            # Odometri frame
    base_link: base_link  # Araç merkezi frame
```

---

## 📖 Modül Hakkında

### YabLoc Nasıl Çalışır?
1. **Kamera** yol çizgilerini görür
2. **Odometri** hız ve dönüş bilgisi verir  
3. **YabLoc** görüntüyü harita ile eşleştirir
4. Sonuç: Araç pozu (x, y, yaw)

### Veri Akışı
```
Kamera (/sensing/camera/camera0/image_raw) ──┐
                                  ├──> YabLoc ──> Pose (/localization/pose_estimator/pose)
Odometri (/vehicle/odometry) ────┘
```

---

## 📂 Dosya Yapısı

```
localization/
├── launch/
│   └── localization.launch.py      # Ana launch dosyası
├── config/
│   ├── localization.yaml           # ✅ DÜZENLE: Topic/frame ayarları
│   └── yabloc.param.yaml           # İsteğe bağlı: Algoritma parametreleri
└── README.md
```

| Dosya | Düzenlenir mi? | Ne zaman? |
|-------|----------------|-----------|
| **localization.yaml** | ✅ **Evet** | İlk kurulum - Topic/frame isimleri |
| **yabloc.param.yaml** | ⚙️ İsteğe bağlı | Performans tuning gerekirse |
| **localization.launch.py** | ❌ Hayır | Kod değişikliği gerekmedikçe |

---

## 🚀 Kullanım

### Normal Kullanım (Önerilen)

```bash
ros2 launch global_bringup global.launch.py
```

global_bringup otomatik olarak localization'ı başlatır.

### Config Test

```bash
ros2 launch localization localization.launch.py
```

Sadece config'leri okur ve logları basar.

---

## 🔧 Performans Ayarları (İsteğe Bağlı)

`config/yabloc.param.yaml` dosyasında yapılır:

| Parametre | Etki | Ne Zaman Değiştir |
|-----------|------|-------------------|
| `max_match_distance` | Eşleştirme toleransı | Lokalizasyon hassasiyeti |
| `min_match_score` | Güvenilirlik eşiği | Yanlış eşleştirmelerde |
| `image_processing_rate_hz` | İşleme hızı | CPU yükü fazlaysa |
| `downsample_ratio` | Görüntü küçültme | Performans için |

**Hızlı Ayarlar:**
```yaml
# Hassas ama yavaş
max_match_distance: 1.0
min_match_score: 0.7

# Hızlı ama az hassas
max_match_distance: 2.5
min_match_score: 0.4
downsample_ratio: 4
```

---

## 🔍 Sorun Giderme

### Kamera Gelmiyor
```bash
ros2 topic hz /sensing/camera/camera0/image_raw
ros2 topic echo /sensing/camera/camera0/image_raw --once
```
**Çözüm:** `localization.yaml`'da `camera` topic'ini düzeltin.

### Odometri Gelmiyor
```bash
ros2 topic hz /vehicle/odometry
```
**Çözüm:** `localization.yaml`'da `wheel_odom` topic'ini düzeltin.

### YabLoc Pose Üretmiyor
```bash
ros2 node list | grep yabloc
ros2 topic echo /localization/pose_estimator/pose
```
**Çözüm:** 
1. Kamera ve odometri girdilerini kontrol edin
2. Harita dosyasının yüklendiğini kontrol edin
3. global_bringup log'larına bakın

### Debug Modu
`yabloc.param.yaml`:
```yaml
publish_debug_markers: true
publish_match_image: true
```

---

## 🔗 Modül İlişkileri

| Modül | İlişki |
|-------|--------|
| **global_bringup** | Bu modülü başlatır |
| **sensing** | Kamera sağlar |
| **vehicle** | Odometri sağlar |
| **Autoware** | YabLoc node'larını çalıştırır |

**Başlatma Sırası:** sensing → vehicle → global_bringup → YabLoc

---

## 📋 Özet

### ✅ Bu Modül:
- Lokalizasyon yapılandırmasını yönetir
- Topic/frame mapping yapar
- global_bringup ile entegre çalışır

### ❌ Bu Modül:
- Kendi node'larını başlatmaz
- YabLoc'u çalıştırmaz (Autoware yapar)
- Bağımsız çalışmaz

**Referans:** [YabLoc Repository](https://github.com/autowarefoundation/autoware.universe)