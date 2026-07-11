# 📍 Localization Module

Aracın harita üzerindeki konumunu **YabLoc** (Visual Localization) algoritması ile belirler.

---

## 📖 Modül Hakkında

### YabLoc Nasıl Çalışır?
1. **Kamera** yol çizgilerini görür
2. **Odometri** hız ve dönüş bilgisi verir  
3. **YabLoc** görüntüyü harita ile eşleştirir
4. Sonuç: Araç pozu (x, y, yaw)

### Veri Akışı
```
Kamera (global topics.yaml'dan) ──┐
                                  ├──> YabLoc ──> Pose (/localization/pose_estimator/pose)
Odometri (/odom) ─────────────────┘
```

---

## 📂 Dosya Yapısı

```
localization/
├── launch/
│   └── localization.launch.py      # Ana launch dosyası
├── map_projector_info_pub.py       # Harita projeksiyon bilgisini yayınlar
├── odom_to_twist_cov.py            # Odometriyi TwistWithCovariance'a çevirir
├── initial_pose_pub.py             # İlk poz yayınlayıcı
└── README.md
```

## 🚀 Kullanım

```bash
ros2 launch global_bringup global.launch.py
```

`global_bringup` otomatik olarak `localization`'ı başlatır.

### Girdiler ve Yapılandırma
Lokalizasyon modülü konfigürasyonunu `global_bringup/config/bringup.yaml` ve `topics.yaml` üzerinden alır.
- Harita yolu `bringup.yaml` dosyasından `lanelet2_map_path` argümanı ile iletilir.
- Kamera topicleri `topics.yaml` dosyasından otomatik olarak çekilir.
- İlk poz (initial pose) RViz2 üzerinden `2D Pose Estimate` kullanılarak verilmelidir.

---

## 🔗 Modül İlişkileri

| Modül | İlişki |
|-------|--------|
| **global_bringup** | Bu modülü başlatır |
| **perception/sensing** | Kamera sağlar |
| **odometry** | Odometri sağlar |

**Referans:** [YabLoc Repository](https://github.com/autowarefoundation/autoware.universe)
