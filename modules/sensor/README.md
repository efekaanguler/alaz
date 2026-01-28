# ALAZ Sensor Kit (Lidar & Camera)

Bu paket, Go-Kart üzerindeki sensörlerin (Lidar, Kamera) **konumlarını (Extrinsic Calibration)** ve **konfigürasyonlarını** barındırır.

Otonom sürüş yığınının (stack), Haritalama (Mapping) ve Algılama (Perception) modüllerinin doğru çalışması için, sensörlerin araca göre nerede durduğunun (x, y, z, roll, pitch, yaw) **milimetrik hassasiyetle** bilinmesi gerekir.

---

## Paket İçeriği ve Kritik Dosyalar

* **`my_sensor_kit_description/config/sensors_calibration.yaml`**: **En Kritik Dosya.** Tüm sensörlerin `base_link`'e olan uzaklıkları ve açıları burada tutulur.
* **`my_sensor_kit_description/urdf/sensors.xacro`**: Sensörlerin 3D modellerini ve parent-child ilişkilerini (Hangi sensör nereye bağlı?) tanımlar.
* **`my_sensor_kit_description/launch/sensors.launch.xml`**: Sensörleri başlatan ana dosya.

---

## Sensör Envanteri

Bu kitte aşağıdaki sensörler tanımlanmıştır:

| Sensör Tipi | İsimlendirme (Frame ID) | Konum | Kullanım Amacı |
| :--- | :--- | :--- | :--- |
| **2D Lidar** | `lidar_link` | Ön Bar (Front Bar) | Engel tespiti & Haritalama (SLAM) |
| **Kamera 1** | `camera_center_link` | Orta (Center) | Şerit takibi & Trafik ışığı |
| **Kamera 2** | `camera_right_link` | Sağ (Right) | Çevre algılama |
| **Kamera 3** | `camera_left_link` | Sol (Left) | Çevre algılama |

---

## Sensör Değişikliği Rehberi (Calibration)

Eğer fiziksel araç üzerinde bir sensör sökülüp başka yere takılırsa veya açısı değiştirilirse, yazılım tarafında **mutlaka** aşağıdaki güncellemeler yapılmalıdır:

### Senaryo 1: Sensörün Yeri Değişti
Sadece kalibrasyon dosyasını güncellemeniz yeterlidir.

1. **Dosya:** `my_sensor_kit_description/config/sensors_calibration.yaml` dosyasını açın.
2. İlgili sensörün satırını bulun.
3. Değerleri güncelleyin:
   * **`x, y, z`**: Sensörün araca göre metre cinsinden mesafesi.
   * **`roll, pitch, yaw`**: Radyan cinsinden açısı. (+X ekseni ileri, +Y sola, +Z yukarı bakar).

### Senaryo 2: Yeni Sensör Eklendi
Hem model hem de kalibrasyon dosyası güncellenmelidir.

1. **Model:** `my_sensor_kit_description/urdf/sensors.xacro` içine yeni sensörün `<xacro:include>` ve `<joint>` tanımlarını ekleyin.
2. **Kalibrasyon:** `my_sensor_kit_description/config/sensors_calibration.yaml` dosyasına yeni sensör için x,y,z değerlerini ekleyin.

---

## Kritik Uyarı: Haritalama (Mapping) Etkisi

Mapping ekibi harita çıkarırken (`ros2 bag record`), buradaki `sensors_calibration.yaml` verilerini kullanır.

* **Eğer buradaki veriler yanlışsa (Örn: Lidar 10 derece eğik ama yazılımda düz görünüyor):**
  * Çıkarılan harita yamuk olur.
  * Araç kendini haritada konumlandıramaz (Localization Failure).
  * **Sonuç:** Otonom sürüş çalışmaz.

Bu yüzden harita toplamaya çıkmadan önce **mezura ile ölçüm yapıp** bu dosyayı doğrulayın.

---

## Görsel Kontrol (Debug)

Sensörlerin doğru yere "monte edildiğini" simülasyon ortamında doğrulamak için:

1. **Simülasyonu başlatın.**
2. **RViz** açın.
3. Sol panelden **TF** kutucuğunu işaretleyin.
4. Ağacı genişletin (`base_link` -> `sensor_kit_base_link` -> ...). ***_link formatında eklediğiniz sensörleri burada görebiliyor olmanız gerekmektedir.


## Nasıl Derlenir?

Sensör konumlarında en ufak bir değişiklik yaptıktan sonra (bir sayıyı bile değiştirseniz) paketi tekrar derlemeniz gerekir:

```bash
cd ~/autoware

# 1. Temiz derleme
colcon build --symlink-install --packages-select my_sensor_kit_description my_sensor_kit_launch --cmake-clean-cache

# 2. Ortamı güncelle
source install/setup.bash