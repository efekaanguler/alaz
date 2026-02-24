# RDW Sensor Kit (2D Lidar & 3 Kamera)

Bu paket, **rdw_vehicle** aracı üzerindeki sensörlerin (2D Lidar, 3x Kamera) **konumlarını (Extrinsic Calibration)**, **konfigürasyonlarını** ve **başlatma (launch)** dosyalarını barındırır.

Otonom sürüş yığınının (stack), Haritalama (Mapping) ve Algılama (Perception) modüllerinin doğru çalışması için, sensörlerin araca göre nerede durduğunun (x, y, z, roll, pitch, yaw) **milimetrik hassasiyetle** bilinmesi gerekir.

## Paket İçeriği ve Kritik Dosyalar

Autoware standartlarına uygun olarak bu kit iki ana paketten oluşur:

### 1. `rdw_sensor_kit_description` (Tanımlamalar)
*   **`config/sensor_kit_calibration.yaml`**: **En Kritik Dosya.** Sensörlerin birbirine göre olan uzaklıkları ve açıları burada tutulur.
*   **`urdf/sensor_kit.xacro`**: Sensörlerin 3D modellerini, parent-child ilişkilerini ve `sensor_kit_calibration.yaml` dosyasından verilerin nasıl okunacağını tanımlar.

### 2. `rdw_sensor_kit_launch` (Başlatma)
*   **`launch/sensing.launch.xml`**: Tüm sensörleri (Lidar, Kamera, GNSS) başlatan ve veri akışını yöneten ana dosya.
*   **`launch/lidar.launch.xml`**: 2D Lidar sürücüsünü (`urg_node` vb.) başlatır ve veriyi 3D PointCloud'a çevirir.
*   **`launch/camera.launch.xml`**: 3 adet USB kamerayı başlatır.
*   **`config/sensors_calibration.yaml`**: Tüm sensör kitinin (`sensor_kit_base_link`) aracın şasesine (`base_link`) göre nerede durduğunu tanımlar.

## Sensör Envanteri

Bu kitte aşağıdaki sensörler tanımlanmıştır:

| Sensör Tipi | İsimlendirme (Frame ID) | Konum | Kullanım Amacı |
| :--- | :--- | :--- | :--- |
| **2D Lidar** | `lidar_link` | Ön/Tavan (Front/Top) | Engel tespiti, Haritalama (SLAM) |
| **Kamera 1** | `camera_center_link` | Orta (Center) | Nesne algılama & Şerit takibi |
| **Kamera 2** | `camera_left_link` | Sol (Left) | Çevre algılama (Kör nokta) |
| **Kamera 3** | `camera_right_link` | Sağ (Right) | Çevre algılama (Kör nokta) |

## Sensör Değişikliği Rehberi (Calibration)

Eğer fiziksel araç üzerinde bir sensör sökülüp başka yere takılırsa veya açısı değiştirilirse, yazılım tarafında **mutlaka** aşağıdaki güncellemeler yapılmalıdır.

### Senaryo 1: Sensörlerin Birbirine Göre Yeri Değişti
Örneğin, sol kamerayı biraz daha sola kaydırdınız.

1.  **Dosya:** `rdw_sensor_kit_description/config/sensor_kit_calibration.yaml` dosyasını açın.
2.  İlgili sensörün (`camera_left_link` vb.) satırını bulun.
3.  Değerleri güncelleyin:
    *   **`x, y, z`**: Sensör kiti merkezine göre metre cinsinden mesafe.
    *   **`roll, pitch, yaw`**: Radyan cinsinden açı. (+X ekseni ileri, +Y sola, +Z yukarı bakar).

### Senaryo 2: Tüm Kitin Araca Göre Yeri Değişti
Örneğin, sensörlerin takılı olduğu tavan barını komple 10 cm öne aldınız.

1.  **Dosya:** `rdw_sensor_kit_description/config/sensors_calibration.yaml` dosyasını açın.
2.  `base_link` -> `sensor_kit_base_link` dönüşümünü güncelleyin.

## Kritik Uyarı: Haritalama (Mapping) Etkisi

Mapping ekibi harita çıkarırken (`ros2 bag record`), buradaki kalibrasyon verilerini kullanır.

*   **Eğer buradaki veriler yanlışsa (Örn: Lidar 10 derece eğik ama yazılımda düz görünüyor):**
    *   Çıkarılan harita yamuk olur.
    *   Araç kendini haritada konumlandıramaz (Localization Failure).
    *   Lidar verisi yer düzlemi (ground) ile yanlış çakışır, Autoware yeri "engel" sanabilir.
    *   **Sonuç:** Otonom sürüş çalışmaz.

Bu yüzden harita toplamaya çıkmadan önce **mezura ile ölçüm yapıp** bu dosyayı doğrulayın.

## Görsel Kontrol (Debug)

Sensörlerin doğru yere "monte edildiğini" simülasyon ortamında doğrulamak için:

1.  Paketi derleyin ve ortamı kaynaklayın.
2.  Launch dosyasını çalıştırın:
    ```bash
    ros2 launch rdw_sensor_kit_launch sensing.launch.xml
    ```
3.  **RViz** açın (`rviz2`).
4.  Global Frame olarak `base_link` seçin.
5.  Sol panelden **TF** kutucuğunu işaretleyin.
6.  Ağacı genişletin (`base_link` -> `sensor_kit_base_link` -> ...). `lidar_link` ve `camera_..._link` frame'lerinin araç modelinin üzerinde doğru noktalarda durduğunu teyit edin.

## Nasıl Derlenir?

Sensör konumlarında veya parametrelerinde en ufak bir değişiklik yaptıktan sonra paketi tekrar derlemeniz önerilir:

```bash
cd ~/autoware

# 1. Temiz derleme (Önerilen)
colcon build --symlink-install --packages-select rdw_sensor_kit_description rdw_sensor_kit_launch --cmake-clean-cache

# 2. Ortamı güncelle
source install/setup.bash
```
