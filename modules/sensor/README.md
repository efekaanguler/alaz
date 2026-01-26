# ALAZ Sensor Kit (Lidar & Camera)

Bu paket, Go-Kart üzerindeki sensörlerin (Lidar, Kamera) **konumlarını (Extrinsic Calibration)** ve **konfigürasyonlarını** barındırır.

Otonom sürüş yığınının (stack) doğru çalışması için sensörlerin araca göre nerede durduğunun (x, y, z, roll, pitch, yaw) milimetrik olarak bilinmesi gerekir.


## Sensör Yerleşimi

Bu kitte aşağıdaki sensörler tanımlanmıştır:

* **Lidar:** 2D Lidar (ön bar montajlı) -> `lidar_link`
* **Kamera-1:** Orta görüş kamerası -> `camera_center_link`
* **Kamera-2:** Sağ görüş kamerası -> `camera_right_link`
* **Kamera-3:** Sol görüş kamerası -> `camera_left_link`


## Kalibrasyon Ayarları (TF Tree)

Sensörlerin konumu değişirse (örneğin Lidar sökülüp 5 cm öne takılırsa), **mutlaka** aşağıdaki dosya güncellenmelidir:

* **Dosya:** `config/sensors_calibration.yaml` (veya `sensors.launch.xml`)
* **Parametreler:**
    * `x`, `y`, `z`: Metre cinsinden mesafe.
    * `roll`, `pitch`, `yaw`: Radyan cinsinden açı.


## Görsel Kontrol (Debug)

Sensörlerin doğru yere "monte edildiğini" simülasyon ortamında doğrulamak için:

1. **Simülasyonu başlatın.**
2. **RViz** açın.
3. Sol panelden **TF** kutucuğunu işaretleyin.
4. Ağacı genişletin.
5. **Kontrol:** Çıkan listede bulunan bağlantılar yukarıda belirtilen sensörler olarak bulunmalı.


## Nasıl Derlenir?

Sensör kitinde bir değişiklik yaptıktan sonra (örn. yeni kamera eklendi), paketi tekrar derlemeniz gerekir:

```bash
cd ~/autoware
colcon build --symlink-install --packages-select my_sensor_kit_description my_sensor_kit_launch
source install/setup.bash