# ALAZ Go-Kart Vehicle Description

Bu paket, **RDW Self-Driving Challenge** için kullanılan standart elektrikli Go-Kart şasisinin fiziksel özelliklerini, 3D modellerini ve kontrol parametrelerini içerir.

Simülasyon, Planlama (Planning) ve Kontrol (Control) algoritmaları, aracın sınırlarını ve yapısını buradaki dosyalardan okur.


## Paket İçeriği

* **`my_vehicle_description/config/vehicle_info.param.yaml`**: **En Kritik Dosya.** Aracın tekerlek aralığı (wheelbase), tekerlek çapı, genişliği ve tampon mesafeleri burada tanımlıdır.
* **`my_vehicle_description/urdf/vehicle.xacro`**: Aracın parçalarının birbirine nasıl bağlandığını (Tekerlek şasiye nereden bağlı? Lidar nereye takılacak?) tanımlayan dosya.
* **`my_vehicle_description/mesh/`**: Aracın simülasyonda görünen 3D çizimleri (`.dae`, `.stl` veya `.fbx`).


## Araç Değişirse Ne Yapılmalı?

Eğer şasi değişirse, lastik ebatları yenilenirse veya sensörlerin montaj yeri değişirse aşağıdaki adımları sırasıyla uygulayın:

### 1. Fiziksel Ölçüleri Güncelleyin
Dosya: `my_vehicle_description/config/vehicle_info.param.yaml`

| Parametre | Ne Zaman Değişir? | Neden Önemli? |
| :--- | :--- | :--- |
| **`wheel_base`** | Şasi uzatılır/kısaltılırsa. | Dönüş yarıçapını ve MPC kontrolcüsünü bozar. Yanlışsa araç virajı alamaz. |
| **`wheel_radius`** | Lastik değişir veya basınç değişirse. | **Hız verisi (Odometry) hatalı hesaplanır.** |
| **`wheel_tread`** | Aks genişliği değişirse. | Aracın dar alanlardan geçip geçemeyeceğini belirler. |
| **`front_overhang`** | Ön tampon değişirse. | Çarpışma algılama ve durma mesafesi için kritiktir. |

### 2. Görsel Modeli Güncelleyin (Opsiyonel)
Dosya: `urdf/vehicle.xacro`

Eğer aracın görüntüsü değiştiyse:
1. Yeni `.dae` dosyasını `mesh/` klasörüne atın.
2. `vehicle.xacro` içindeki `<mesh filename="package://..." />` satırını yeni dosya ismiyle değiştirin.


## Kritik Uyarı: Kontrolcü (Controller) Etkisi

Burada yapacağınız **`wheel_base` (Aks mesafesi)** değişikliği, otonom sürüş kontrolcüsünü (MPC/PID) doğrudan etkiler.

* **Eğer `wheel_base` değerini değiştirirseniz:**
  * Kontrol parametrelerini (`modules/alaz_control_config` altındaki PID/MPC ayarlarını) tekrar **Tune** etmeniz gerekebilir.
  * Aksi takdirde araç yolda "yılan gibi" (oscillation) gidebilir.


## Nasıl Derlenir? (Build)

Parametrelerde (yaml) veya modelde (urdf) en ufak bir değişiklik yaptıktan sonra paketi **mutlaka** tekrar derlemelisiniz. Aksi takdirde Autoware eski ayarları kullanmaya devam eder.

```bash
cd ~/autoware

# 1. Eski paket kalıntılarını temizleyin (Önerilir)
colcon build --packages-select my_vehicle_description --cmake-clean-cache

# 2. Yeni ayarlarla derleyin
colcon build --symlink-install --packages-select my_vehicle_description my_vehicle_launch

# 3. Ortamı güncelleyin
source install/setup.bash