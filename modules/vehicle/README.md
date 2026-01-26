# ALAZ Go-Kart Vehicle Description

Bu paket, **RDW Self-Driving Challenge** için kullanılan standart elektrikli Go-Kart şasisinin fiziksel özelliklerini, 3D modellerini ve kontrol parametrelerini içerir.

## Paket İçeriği

* **`config/vehicle_info.yaml`**: **En Kritik Dosya.** Aracın tekerlek aralığı (wheelbase), tekerlek çapı, genişliği ve dönüş yarıçapı limitleri burada tanımlıdır.
* **`urdf/`**: Aracın 3D görsel modeli (Mesh dosyaları) ve fiziksel eklemleri. (Şimdilik varsayılan araç olan Lexus)
* **`mesh/`**: `.dae` veya `.stl` formatındaki araç kaporta çizimleri.

## Teknik Parametreler (RDW Standartları)

Eğer araç üzerinde fiziksel bir değişiklik yapılırsa (örn. lastik basıncı değişimiyle tekerlek çapı değişirse), aşağıdaki parametrelerin güncellenmesi gerekir:

| Parametre | Açıklama | Dosya Konumu |
| :--- | :--- | :--- |
| **`wheel_base`** | Ön-Arka aks mesafesi | `config/vehicle_info.yaml` |
| **`wheel_tread`** | Sağ-Sol tekerlek mesafesi | `config/vehicle_info.yaml` |
| **`max_steer_angle`** | Maksimum direksiyon açısı | `config/vehicle_info.yaml` |

## Nasıl Derlenir?

Araç üzerinde bir değişiklik yaptıktan sonra (örn. aks açıklığı değiştirildi), paketi tekrar derlemeniz gerekir:

```bash
cd ~/autoware
colcon build --symlink-install --packages-select my_vehicle_description my_vehicle_launch
source install/setup.bash