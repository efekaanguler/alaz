# ALAZ Otonom Araç Modelleri (Teknofest & RDW)

Bu paket, takımımızın katıldığı farklı yarışmalar için hazırlanan araçların fiziksel özelliklerini, 3D modellerini ve sınırlarını barındırır. Autoware simülasyon, planlama ve kontrol modülleri, hangi aracı seçerseniz o aracın fiziksel limitlerine (dönüş çapı, genişlik vb.) göre hareket eder.

Şu anda filomuzda iki farklı araç tanımlıdır:

1.  **`my_vehicle` (Teknofest Aracı):** Takımın geliştirdiği özgün araç.
2.  **`rdw_vehicle` (RDW Challenge Aracı):** RDW yarışması için sağlanan standart elektrikli Go-Kart şasisi.

## Paket Yapısı ve Kritik Dosyalar

Her araç, Autoware standartlarına uygun olarak `description` (görsel/fiziksel tanım) ve `launch` (başlatma) olmak üzere ikişer paketten oluşur.

### 1. Teknofest Aracı (`my_vehicle`)
*   **Tanım:** `my_vehicle_description/config/vehicle_info.param.yaml`
    *   *Aks Mesafesi (Wheelbase):* ~1.55m
*   **Görsel:** `my_vehicle_description/urdf/vehicle.xacro`
*   **Başlatma:** `my_vehicle_launch/launch/vehicle.launch.xml`

### 2. RDW Yarışma Aracı (`rdw_vehicle`)
*   **Tanım:** `rdw_vehicle_description/config/vehicle_info.param.yaml`
    *   *Aks Mesafesi (Wheelbase):* 1.05m (Daha kısa şasi)
*   **Görsel:** `rdw_vehicle_description/urdf/vehicle.xacro`
*   **Başlatma:** `rdw_vehicle_launch/launch/vehicle.launch.xml`

## Araç Değişirse Ne Yapılmalı?

Yarışma kuralları gereği veya mekanik revizyonlar sonucu araçlardan birinin fiziksel özellikleri değişirse, **ilgili aracın** klasöründe aşağıdaki güncellemeler yapılmalıdır.

### 1. Fiziksel Ölçüleri Güncelleyin
**Dosya:** `<ARAÇ_ISMI>_vehicle_description/config/vehicle_info.param.yaml`

| Parametre | Açıklama ve Kritik Etkisi |
| :--- | :--- |
| **`wheel_base`** | **En Kritik Değer.** Ön ve arka tekerlek merkezleri arası mesafe. Değişirse araç virajları alamaz ve MPC (Path Following) bozulur. |
| **`wheel_tread`** | Sağ ve sol tekerlek arası mesafe. Aracın kapı veya duba aralarından geçip geçemeyeceğini planlayıcı buna göre hesaplar. |
| **`front_overhang`** | Ön tekerlek merkezinden tampon ucuna olan mesafe. Çarpışma güvenliği ve durma noktası için hayati önem taşır. |
| **`wheel_radius`** | Tekerlek yarıçapı. Lastik basıncı değişse bile burayı güncelleyin, aksi takdirde hız verisi (odometry) hatalı olur. |

### 2. Görsel Modeli Güncelleyin (Opsiyonel)
Eğer aracın dış görünüşü değiştiyse (yeni kaporta vb.):
1.  Yeni `.fbx` veya `.dae` dosyasını ilgili aracın `mesh/` klasörüne atın.
2.  `urdf/vehicle.xacro` içindeki dosya yolu referansını güncelleyin.

## Kritik Uyarı: Kontrolcü (Controller) Farklılığı

İki araç birbirinden fiziksel olarak çok farklıdır (`my_vehicle` 1.55m vs `rdw_vehicle` 1.05m wheelbase).

*   **`rdw_vehicle`** daha kıvrak ve kısa olduğu için, **`my_vehicle`** için ayarlanmış PID veya MPC katsayıları bu araçta agresif titremelere (oscillation) sebep olabilir.
*   Simülasyon veya gerçek sürüş testi yaparken doğru araç konfigürasyonunu (`vehicle_id`) seçtiğinizden emin olun.

## Nasıl Derlenir? (Build)

Herhangi bir araçta (`.yaml` veya `.xacro`) değişiklik yaptıktan sonra, **değişiklik yaptığınız paketi** veya hepsini yeniden derlemelisiniz.

```bash
cd ~/autoware

# SEÇENEK A: Sadece RDW Aracını Derle (Hızlı)
colcon build --symlink-install --packages-select rdw_vehicle_description rdw_vehicle_launch

# SEÇENEK B: Sadece Teknofest Aracını Derle
colcon build --symlink-install --packages-select my_vehicle_description my_vehicle_launch

# SEÇENEK C: Tüm Araçları Derle (Temiz Kurulum)
colcon build --symlink-install --packages-select my_vehicle_description my_vehicle_launch rdw_vehicle_description rdw_vehicle_launch --cmake-clean-cache

# Son olarak ortamı güncelle
source install/setup.bash
```