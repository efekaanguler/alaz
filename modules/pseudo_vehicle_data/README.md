# pseudo_vehicle_data

ROS 2 tabanlı bu paket, araç geri bildirimi için pseudo veri üretmek ve basit bir araç dinamiği simülasyonu yapmak amacıyla hazırlanmıştır.

## İçerik

Paket içinde iki node bulunur:

1. `auto_pseudo_node`
	- Komut almadan otomatik olarak gerçekçi hız ve direksiyon verisi üretir.
	- 20 Hz'de yayın yapar.
	- Hız ve direksiyon değerleri sinüs tabanlı yumuşak değişimler ile oluşturulur.

2. `vehicle_simulator_node`
	- Gaz, fren ve direksiyon komutlarını alır.
	- Basit ivmelenme/yavaşlama/sürtünme modeli ile hız hesaplar.
	- Direksiyon açısını hedef değere kademeli yaklaştırır.
	- 20 Hz'de geri bildirim yayınlar.

## Topic Arabirimleri

### Yayınlanan Topic'ler

- `/vehicle_speed` (`std_msgs/Float32`): Araç hızı (m/s)
- `/steering_angle` (`std_msgs/Float32`): Direksiyon açısı (normalize değer)

### Abone Olunan Topic'ler (`vehicle_simulator_node` için)

- `/throttle_cmd` (`std_msgs/Int32`): Gaz komutu (`0..100`)
- `/brake_cmd` (`std_msgs/Int32`): Fren komutu (`0..100`)
- `/steering_cmd` (`std_msgs/Float32`): Direksiyon komutu (`-1.0..1.0`)

## Bağımlılıklar

- `rclpy`
- `std_msgs`

## Derleme

Çalışma alanı kök dizininde:

```bash
colcon build --packages-select pseudo_vehicle_data
source install/setup.bash
```

## Çalıştırma

### 1) Otomatik pseudo veri üretimi

```bash
ros2 run pseudo_vehicle_data auto_pseudo
```

### 2) Araç simülasyonu node'u

Bu node kodda mevcut olsa da `setup.py` içinde console script olarak tanımlı değildir. Aşağıdaki iki yöntemden biri kullanılabilir:

- `setup.py` içine entry point ekleyip `ros2 run` ile çalıştırmak
- Python modülü olarak doğrudan çalıştırmak

Örnek doğrudan çalıştırma:

```bash
python3 -m pseudo_vehicle_data.vehicle_simulator_node
```

## Örnek Komut Yayını (`vehicle_simulator_node` çalışırken)

```bash
ros2 topic pub /throttle_cmd std_msgs/msg/Int32 "{data: 40}" -r 10
ros2 topic pub /brake_cmd std_msgs/msg/Int32 "{data: 0}" -r 10
ros2 topic pub /steering_cmd std_msgs/msg/Float32 "{data: 0.3}" -r 10
```

## İzleme

```bash
ros2 topic echo /vehicle_speed
ros2 topic echo /steering_angle
```

## Notlar

- Simülasyon modeli basit tutulmuştur; fiziksel doğruluk yerine test ve entegrasyon kolaylığı hedeflenmiştir.
- `auto_pseudo_node`, kontrol komutlarından bağımsız sentetik geri bildirim üretir.
