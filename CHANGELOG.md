# Changelog

## [0.1.0] - 2026-07-11

### Fixed

- Autoware engage mesaj tipi, operation-mode servisleri ve autonomous control
  gecisindeki deadlock giderildi.
- Planning trajectory follower, shift decider, vehicle command gate, operation
  mode manager ve vehicle interface resmi Autoware topic zincirinde hizalandi.
- Global launch argumanlari scope edilerek `config_file` ve arac parametrelerinin
  moduller arasinda sizmasi engellendi.
- Kamera ve LiDAR abonelikleri sensor-data QoS ile; odometry, localization ve
  control topic adlari Autoware arayuzleriyle hizalandi.
- Dinamik TF sahipligi localization'a verildi; odometry TF yayini varsayilan
  olarak kapatildi ve sensor frame zinciri tamamlandi.
- CAN bridge yalniz `/to_can_bus` ve `/from_can_bus` transportu yapacak sekilde
  ayrildi; virtual test transportu ve temiz kapanis eklendi.
- Vehicle interface MANUAL/NEUTRAL fail-safe, command timeout ve hardware mode
  feedback korumalariyla guvenli hale getirildi.
- Perception planner nesneleri metric LiDAR tracking/prediction hattina tasindi;
  piksel koordinatlarini metre kabul eden 2D bridge varsayilan olarak kapatildi.

### Removed

- Onceki araca ait CAN wiki testleri, gecersiz topic'leri kullanan perception
  testleri, yinelenen LiDAR node'lari ve eski baslatma scriptleri kaldirildi.
- Gercek LiDAR yokken otomatik dummy veri ureten guvensiz fallback kaldirildi.

### Verified

- Temiz build: 17/17 paket.
- Ana workspace: 92 test, 0 error, 0 failure.
- Perception child workspace: 5/5 paket build.
- Control E2E ve Docker perception smoke testleri PASS.

### Engage

```bash
ros2 service call /system/operation_mode/change_operation_mode autoware_system_msgs/srv/ChangeOperationMode "{mode: 2}" && ros2 service call /system/operation_mode/change_autoware_control autoware_system_msgs/srv/ChangeAutowareControl "{autoware_control: true}"
```
