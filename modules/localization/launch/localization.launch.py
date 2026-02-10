#!/usr/bin/env python3
"""
============================================================
LOCALIZATION LAUNCH - YabLoc Tabanlı Lokalizasyon
============================================================

Bu launch dosyası, araç lokalizasyonu için merkezi yapılandırma sağlar.
Şu anda sadece YabLoc kullanılmaktadır.

YabLoc için girdiler:
  - Kamera görüntüsü (camera input)
  - Odometri verisi (wheel odometry)

global_bringup tarafından çağrılır ve Autoware'e pose_source parametresi iletir.

Düzenlenebilir parametreler:
  - localization.yaml: Topic isimleri, frame isimleri
  - yabloc.param.yaml: YabLoc algoritma parametreleri (match threshold, vb.)
============================================================
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _load_yaml(path: str) -> dict:
    """YAML dosyasını yükler. Dosya bulunamazsa boş dict döner."""
    if not os.path.exists(path):
        return {}
    try:
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


def _get_value(cfg: dict, key_path: str, default):
    """İç içe dict'ten değer alır. Örn: "localization.pose_source" """
    keys = key_path.split(".")
    for key in keys:
        if not isinstance(cfg, dict) or key not in cfg:
            return default
        cfg = cfg[key]
    return cfg


def _launch_setup(context, *args, **kwargs):
    """Launch dosyasının çalışma zamanı kurulumu."""
    
    # Paket yollarını al
    share_dir = get_package_share_directory("localization")
    default_config_dir = os.path.join(share_dir, "config")
    
    # Parametreleri oku
    config_dir = LaunchConfiguration("config_dir").perform(context) or default_config_dir
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    
    # localization.yaml dosyasını yükle
    loc_yaml_path = os.path.join(config_dir, "localization.yaml")
    cfg = _load_yaml(loc_yaml_path)
    
    # Konfigürasyondan değerleri al
    pose_source = _get_value(cfg, "localization.pose_source", "yabloc")
    camera_topic = _get_value(cfg, "localization.topics.camera", "/sensing/camera/image")
    odom_topic = _get_value(cfg, "localization.topics.wheel_odom", "/vehicle/odometry")
    
    # Bilgilendirme logları
    return [
        LogInfo(msg=""),
        LogInfo(msg="========================================"),
        LogInfo(msg="  LOCALIZATION MODULE - YabLoc Setup"),
        LogInfo(msg="========================================"),
        LogInfo(msg=f"Config dizini    : {config_dir}"),
        LogInfo(msg=f"Pose kaynağı     : {pose_source}"),
        LogInfo(msg=f"Kamera topic     : {camera_topic}"),
        LogInfo(msg=f"Odometri topic   : {odom_topic}"),
        LogInfo(msg=f"Simülasyon modu  : {use_sim_time}"),
        LogInfo(msg="========================================"),
        LogInfo(msg=""),
        LogInfo(msg="[NOT] Bu modül sadece yapılandırma merkezi olarak çalışır."),
        LogInfo(msg="[NOT] YabLoc, Autoware tarafından global_bringup üzerinden başlatılır."),
        LogInfo(msg="[NOT] Topic ve parametreler localization.yaml'dan okunur."),
        LogInfo(msg=""),
    ]


def generate_launch_description():
    """Launch tanımını oluşturur."""
    
    share_dir = get_package_share_directory("localization")
    default_config_dir = os.path.join(share_dir, "config")
    
    return LaunchDescription([
        # ===== TEMEL PARAMETRELER =====
        DeclareLaunchArgument(
            "config_dir",
            default_value=default_config_dir,
            description="Lokalizasyon config dosyalarının bulunduğu dizin"
        ),
        
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Simülasyon zamanı kullanılsın mı (true/false)"
        ),
        
        # Launch setup fonksiyonunu çağır
        OpaqueFunction(function=_launch_setup),
    ])
