# How to develop in "/modules"

## Dizin Yapısı

```
modules/
├── build.sh
├── README.md
├── my_pkg1/
├── my_pkg2/
└── my_pkg3/
```

## Paket Organizasyonu

### Standart ROS2 Paket Yapısı

#### C++ Paketi:
```
my_package_cpp/
├── package.xml             # Paket tanımı ve bağımlılıklar
├── CMakeLists.txt          # CMake build konfigürasyonu
├── include/
│   └── my_package_cpp/
│       └── my_node.hpp     # Header dosyaları
├── src/
│   ├── my_node.cpp         # Kaynak dosyalar
│   └── main.cpp
├── launch/
│   └── my_launch.py        # Launch dosyaları
├── config/
│   └── params.yaml         # Parametre dosyaları
└── test/
    └── test_my_node.cpp    # Unit testler
```

#### Python Paketi:
```
my_package_py/
├── package.xml             # Paket tanımı
├── setup.py                # Python kurulum scripti
├── setup.cfg               # Setup konfigürasyonu
├── my_package_py/
│   ├── __init__.py
│   ├── my_node.py          # Python node'ları
│   └── utils.py
├── launch/
│   └── my_launch.py
├── config/
│   └── params.yaml
└── test/
    └── test_my_node.py
```