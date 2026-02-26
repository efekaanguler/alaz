#!/usr/bin/env bash

echo "Udev kuralları sisteme kopyalanıyor..."
sudo cp udev_rules/*.rules /etc/udev/rules.d/

echo "Udev kuralları yeniden yükleniyor..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Kurulum tamamlandı! Lütfen sensörlerin USB kablolarını çıkarıp tekrar takın."
