#!/bin/bash

echo "Build, install, log ve önbellek (cache) dizinleri temizleniyor..."

rm -rf ./build ./install ./log
rm -rf ./modules/build ./modules/install ./modules/log
find . \( -name "__pycache__" -o -name "*.pyc" -o -name ".DS_Store" \) -exec rm -rf {} + 2>/dev/null || true

echo "Temizleme tamamlandı!"