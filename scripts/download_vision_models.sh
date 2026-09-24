#!/usr/bin/env bash
set -euo pipefail

MODEL_DIR="$HOME/Wiz/models/mobilenet_ssd"
mkdir -p "$MODEL_DIR"
cd "$MODEL_DIR"

echo "Downloading MobileNet-SSD (Caffe, VOC0712, MIT licensed - chuanqi305/MobileNet-SSD)..."
curl -sSL -o deploy.prototxt \
  "https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/deploy.prototxt"
curl -sSL -o mobilenet_iter_73000.caffemodel \
  "https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/mobilenet_iter_73000.caffemodel"

echo "Done. Model files in $MODEL_DIR:"
ls -la "$MODEL_DIR"
