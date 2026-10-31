#!/usr/bin/env bash
# Phone detector files (too big for Git): ONNX Runtime Web (npm) + the YOLO11n model exported at 320 px as
#   yolo11n_320.onnx    for the web engine (ONNX Runtime Web, one core in a WebView)
#   yolo11n_320.tflite  for the app's native engine (TensorFlow Lite on the GPU / all CPU cores), float16 weights,
#                       converted from that ONNX file with onnx2tf (same boxes; checked against ONNX Runtime)
# Puts them in ros2_ws/src/lidar_robot/lidar_robot/web/app/vendor/ (ignored by Git), where the APK build picks them up.
#
#   bash scripts/fetch_phone_detector.sh            # needs node/npm and python3 3.9-3.12 + pip (internet)
#   MODEL_FILE=~/yolo11n.onnx TFLITE_FILE=~/yolo11n_float16.tflite bash scripts/...   # use models you already exported
#
# Used by .github/workflows/android.yml before building the APK.
set -euo pipefail
ORT_VERSION=1.20.1
ULTRALYTICS_VERSION=${ULTRALYTICS_VERSION:-8.4.194}
here="$(cd "$(dirname "$0")/.." && pwd)"
dest="$here/ros2_ws/src/lidar_robot/lidar_robot/web/app/vendor"
mkdir -p "$dest/ort"
tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT

torch_cpu() { python3 -c "import torch" 2>/dev/null || python3 -m pip install -q --index-url https://download.pytorch.org/whl/cpu torch torchvision; }

echo "== ONNX Runtime Web $ORT_VERSION"
(cd "$tmp" && npm pack --silent "onnxruntime-web@$ORT_VERSION" >/dev/null && tar -xzf onnxruntime-web-*.tgz)
for f in ort.wasm.min.js ort-wasm-simd-threaded.mjs ort-wasm-simd-threaded.wasm; do
    cp "$tmp/package/dist/$f" "$dest/ort/$f"
done

model="$dest/yolo11n_320.onnx"
if [ -n "${MODEL_FILE:-}" ]; then
    echo "== model from $MODEL_FILE"
    cp "$MODEL_FILE" "$model"
elif [ -s "$model" ] && [ -z "${FORCE_EXPORT:-}" ]; then
    echo "== model already there ($model)"
else
    echo "== exporting YOLO11n to ONNX (ultralytics $ULTRALYTICS_VERSION, CPU-only PyTorch)"
    torch_cpu
    python3 -m pip install -q "ultralytics==$ULTRALYTICS_VERSION" onnx onnxslim
    (cd "$tmp" && python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt').export(format='onnx', imgsz=320, opset=17, simplify=True, dynamic=False)")
    cp "$tmp/yolo11n.onnx" "$model"
fi

python3 -c "import onnx" 2>/dev/null || python3 -m pip install -q onnx    # the cached-model path skipped the export installs
python3 "$here/scripts/check_phone_model.py" onnx "$model"

tfl="$dest/yolo11n_320.tflite"
if [ -n "${TFLITE_FILE:-}" ]; then
    echo "== TFLite model from $TFLITE_FILE"
    cp "$TFLITE_FILE" "$tfl"
elif [ -s "$tfl" ] && [ -z "${FORCE_EXPORT:-}" ]; then
    echo "== TFLite model already there ($tfl)"
else
    echo "== converting the ONNX model to TFLite with onnx2tf (float16 weights; TensorFlow needs Python 3.9-3.12)"
    python3 -m pip install -q "tensorflow-cpu==2.19.*" onnx onnxslim "onnx2tf>=1.26" sng4onnx onnx_graphsurgeon \
        tf_keras ai-edge-litert psutil
    # onnx2tf downloads 20 sample pictures for its self-check; give it a local file instead
    (cd "$tmp" && python3 -c "import numpy as n; n.save('calibration_image_sample_data_20x128x128x3_float32.npy', n.random.rand(20,128,128,3).astype('float32'))" \
        && TF_CPP_MIN_LOG_LEVEL=3 onnx2tf -i "$model" -o tfl -nuo -osd >onnx2tf.log 2>&1 || { tail -30 onnx2tf.log; exit 1; })
    cp "$tmp/tfl/yolo11n_320_float16.tflite" "$tfl"
fi
python3 -c "import ai_edge_litert" 2>/dev/null || python3 -m pip install -q ai-edge-litert
python3 "$here/scripts/check_phone_model.py" tflite "$tfl"

ls -lh "$dest" "$dest/ort"
