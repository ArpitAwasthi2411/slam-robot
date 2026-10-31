#!/usr/bin/env bash
# Phone detector files (too big for Git): ONNX Runtime Web (npm) + the YOLO11n model exported to ONNX at 320 px.
# Puts them in ros2_ws/src/lidar_robot/lidar_robot/web/app/vendor/ (ignored by Git), where the APK build picks them up.
#
#   bash scripts/fetch_phone_detector.sh            # needs node/npm and python3 + pip (internet)
#   MODEL_FILE=~/yolo11n.onnx bash scripts/...      # use a model you already exported
#
# Used by .github/workflows/android.yml before building the APK.
set -euo pipefail
ORT_VERSION=1.20.1
ULTRALYTICS_VERSION=${ULTRALYTICS_VERSION:-8.4.194}
here="$(cd "$(dirname "$0")/.." && pwd)"
dest="$here/ros2_ws/src/lidar_robot/lidar_robot/web/app/vendor"
mkdir -p "$dest/ort"
tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT

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
    python3 -m pip install -q --index-url https://download.pytorch.org/whl/cpu torch torchvision
    python3 -m pip install -q "ultralytics==$ULTRALYTICS_VERSION" onnx onnxslim
    (cd "$tmp" && python3 - <<'EOF'
from ultralytics import YOLO
YOLO('yolo11n.pt').export(format='onnx', imgsz=320, opset=17, simplify=True, dynamic=False)
EOF
    )
    cp "$tmp/yolo11n.onnx" "$model"
fi

python3 -c "import onnx" 2>/dev/null || python3 -m pip install -q onnx    # the cached-model path skipped the export installs
python3 - "$model" <<'EOF'
import sys, onnx
m = onnx.load(sys.argv[1])
ins = [[d.dim_value for d in i.type.tensor_type.shape.dim] for i in m.graph.input]
outs = [[d.dim_value for d in o.type.tensor_type.shape.dim] for o in m.graph.output]
assert ins == [[1, 3, 320, 320]] and outs == [[1, 84, 2100]], (ins, outs)
print('model check: input', ins[0], 'output', outs[0])
EOF
ls -lh "$dest" "$dest/ort"
