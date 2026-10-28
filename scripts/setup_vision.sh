#!/usr/bin/env bash
# People detection for the robot's camera: NCNN runtime (light, no PyTorch while driving) + YOLO11n.
#   bash ~/slam-robot/scripts/setup_vision.sh        (as arpit, NOT sudo; 15-30 min the first time)
# The model is exported once in a throwaway Python venv (that part downloads PyTorch, ~1 GB, then deletes it).
# Faster: export on a laptop that has ultralytics:  yolo export model=yolo11n.pt format=ncnn imgsz=320
#         then copy the folder:  scp -r yolo11n_ncnn_model arpit@<pi>:~/models/
set -euo pipefail
[ "$(id -u)" -ne 0 ] || { echo "run WITHOUT sudo (as your normal user)"; exit 1; }
MODELS="$HOME/models"
OUT="$MODELS/yolo11n_ncnn_model"

echo "== 1/3 runtime: OpenCV, numpy (from Ubuntu, so ROS keeps its numpy) + ncnn"
sudo apt-get install -y python3-opencv python3-numpy python3-venv python3-pip
# --no-deps: ncnn's wheel would pull opencv-python + a new numpy over Ubuntu's (ROS uses those)
pip3 install --user --no-deps 'ncnn==1.0.20250503'
python3 -c "import ncnn, cv2, numpy; print('  ncnn', ncnn.__version__, '| opencv', cv2.__version__, '| numpy', numpy.__version__)"

echo "== 2/3 YOLO11n model for NCNN (320 x 320)"
if [ -f "$OUT/model.ncnn.param" ] && [ -f "$OUT/model.ncnn.bin" ]; then
  echo "  already there: $OUT"
else
  VENV="$HOME/.cache/yolo-export"
  python3 -m venv "$VENV"
  "$VENV/bin/pip" install --upgrade pip
  "$VENV/bin/pip" install ultralytics ncnn
  mkdir -p "$MODELS" && cd "$MODELS"
  "$VENV/bin/yolo" export model=yolo11n.pt format=ncnn imgsz=320
  rm -rf "$VENV" "$MODELS/yolo11n.pt"
  [ -f "$OUT/model.ncnn.param" ] || { echo "  !! export failed: send the output above to Claude"; exit 1; }
fi

echo "== 3/3 quick test on a blank picture"
python3 - "$OUT" <<'PY'
import sys, time, numpy as np, ncnn
d = sys.argv[1]
net = ncnn.Net(); net.opt.num_threads = 2
net.load_param(d + '/model.ncnn.param'); net.load_model(d + '/model.ncnn.bin')
x = np.zeros((3, 320, 320), np.float32)
t = time.time()
for _ in range(5):
    ex = net.create_extractor(); ex.input('in0', ncnn.Mat(x).clone()); _, out = ex.extract('out0')
print(f'  output {np.array(out).shape}, {(time.time() - t) / 5 * 1000:.0f} ms per picture')
PY
echo "Done. Restart the robot:  sudo systemctl restart robot"
echo "Check:  source ~/ros2_ws/install/setup.bash && ros2 topic echo /vision/decision --once"
