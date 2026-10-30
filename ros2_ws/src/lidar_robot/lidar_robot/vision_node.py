#!/usr/bin/env python3
"""Camera node: people detection -> STOP / SLOW_DOWN / CONTINUE for the navigator, live picture for the app.

  camera (Camera Module 3 Wide via libcamera/GStreamer, or a USB webcam): sensor at fps, only out_fps
  frames/s reach Python (GStreamer drops the rest; the Pi's ISP makes BGR when it can)
    -> YOLO11n person detection (NCNN, no PyTorch at run time; det_rate per second, 1 thread)
    -> PersonTracker (ids, approaching) -> distance from the LiDAR scan at each person's bearing
    -> vision_rules.decide()

Publishes
  /vision/decision          std_msgs/String  JSON {action, speed_factor, reason, steer, people[], fps, ...}
  /vision/image/compressed  sensor_msgs/CompressedImage  JPEG with boxes + decision (for the app), only
                            while the app's camera view is open (/vision/stream_request)
  /vision/dock_marker       std_msgs/String  dock marker pose, only while docking (/vision/marker_request)
Subscribes
  /scan                     distances for the people the camera sees
  /vision/stream_request, /vision/marker_request   std_msgs/Empty  "keep doing it for a few more seconds"

Setup on the Pi (once): scripts/setup_pi_camera.sh (libcamera for the Camera Module 3) and
scripts/setup_vision.sh (NCNN + the YOLO model). Without them the node keeps running and reports
what is missing; the navigator then simply ignores the camera.
"""
import json
import os
import shutil
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String, Empty

from lidar_robot.vision_rules import (PersonTracker, bearing_deg, decide, decode_yolo,
                                      letterbox_params, lidar_distance)

LIBCAMERA_PATHS = {   # where scripts/setup_pi_camera.sh installs libcamera (meson default prefix)
    'GST_PLUGIN_PATH': '/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0',
    'LD_LIBRARY_PATH': '/usr/local/lib/aarch64-linux-gnu',
}
COLOURS = {'STOP': (40, 40, 230), 'SLOW_DOWN': (0, 170, 255), 'CONTINUE': (80, 200, 60), 'UNKNOWN': (160, 160, 160)}


def _env_with_libcamera():
    env = dict(os.environ)
    for k, p in LIBCAMERA_PATHS.items():
        if os.path.isdir(p) and p not in env.get(k, ''):
            env[k] = p + (':' + env[k] if env.get(k) else '')
    return env


class Camera:
    """Latest frame from libcamera (gst-launch subprocess -> raw BGR on stdout) or OpenCV."""

    def __init__(self, source, width, height, fps, log, out_fps=5):
        self.source, self.w, self.h, self.fps, self.log = source, width, height, fps, log
        # the sensor runs at fps (short exposures, no motion blur) but only out_fps frames a second reach
        # Python: videorate drops the rest inside GStreamer, before any colour conversion or pipe copy
        self.out_fps = max(1, min(int(fps), int(out_fps)))
        self.frame, self.stamp, self.error, self.kind, self.stderr_tail = None, 0.0, '', '', ''
        self.mode = ''                 # 'isp' (the Pi's ISP makes BGR, no CPU conversion) | 'convert' | 'opencv'
        # ISP output is tried first; rows must need no padding (640 x 3 bytes is a multiple of 64)
        self.isp_ok = None if (width * 3) % 64 == 0 else False
        self.proc, self.cap = None, None
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def pipeline(self, isp):
        """gst-launch-1.0 arguments: libcamera -> drop to out_fps -> BGR -> stdout."""
        caps = f'video/x-raw,width={self.w},height={self.h},framerate={int(self.fps)}/1'
        rate = ['videorate', 'drop-only=true', '!', f'video/x-raw,framerate={self.out_fps}/1', '!']
        if isp:
            return ['gst-launch-1.0', '-q', 'libcamerasrc', '!', caps + ',format=BGR', '!'] + rate + \
                   ['fdsink', 'fd=1', 'sync=false']
        return ['gst-launch-1.0', '-q', 'libcamerasrc', '!', caps, '!'] + rate + \
               ['videoconvert', '!', 'video/x-raw,format=BGR', '!', 'fdsink', 'fd=1', 'sync=false']

    def _libcamera_ok(self, env):
        if not shutil.which('gst-launch-1.0') or not shutil.which('gst-inspect-1.0'):
            return False
        r = subprocess.run(['gst-inspect-1.0', 'libcamerasrc'], env=env, capture_output=True, timeout=20)
        return r.returncode == 0

    def _open(self):
        env = _env_with_libcamera()
        src = str(self.source)
        if src in ('auto', 'libcamera') and self._libcamera_ok(env):
            isp = self.isp_ok is not False
            cmd = self.pipeline(isp)
            self.mode = 'isp' if isp else 'convert'
            self.proc = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                         bufsize=0)
            self.stderr_tail = ''
            threading.Thread(target=self._drain_stderr, args=(self.proc,), daemon=True).start()
            self.kind = 'libcamera'
            return True
        if src == 'libcamera':
            self.error = ('libcamera not found (gst-inspect-1.0 libcamerasrc failed): '
                          'run scripts/setup_pi_camera.sh')
            return False
        import cv2
        dev = int(src) if src.isdigit() else ('/dev/video0' if src == 'auto' else src)
        self.cap = cv2.VideoCapture(dev)
        if not self.cap.isOpened():
            self.error = f'no camera: libcamera not installed and {dev} did not open'
            self.cap = None
            return False
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        self.cap.set(cv2.CAP_PROP_FPS, self.out_fps)
        self.kind, self.mode = f'opencv {dev}', 'opencv'
        return True

    def _drain_stderr(self, proc):
        # a full stderr pipe would block GStreamer: keep reading, remember the end for error messages
        for line in iter(proc.stderr.readline, b''):
            self.stderr_tail = (self.stderr_tail + line.decode(errors='replace'))[-400:]

    def _read_exact(self, n):
        buf = bytearray(n)
        view, got = memoryview(buf), 0
        while got < n:
            k = self.proc.stdout.readinto(view[got:])
            if not k:
                return None
            got += k
        return buf

    def _run(self):
        import numpy as np
        while self.running:
            try:
                if not self._open():
                    self.log(self.error)
                    time.sleep(10.0)
                    continue
                self.error = ''
                self.log(f'camera: {self.kind} ({self.mode}) {self.w}x{self.h}, {self.out_fps} frames/s used')
                n = self.w * self.h * 3
                got_one = False
                while self.running:
                    if self.proc:
                        buf = self._read_exact(n)
                        if buf is None:
                            time.sleep(0.2)
                            if self.mode == 'isp' and not got_one:
                                # this libcamera can't hand out BGR itself: convert on the CPU from now on
                                self.isp_ok = False
                                self.log('camera: ISP BGR output refused, using videoconvert')
                                self.close()
                                break
                            raise RuntimeError('camera stream ended: ' + self.stderr_tail.strip()[-300:])
                        if self.mode == 'isp':
                            self.isp_ok = True
                        got_one = True
                        frame = np.frombuffer(buf, dtype=np.uint8).reshape(self.h, self.w, 3)
                    else:
                        ok, frame = self.cap.read()
                        if not ok:
                            raise RuntimeError('camera read failed')
                    self.frame, self.stamp = frame, time.monotonic()
            except Exception as e:                   # unplugged, pipeline died...: retry
                self.error = str(e)
                self.log(f'camera error: {e}; retrying in 5 s')
                self.close()
                time.sleep(5.0)

    def close(self):
        if self.proc:
            try:
                self.proc.kill()
            except Exception:
                pass
            self.proc = None
        if self.cap:
            self.cap.release()
            self.cap = None


class Detector:
    """YOLO11n exported to NCNN (model.ncnn.param / model.ncnn.bin), person class only."""

    def __init__(self, model_dir, size, threads):
        import ncnn
        self.size = size
        self.net = ncnn.Net()
        self.net.opt.num_threads = threads
        self.net.opt.use_vulkan_compute = False
        p, b = os.path.join(model_dir, 'model.ncnn.param'), os.path.join(model_dir, 'model.ncnn.bin')
        if not (os.path.isfile(p) and os.path.isfile(b)):
            raise FileNotFoundError(f'{model_dir} has no model.ncnn.param/.bin: run scripts/setup_vision.sh')
        self.net.load_param(p)
        self.net.load_model(b)
        self.ncnn = ncnn

    def detect(self, frame, conf):
        import cv2
        import numpy as np
        h, w = frame.shape[:2]
        s, px, py, nw, nh = letterbox_params(w, h, self.size)
        img = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
        img = cv2.copyMakeBorder(img, py, self.size - nh - py, px, self.size - nw - px,
                                 cv2.BORDER_CONSTANT, value=(114, 114, 114))
        chw = np.ascontiguousarray(img[:, :, ::-1].transpose(2, 0, 1), dtype=np.float32) / 255.0
        ex = self.net.create_extractor()
        ex.input('in0', self.ncnn.Mat(chw).clone())
        _, out = ex.extract('out0')
        return decode_yolo(np.array(out), w, h, self.size, conf=conf)


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision')
        d = self.declare_parameter
        d('camera', 'auto')             # auto | libcamera | /dev/videoN | N
        d('width', 640)
        d('height', 360)                # 16:9 keeps the wide lens' full view
        d('fps', 15)                    # sensor frame rate (short exposures = sharp people)
        d('out_fps', 5)                 # frames per second handed to Python (detection + app picture)
        d('hfov_deg', 102.0)            # Camera Module 3 Wide: 102 deg horizontal (120 diagonal)
        d('flip', False)                # camera mounted upside down
        d('model_dir', os.path.expanduser('~/models/yolo11n_ncnn_model'))
        d('imgsz', 320)
        d('conf', 0.45)
        d('threads', 1)                 # 1 core: Cartographer + bridge + navigator need the rest of a Pi 4
        d('det_rate', 2.0)
        d('stream_rate', 5.0)
        d('jpeg_quality', 60)
        d('laser_yaw', 0.0)             # same as the launch's laser_yaw (base_link -> laser)
        d('path_half_angle_deg', 15.0)
        d('stop_m', 0.8)
        d('slow_m', 1.8)
        # dock marker for front docking (marker_pose.py): ArUco DICT_4X4_50, id 0, 8 cm, ~10 Hz
        d('dock_marker', True)
        d('marker_dict', 'DICT_4X4_50')
        d('marker_id', 0)
        d('marker_size_m', 0.08)
        d('marker_rate', 10.0)
        d('marker_always', False)       # True = look for the marker all the time (bench tests); normally
                                        # only while the navigator is docking (vision/marker_request)
        d('camera_calib', '')           # YAML (config/camera_calib.yaml); empty = built-in defaults
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.p = {n: g(n) for n in ('hfov_deg', 'flip', 'conf', 'det_rate', 'stream_rate', 'jpeg_quality',
                                     'laser_yaw')}
        self.cfg = {'path_half_angle_deg': g('path_half_angle_deg'), 'stop_m': g('stop_m'), 'slow_m': g('slow_m')}
        try:
            os.nice(5)                  # the robot's motion nodes come first
        except OSError:
            pass

        self.dec_pub = self.create_publisher(String, 'vision/decision', 10)
        self.marker_pub = self.create_publisher(String, 'vision/dock_marker', 10)
        self.img_pub = self.create_publisher(CompressedImage, 'vision/image/compressed', 2)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.scan, self.scan_t = None, 0.0
        self.lock = threading.Lock()
        self.people, self.decision, self.fps = [], None, 0.0

        self.camera = Camera(g('camera'), int(g('width')), int(g('height')), int(g('fps')),
                             lambda m: self.get_logger().warn(m, throttle_duration_sec=30.0),
                             out_fps=int(g('out_fps')))
        self.detector, self.det_error = None, ''
        try:
            self.detector = Detector(os.path.expanduser(g('model_dir')), int(g('imgsz')), int(g('threads')))
        except Exception as e:          # ncnn not installed / model missing
            self.det_error = f'people detection off: {e}'
            self.get_logger().warn(self.det_error)
        self.tracker = PersonTracker()
        self.running = True
        threading.Thread(target=self._detect_loop, daemon=True).start()
        self.marker = None
        if g('dock_marker'):
            from lidar_robot.marker_pose import MarkerDetector, load_calibration
            try:
                calib = load_calibration(str(g('camera_calib')) or None)
            except Exception as e:      # bad YAML: say so, use the defaults
                self.get_logger().warn(f'camera_calib: {e}; using built-in defaults')
                calib = None
            self.marker = MarkerDetector(calib, g('marker_dict'), int(g('marker_id')), float(g('marker_size_m')))
            if self.marker.error:
                self.get_logger().warn(f'dock marker detection off: {self.marker.error}')
            else:
                self.marker_period = 1.0 / max(1.0, min(float(g('marker_rate')), float(self.camera.out_fps)))
                self.marker_always = bool(g('marker_always'))
                self.marker_until = 0.0
                self.create_subscription(Empty, 'vision/marker_request', self._on_marker_request, 5)
                threading.Thread(target=self._marker_loop, daemon=True).start()
        # the live picture costs CPU (draw + JPEG + ROS transfer): only while someone has the app's camera view open
        self.stream_until = 0.0
        self.create_subscription(Empty, 'vision/stream_request', self._on_stream_request, 5)
        self.create_timer(1.0 / max(0.5, float(self.p['stream_rate'])), self._publish_image)
        self.get_logger().info('vision node up')

    def _on_scan(self, msg):
        # keep the message's own array (no per-scan list copy): it is only read at detection time
        with self.lock:
            self.scan = (msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
            self.scan_t = time.monotonic()

    def _on_marker_request(self, _msg):
        self.marker_until = time.monotonic() + 3.0

    def _frame(self):
        f, t = self.camera.frame, self.camera.stamp
        if f is None or time.monotonic() - t > 1.0:
            return None
        if self.p['flip']:
            import cv2
            f = cv2.rotate(f, cv2.ROTATE_180)
        return f

    def marker_wanted(self, now=None):
        """ArUco search costs a core-share at 10 Hz: only while docking (or marker_always)."""
        return self.marker_always or (now or time.monotonic()) < self.marker_until

    def _marker_loop(self):
        last_stamp = None
        while self.running:
            t0 = time.monotonic()
            if not self.marker_wanted(t0):
                time.sleep(0.2)
                continue
            if self.camera.stamp == last_stamp and self.camera.frame is not None:
                time.sleep(0.02)            # same picture as last time: nothing new to look at
                continue
            last_stamp = self.camera.stamp
            frame = self._frame()
            r = self.marker.detect(frame) if frame is not None else {'found': False, 'error': 'no camera picture'}
            r['t'] = round(time.time(), 2)
            if self.marker.calib_note:
                r['note'] = self.marker.calib_note
            self.marker_pub.publish(String(data=json.dumps(r)))
            time.sleep(max(0.0, self.marker_period - (time.monotonic() - t0)))

    def _detect_loop(self):
        period = 1.0 / max(0.5, float(self.p['det_rate']))
        last = time.monotonic()
        last_stamp = None
        while self.running:
            t0 = time.monotonic()
            if self.camera.stamp == last_stamp and self.camera.frame is not None \
                    and t0 - self.camera.stamp < 1.0:
                time.sleep(0.02)            # no new picture yet: don't run YOLO on the same one twice
                continue
            last_stamp = self.camera.stamp
            frame = self._frame()
            msg = {'camera_ok': frame is not None, 'camera': self.camera.kind,
                   'camera_error': self.camera.error, 'detector': self.detector is not None,
                   'detector_error': self.det_error}
            if frame is None:
                msg.update(action='UNKNOWN', speed_factor=1.0, steer='none', people=[], people_in_path=0,
                           reason='no camera picture' + (f': {self.camera.error}' if self.camera.error else ''))
            elif self.detector is None:
                msg.update(action='UNKNOWN', speed_factor=1.0, steer='none', people=[], people_in_path=0,
                           reason=self.det_error)
            else:
                try:
                    boxes = self.detector.detect(frame, float(self.p['conf']))
                except Exception as e:
                    boxes = []
                    self.get_logger().error(f'detection failed: {e}', throttle_duration_sec=10.0)
                h, w = frame.shape[:2]
                with self.lock:
                    scan = self.scan if time.monotonic() - self.scan_t < 0.6 else None
                people = self.tracker.update(boxes)
                for p in people:
                    p['bearing_deg'] = round(bearing_deg(p['box'], w, self.p['hfov_deg']), 1)
                    dist = None
                    if scan:
                        dist = lidar_distance(p['bearing_deg'], scan[0], scan[1], scan[2],
                                              float(self.p['laser_yaw']), range_min=max(0.12, scan[3]),
                                              range_max=scan[4])
                    p['distance_m'] = None if dist is None else round(dist, 2)
                dec = decide(people, h, self.cfg)
                now = time.monotonic()
                self.fps = 0.8 * self.fps + 0.2 / max(1e-3, now - last) if self.fps else 1.0 / max(1e-3, now - last)
                last = now
                msg.update(dec, people=people, frame_w=w, frame_h=h, fps=round(self.fps, 1),
                           lidar=scan is not None)
                with self.lock:
                    self.people = people
            msg['time'] = time.time()
            with self.lock:
                self.decision = msg
            self.dec_pub.publish(String(data=json.dumps(msg)))
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def _on_stream_request(self, _msg):
        self.stream_until = time.monotonic() + 5.0

    def _publish_image(self):
        if time.monotonic() > self.stream_until:
            return
        frame = self._frame()
        if frame is None:
            return
        import cv2
        img = frame.copy()
        h, w = img.shape[:2]
        with self.lock:
            people, dec = list(self.people), dict(self.decision or {})
        # the robot's path: +-path_half_angle around the centre
        half = self.cfg['path_half_angle_deg'] / self.p['hfov_deg'] * w
        for x in (int(w / 2 - half), int(w / 2 + half)):
            cv2.line(img, (x, h), (x, int(h * 0.45)), (230, 230, 230), 1, cv2.LINE_AA)
        for p in people:
            x1, y1, x2, y2 = p['box']
            in_path = abs(p['bearing_deg']) <= self.cfg['path_half_angle_deg']
            c = COLOURS['STOP'] if in_path and dec.get('action') == 'STOP' else \
                COLOURS['SLOW_DOWN'] if in_path else (255, 200, 0)
            cv2.rectangle(img, (x1, y1), (x2, y2), c, 2)
            label = f"{p['bearing_deg']:+.0f}deg" + (f" {p['distance_m']:.1f}m" if p.get('distance_m') else '') + \
                    (' >>' if p.get('approaching') else '')
            cv2.putText(img, label, (x1 + 2, max(14, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2, cv2.LINE_AA)
        a = dec.get('action', 'UNKNOWN')
        cv2.rectangle(img, (0, 0), (w, 26), (0, 0, 0), -1)
        cv2.putText(img, f"{a}  x{dec.get('speed_factor', 1.0):.1f}  {dec.get('reason', '')}"[:70], (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOURS.get(a, COLOURS['UNKNOWN']), 1, cv2.LINE_AA)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, int(self.p['jpeg_quality'])])
        if ok:
            m = CompressedImage()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'camera'
            m.format = 'jpeg'
            m.data = jpg.tobytes()
            self.img_pub.publish(m)

    def destroy_node(self):
        self.running = False
        self.camera.running = False
        self.camera.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
