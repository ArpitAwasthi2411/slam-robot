"""Dock marker detection: ArUco (OpenCV) -> where the marker is relative to the camera.

    from lidar_robot.marker_pose import detect
    detect(frame) -> {'found': True, 'x_m': 0.03, 'y_m': -0.01, 'z_m': 0.62, 'yaw_deg': -4.0, 'id': 0}

This is the interface Neelaksh's detector plugs into (same keys, same frame), so the navigator does not care
which detector runs:
  x_m      marker centre to the RIGHT of the camera axis (m)
  y_m      marker centre BELOW the camera axis (m)
  z_m      marker centre straight AHEAD of the camera (m), i.e. distance along the optical axis
  yaw_deg  how much the marker is turned about the vertical axis: 0 = it faces the camera squarely;
           > 0 = the marker's face points towards the camera's right side (the dock line runs off to the right)
  (OpenCV camera frame: x right, y down, z forward.)

Defaults (config/robot_params.yaml, vision node): DICT_4X4_50, marker id 0, 8 cm black square.
Calibration: config/camera_calib.yaml (Camera Module 3 Wide at 640x480 approximations until measured);
the camera matrix is scaled when the frame size differs from the calibration size.
OpenCV is optional for the rest of the package: importing this module never fails without it.
"""
import math
import os

try:
    import cv2
    import numpy as np
except ImportError:                       # laptop/CI without OpenCV: detect() reports why
    cv2 = None
    np = None

# Camera Module 3 Wide (IMX708, 16:9 sensor, ~102 deg horizontal over the full width). A 4:3 mode (640x480)
# crops the sides (~86 deg horizontal); the 16:9 mode the vision node uses (640x360) keeps all of it.
# Approximate pinhole values + mild barrel distortion: measure the real ones (docs/docking.md) and put
# them in config/camera_calib.yaml.
DEFAULT_CALIB = {
    'width': 640, 'height': 480,
    'camera_matrix': [[346.0, 0.0, 320.0], [0.0, 346.0, 240.0], [0.0, 0.0, 1.0]],
    'dist_coeffs': [-0.05, 0.01, 0.0, 0.0, 0.0],
    'source': 'built-in defaults (Camera Module 3 Wide, 640x480, not measured)',
}
DEFAULT_CALIB_16_9 = {
    'width': 640, 'height': 360,
    'camera_matrix': [[259.0, 0.0, 320.0], [0.0, 259.0, 180.0], [0.0, 0.0, 1.0]],
    'dist_coeffs': [-0.05, 0.01, 0.0, 0.0, 0.0],
    'source': 'built-in defaults (Camera Module 3 Wide, 640x360, not measured)',
}


def load_calibration(path=None):
    """YAML with width, height, camera_matrix (3x3), dist_coeffs (list). Missing file -> defaults."""
    if not path or not os.path.isfile(os.path.expanduser(path)):
        return dict(DEFAULT_CALIB)
    import yaml
    with open(os.path.expanduser(path)) as f:
        d = yaml.safe_load(f) or {}
    out = dict(DEFAULT_CALIB)
    for k in ('width', 'height', 'camera_matrix', 'dist_coeffs'):
        if d.get(k) is not None:
            out[k] = d[k]
    cm = out['camera_matrix']
    if len(cm) != 3 or any(len(r) != 3 for r in cm):
        raise ValueError(f'{path}: camera_matrix must be 3x3')
    out['source'] = path
    return out


def yaw_from_normal(nx, nz):
    """Marker normal (pointing out of the marker, towards the camera) in the camera frame -> yaw_deg."""
    return math.degrees(math.atan2(nx, -nz))


class MarkerDetector:
    def __init__(self, calib=None, dictionary='DICT_4X4_50', marker_id=0, marker_size_m=0.08):
        self.calib = calib or dict(DEFAULT_CALIB)
        self.dictionary_name = dictionary
        self.marker_id = int(marker_id)
        self.size = float(marker_size_m)
        self.error = None
        self.calib_note = ''
        self._detector = None
        if cv2 is None:
            self.error = 'OpenCV (python3-opencv) is not installed'
            return
        if not hasattr(cv2, 'aruco'):
            self.error = 'this OpenCV has no aruco module (install python3-opencv or opencv-contrib-python)'
            return
        d = getattr(cv2.aruco, dictionary, None)
        if d is None:
            self.error = f'unknown ArUco dictionary {dictionary}'
            return
        self._dict = cv2.aruco.getPredefinedDictionary(d)
        if hasattr(cv2.aruco, 'ArucoDetector'):                     # OpenCV >= 4.7
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self._detector = cv2.aruco.ArucoDetector(self._dict, params)
        else:                                                       # Ubuntu 22.04: OpenCV 4.5
            self._params = cv2.aruco.DetectorParameters_create()
            self._params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        h = self.size / 2.0                 # marker corners: top-left, top-right, bottom-right, bottom-left
        self._obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)

    def _camera(self, w, h):
        cal = self.calib
        if abs(w / float(h) - cal['width'] / float(cal['height'])) > 0.02:
            # another aspect ratio is another sensor crop: the matrix can't just be scaled
            same = [c for c in (DEFAULT_CALIB, DEFAULT_CALIB_16_9) if abs(w / float(h) - c['width'] / float(c['height'])) < 0.02]
            cal = same[0] if same else cal
            self.calib_note = f'frame {w}x{h} does not match the calibration ({self.calib["width"]}x{self.calib["height"]}): ' \
                              + ('using the built-in default for this aspect' if same else 'scaling anyway (inaccurate)')
        K = np.array(cal['camera_matrix'], dtype=np.float64)
        sx, sy = w / float(cal['width']), h / float(cal['height'])
        if abs(sx - 1) > 1e-6 or abs(sy - 1) > 1e-6:
            K = K.copy()
            K[0, 0] *= sx
            K[0, 2] *= sx
            K[1, 1] *= sy
            K[1, 2] *= sy
        return K, np.array(cal['dist_coeffs'], dtype=np.float64)

    def detect(self, frame):
        if self.error:
            return {'found': False, 'error': self.error}
        if frame is None:
            return {'found': False, 'error': 'no frame'}
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._dict, parameters=self._params)
        if ids is None:
            return {'found': False}
        hits = [i for i, v in enumerate(ids.flatten()) if int(v) == self.marker_id]
        if not hits:
            return {'found': False, 'others': [int(v) for v in ids.flatten()][:5]}
        c = corners[hits[0]].reshape(4, 2).astype(np.float64)
        K, dist = self._camera(gray.shape[1], gray.shape[0])
        rvec, tvec = self._solve(c, K, dist)
        if rvec is None:
            return {'found': False, 'error': 'pose not solvable'}
        R, _ = cv2.Rodrigues(rvec)
        n = R[:, 2]                         # marker z axis = its normal, out of the printed face
        if n[2] > 0:                        # always the side facing the camera
            n = -n
        t = tvec.flatten()
        return {'found': True, 'id': self.marker_id, 'x_m': round(float(t[0]), 4), 'y_m': round(float(t[1]), 4),
                'z_m': round(float(t[2]), 4), 'yaw_deg': round(yaw_from_normal(float(n[0]), float(n[2])), 2),
                'px': [round(float(c[:, 0].mean()), 1), round(float(c[:, 1].mean()), 1)],
                'side_px': round(float(np.linalg.norm(c[0] - c[1])), 1)}


    def _solve(self, c, K, dist):
        """A flat square has two poses that look almost alike (the yaw sign flips), and some OpenCV builds
        return poor IPPE solutions: collect the IPPE pair and the iterative solution, keep the one that
        reprojects the four corners best (measured here, not trusted from the solver)."""
        cands = []
        for flag in (getattr(cv2, 'SOLVEPNP_IPPE_SQUARE', None), cv2.SOLVEPNP_ITERATIVE):
            if flag is None:
                continue
            try:
                if hasattr(cv2, 'solvePnPGeneric'):
                    n, rvecs, tvecs, _ = cv2.solvePnPGeneric(self._obj, c, K, dist, flags=flag)
                    cands += [(rvecs[i], tvecs[i]) for i in range(n)]
                else:
                    ok, rv, tv = cv2.solvePnP(self._obj, c, K, dist, flags=flag)
                    if ok:
                        cands.append((rv, tv))
            except cv2.error:
                continue
        best, best_err = (None, None), float('inf')
        for rv, tv in cands:
            if float(np.array(tv).flatten()[2]) <= 0:
                continue                                   # behind the camera
            proj, _ = cv2.projectPoints(self._obj, rv, tv, K, dist)
            err = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - c) ** 2, axis=1))))
            if err < best_err:
                best, best_err = (rv, tv), err
        self.last_reproj_px = best_err
        return best


_default = None


def detect(frame, **kw):
    """Module-level interface: detect(frame) -> {found, x_m, y_m, z_m, yaw_deg}. kw rebuilds the detector."""
    global _default
    if _default is None or kw:
        _default = MarkerDetector(**kw)
    return _default.detect(frame)


def render_marker(dictionary='DICT_4X4_50', marker_id=0, px=200):
    """The marker image to print (also used by the tests and tools/marker_webcam.py --print)."""
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(d, marker_id, px)
    return cv2.aruco.drawMarker(d, marker_id, px)
