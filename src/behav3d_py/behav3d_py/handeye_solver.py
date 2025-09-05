#!/usr/bin/env python3
import os
import sys
import json
import math
import io
import numpy as np
import yaml
import cv2

import rclpy
from rclpy.node import Node
from behav3d_interfaces.srv import ComputeHandEye



# ---------- Defaults (can be overridden via ROS params) ----------
ARUCO_DICT_NAME_DEFAULT = "DICT_5X5_100"
SQUARES_X_DEFAULT = 12
SQUARES_Y_DEFAULT = 9
SQUARE_LENGTH_M_DEFAULT = 0.03    # meters
MARKER_LENGTH_M_DEFAULT = 0.022   # meters
# -----------------------------------------------------------------


def quat_xyzw_to_R(q):
    """Convert quaternion [x,y,z,w] to 3x3 rotation matrix."""
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]
    ], dtype=np.float64)


def R_to_quat(R):
    """Convert 3x3 rotation matrix to quaternion [x,y,z,w]."""
    qw = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    qx = math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    qy = math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    qz = math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    qx = math.copysign(qx, R[2, 1] - R[1, 2])
    qy = math.copysign(qy, R[0, 2] - R[2, 0])
    qz = math.copysign(qz, R[1, 0] - R[0, 1])
    return [qx, qy, qz, qw]


def load_manifest(path):
    """Load manifest.json with fields: session_dir, captures (list)."""
    with open(path, "r", encoding="utf-8") as f:
        obj = json.load(f)
    session_dir = obj.get("session_dir") or os.path.dirname(path)
    captures = obj.get("captures") or []
    if not isinstance(captures, list):
        raise ValueError("Manifest 'captures' must be a list")
    return {"session_dir": session_dir, "captures": captures}


def iter_caps(manifest):
    """Yield (index, image_abs_path, pose_tool0) for valid capture entries."""
    root = manifest["session_dir"]
    for cap in manifest["captures"]:
        pose = cap.get("pose_tool0")
        img = cap.get("image_color")

        img_abs = os.path.join(root, img) if (img and not os.path.isabs(img)) else img

        # Fallback: look under color_raw/ if needed
        if not (img_abs and os.path.exists(img_abs)) and img:
            cand = os.path.join(root, "color_raw", os.path.basename(img))
            if os.path.exists(cand):
                img_abs = cand

        if img_abs and os.path.exists(img_abs) and pose is not None:
            yield cap.get("index"), img_abs, pose


def load_cam_info(path):
    """Load intrinsics from either flat K/D YAML or OpenCV FileStorage (!!opencv-matrix)."""
    class OpenCVLoader(yaml.SafeLoader):
        pass

    def _opencv_matrix_constructor(loader, node):
        mapping = loader.construct_mapping(node, deep=True)
        rows = int(mapping.get("rows", 0))
        cols = int(mapping.get("cols", 0))
        data = mapping.get("data", [])
        arr = np.array(data, dtype=np.float64)
        if rows and cols:
            arr = arr.reshape(rows, cols)
        return arr

    yaml.add_constructor("tag:yaml.org,2002:opencv-matrix",
                         _opencv_matrix_constructor,
                         Loader=OpenCVLoader)

    with open(path, "r", encoding="utf-8") as f:
        txt = f.read()
    if txt.startswith("%YAML:"):
        txt = txt.replace("%YAML:", "%YAML ", 1)

    y = yaml.load(io.StringIO(txt), Loader=OpenCVLoader)

    # K
    K = None
    if "K" in y:
        K = y["K"]
    elif "k" in y:
        K = y["k"]
    elif "camera_matrix" in y:
        cm = y["camera_matrix"]
        if isinstance(cm, np.ndarray):
            K = cm
        elif isinstance(cm, dict) and "data" in cm:
            K = np.array(cm["data"], dtype=np.float64).reshape(3, 3)

    # D
    D = None
    if "D" in y:
        D = y["D"]
    elif "d" in y:
        D = y["d"]
    elif "distortion_coefficients" in y:
        dc = y["distortion_coefficients"]
        if isinstance(dc, np.ndarray):
            D = dc.reshape(-1)
        elif isinstance(dc, dict) and "data" in dc:
            D = np.array(dc["data"], dtype=np.float64).reshape(-1)
    elif "distortion" in y:
        dc = y["distortion"]
        if isinstance(dc, np.ndarray):
            D = dc.reshape(-1)
        elif isinstance(dc, dict) and "data" in dc:
            D = np.array(dc["data"], dtype=np.float64).reshape(-1)

    if K is None or D is None:
        raise ValueError(f"Could not extract intrinsics from {path}")

    return np.array(K, dtype=np.float64).reshape(3, 3), np.array(D, dtype=np.float64).reshape(-1)


def make_board(dict_name, sx, sy, s_len_m, m_len_m):
    """Create Charuco board and detector with modern or legacy ArUco API."""
    dict_id = getattr(cv2.aruco, dict_name)
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    board = cv2.aruco.CharucoBoard_create(sx, sy, s_len_m, m_len_m, aruco_dict)
    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        return board, aruco_dict, detector, None
    else:
        params = cv2.aruco.DetectorParameters_create()
        return board, aruco_dict, None, params


def collect_pairs(manifest_path, camera_info_yaml, dict_name, sx, sy, s_len_m, m_len_m):
    """Build (R_bt, t_bt, R_tc, t_tc) pairs from images + poses."""
    manifest = load_manifest(manifest_path)
    K, D = load_cam_info(camera_info_yaml)
    board, aruco_dict, detector, legacy_params = make_board(dict_name, sx, sy, s_len_m, m_len_m)

    pairs = []
    valid = 0
    for idx, img_path, pose in iter_caps(manifest):
        img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Detect markers
        if detector:
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=legacy_params)
        if ids is None or len(ids) == 0:
            continue

        # Interpolate ChArUco corners
        ret, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
        if ch_ids is None or len(ch_ids) < 6:
            continue

        # Estimate board pose in camera
        rvec = np.zeros((3, 1)); tvec = np.zeros((3, 1))
        try:
            ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(ch_corners, ch_ids, board, K, D)
        except cv2.error:
            ok = cv2.aruco.estimatePoseCharucoBoard(ch_corners, ch_ids, board, K, D, rvec, tvec)
        if not ok:
            continue

        R_tc, _ = cv2.Rodrigues(rvec)
        t_tc = tvec

        # base -> tool0 (from manifest pose)
        t_bt = np.array(pose["position"], dtype=np.float64).reshape(3, 1)
        R_bt = quat_xyzw_to_R(pose["orientation_xyzw"])

        pairs.append((R_bt, t_bt, R_tc, t_tc))
        valid += 1

    return pairs, valid


class HandeyeSolverNode(Node):
    def __init__(self):
        super().__init__("handeye_solver")

        # Declare parameters (so you can tune without code changes)
        self.declare_parameter("aruco_dict_name", ARUCO_DICT_NAME_DEFAULT)
        self.declare_parameter("squares_x", SQUARES_X_DEFAULT)
        self.declare_parameter("squares_y", SQUARES_Y_DEFAULT)
        self.declare_parameter("square_length_m", SQUARE_LENGTH_M_DEFAULT)
        self.declare_parameter("marker_length_m", MARKER_LENGTH_M_DEFAULT)

        self._srv = self.create_service(ComputeHandEye, "compute_handeye", self.on_compute)
        self._shutdown_timer = None
        self.get_logger().info("handeye_solver ready: service '~/compute_handeye'")

    def on_compute(self, request, response):
        """
        request.session_dir: absolute path to session dir
        request.one_shot:    solver will shutdown after replying if True
        """
        try:
            session_dir = request.session_dir
            if not session_dir or not os.path.isdir(session_dir):
                raise RuntimeError(f"Invalid session_dir: {session_dir}")

            manifest_path = os.path.join(session_dir, "manifest.json")
            if not os.path.exists(manifest_path):
                raise RuntimeError(f"manifest.json not found at {manifest_path}")

            camera_info_yaml = os.path.join(session_dir, "calib", "color_intrinsics.yaml")
            if not os.path.exists(camera_info_yaml):
                raise RuntimeError(f"color_intrinsics.yaml not found at {camera_info_yaml}")

            dict_name = self.get_parameter("aruco_dict_name").get_parameter_value().string_value
            sx = int(self.get_parameter("squares_x").get_parameter_value().integer_value)
            sy = int(self.get_parameter("squares_y").get_parameter_value().integer_value)
            s_len = float(self.get_parameter("square_length_m").get_parameter_value().double_value)
            m_len = float(self.get_parameter("marker_length_m").get_parameter_value().double_value)

            pairs, n_valid = collect_pairs(
                manifest_path, camera_info_yaml, dict_name, sx, sy, s_len, m_len
            )

            if len(pairs) < 3:
                raise RuntimeError(f"Too few valid pairs: {len(pairs)} (need >= 3)")

            Rg2b, tg2b, Rt2c, tt2c = [], [], [], []
            for R_bt, t_bt, R_tc, t_tc in pairs:
                Rg2b.append(R_bt); tg2b.append(t_bt)
                Rt2c.append(R_tc);  tt2c.append(t_tc)

            R, t = cv2.calibrateHandEye(
                Rg2b, tg2b, Rt2c, tt2c, method=cv2.CALIB_HAND_EYE_TSAI
            )
            q = R_to_quat(R)

            # Save result alongside intrinsics
            out_json = os.path.join(session_dir, "calib", "handeye_result_final.json")
            os.makedirs(os.path.dirname(out_json), exist_ok=True)
            result = {
                "translation_m": t.ravel().tolist(),
                "quaternion_xyzw": q,
                "method": "CALIB_HAND_EYE_TSAI",
                "num_pairs": int(len(pairs)),
                "aruco": {
                    "dict": dict_name, "squares_x": sx, "squares_y": sy,
                    "square_length_m": s_len, "marker_length_m": m_len
                }
            }
            with open(out_json, "w", encoding="utf-8") as f:
                json.dump(result, f, indent=2)

            # Fill service response
            response.t_m = t.ravel().tolist()
            response.q_xyzw = q
            response.num_pairs = int(len(pairs))
            response.rms_px = 0.0  # not computed here
            response.out_json = out_json
            response.ok = True
            response.message = "ok"

            self.get_logger().info(
                f"Hand-eye done. t={response.t_m}, q={response.q_xyzw} -> {out_json}"
            )

        except Exception as e:
            response.ok = False
            response.message = str(e)
            response.t_m = [0.0, 0.0, 0.0]
            response.q_xyzw = [0.0, 0.0, 0.0, 1.0]
            response.num_pairs = 0
            response.rms_px = 0.0
            response.out_json = ""
            self.get_logger().error(f"Hand-eye failed: {e}")

        # one-shot: schedule shutdown after replying
        if request.one_shot:
            self.get_logger().info("one_shot=True → shutting down after response.")
            # Defer shutdown slightly to ensure response is sent
            if self._shutdown_timer is None:
                self._shutdown_timer = self.create_timer(0.25, self._do_shutdown)

        return response

    def _do_shutdown(self):
        try:
            if self._shutdown_timer:
                self._shutdown_timer.cancel()
                self._shutdown_timer = None
        finally:
            self.get_logger().info("handeye_solver: shutdown.")
            rclpy.shutdown()


def main(argv=None):
    rclpy.init(args=argv)
    node = HandeyeSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
