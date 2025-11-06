#!/usr/bin/env python3
# handeye_quick_ir.py
# Eye-in-hand hand–eye calibration with ChArUco for IR (default) or RGB streams.
#
# NEW: reads your YAML manifest (preferred) with keys:
#   session.path (abs), captures[].ir / rgb / depth, captures[].pose_tool0.{orientation_xyzw, position}
# Still supports the old JSON manifest as a fallback.
#
# Intrinsics default to FIXED ROS paths (below). If missing, falls back to per-session calib/*.yaml.
#
# Output:
#   SESS_PATH/calib/handeye_result.json
#   SESS_PATH/calib/tool0_from_cam.yaml
#   (optional) annotated previews saved next to source images

import os
import sys
import json
import math
import argparse
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R


# ---------- Session / options ----------
SESS_PATH = "/home/lab/behav3d_ws/captures/251104_140521/manual_caps"  # set your session folder here
PREVIEW = True
METHOD = "Daniilidis"         # Tsai | Park | Horaud | Andreff | Daniilidis
STREAM_DEFAULT = "color"   # "ir" or "color"

# ---------- Fixed intrinsics (default) ----------
# These are the new fixed ROS paths you gave me.
FIXED_INTRINSICS_IR    = "/home/lab/behav3d_ws/src/custom_workcell/ur20_workcell/config/bolt_intrinsics/ir_intrinsics.yaml"
FIXED_INTRINSICS_COLOR = "/home/lab/behav3d_ws/src/custom_workcell/ur20_workcell/config/bolt_intrinsics/color_intrinsics.yaml"

# ---------- Board config (unchanged) ----------
SQUARES_X = 6
SQUARES_Y = 5
SQUARE_LENGTH_M = 0.055
MARKER_LENGTH_M = 0.041
ARUCO_DICT_NAME = "DICT_5X5_100"

_ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

def get_dict(name: str):
    return cv2.aruco.getPredefinedDictionary(_ARUCO_DICTS[name])

def make_board(dictionary):
    # Use legacy CharucoBoard_create for compatibility with older OpenCV builds
    return cv2.aruco.CharucoBoard_create(
        squaresX=SQUARES_X,
        squaresY=SQUARES_Y,
        squareLength=SQUARE_LENGTH_M,
        markerLength=MARKER_LENGTH_M,
        dictionary=dictionary,
    )

# ---------- Intrinsics ----------
def load_intrinsics(yaml_path: str):
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise IOError(f"Failed to open intrinsics YAML: {yaml_path}")
    try:
        K = fs.getNode("camera_matrix").mat()
        D = fs.getNode("distortion_coefficients").mat()
        if K is None or D is None:
            raise ValueError("Missing camera_matrix or distortion_coefficients in YAML.")
        K = np.asarray(K, dtype=np.float64).reshape(3, 3)
        D = np.asarray(D, dtype=np.float64).ravel()
        return K, D
    finally:

        fs.release()

# ---------- IR-aware loading + preprocessing ----------
def load_ir_image(path: str):
    """Load IR image preserving depth; return (bgr_for_viz, gray_u8)."""
    img_raw = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img_raw is None:
        return None, None
    if img_raw.dtype == np.uint16:
        ir16 = img_raw
        p1, p99 = np.percentile(ir16, (1, 99))
        if p99 <= p1:
            p1, p99 = ir16.min(), ir16.max()
        ir16 = np.clip(ir16, p1, p99)
        gray = ((ir16 - p1) * (255.0 / (p99 - p1 + 1e-9))).astype(np.uint8)
        bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    elif len(img_raw.shape) == 2:
        gray = img_raw.astype(np.uint8, copy=False)
        bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    else:
        bgr = img_raw
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return bgr, gray

def preprocess_ir(gray_u8: np.ndarray, invert: bool=False, use_clahe: bool=True):
    g = gray_u8

    # Optional inversion if black-white polarity flips
    if invert:
        g = cv2.bitwise_not(g)

    # Mild denoising to reduce sensor speckle
    g = cv2.bilateralFilter(g, d=5, sigmaColor=25, sigmaSpace=25)

    # Adaptive contrast enhancement
    if use_clahe:
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        g = clahe.apply(g)

    # Local sharpening to enhance edges of markers
    g = cv2.GaussianBlur(g, (0, 0), 1.0)
    g = cv2.addWeighted(gray_u8, 1.5, g, -0.5, 0)

    return g


def make_detector_params():
    p = cv2.aruco.DetectorParameters_create()

    # Thresholding (reduce noise sensitivity)
    p.adaptiveThreshWinSizeMin = 5
    p.adaptiveThreshWinSizeMax = 25
    p.adaptiveThreshWinSizeStep = 5
    p.adaptiveThreshConstant = 7

    # Stricter marker geometry limits
    p.minMarkerPerimeterRate = 0.04     # smaller markers rejected
    p.maxMarkerPerimeterRate = 0.3      # exclude full checker areas
    p.minCornerDistanceRate = 0.1
    p.minMarkerDistanceRate = 0.05

    # Require strong, clean contours
    p.polygonalApproxAccuracyRate = 0.04
    p.minOtsuStdDev = 7.0
    p.perspectiveRemovePixelPerCell = 8

    # Refine only after valid ArUco detection
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 5
    p.cornerRefinementMaxIterations = 200
    p.cornerRefinementMinAccuracy = 0.001

    # Lower tolerance for border errors
    p.maxErroneousBitsInBorderRate = 0.2
    p.errorCorrectionRate = 0.5

    return p


# ---------- ChArUco detection ----------
def detect_charuco_pose(image_path: str, K, D, dictionary, board, stream: str, undistort: bool, invert: bool, no_clahe: bool):
    if stream == "ir":
        bgr, gray0 = load_ir_image(image_path)
        if bgr is None:
            raise IOError(f"Could not read image: {image_path}")
        if undistort:
            bgr = cv2.undistort(bgr, K, D)
            gray0 = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = preprocess_ir(gray0, invert=invert, use_clahe=not no_clahe)
    else:
        img = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if img is None:
            raise IOError(f"Could not read image: {image_path}")
        bgr = img
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if undistort:
            bgr = cv2.undistort(bgr, K, D)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    # Legacy API for compatibility with OpenCV < 4.7
    params = make_detector_params() if stream == "ir" else cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

    retval = {
        "n_markers": 0 if ids is None else len(ids),
        "n_charuco": 0,
        "ok_pose": False,
        "rvec": None,
        "tvec": None,
    }
    if ids is None or len(ids) == 0:
        return retval, bgr

    out = cv2.aruco.interpolateCornersCharuco(
        markerCorners=corners,
        markerIds=ids,
        image=gray,
        board=board,
        cameraMatrix=K,
        distCoeffs=D,
    )
    # OpenCV returns (retval, corners, ids) in some versions; in others, just (corners, ids)
    if isinstance(out, tuple) and len(out) >= 3:
        _, ch_corners, ch_ids = out
    else:
        ch_corners, ch_ids = out

    if ch_corners is None or ch_ids is None or len(ch_ids) < 4:
        retval["n_charuco"] = 0 if ch_ids is None else int(len(ch_ids))
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
        return retval, bgr

    retval["n_charuco"] = int(len(ch_ids))
    try:
        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charucoCorners=ch_corners,
            charucoIds=ch_ids,
            board=board,
            cameraMatrix=K,
            distCoeffs=D,
        )
        if ok:
            retval["ok_pose"] = True
            retval["rvec"] = rvec
            retval["tvec"] = tvec
    except Exception:
        pass

    if not retval["ok_pose"]:
        corners3d = np.array(board.chessboardCorners, dtype=np.float32).reshape(-1, 3)
        obj_pts = corners3d[ch_ids.ravel().astype(int)]
        img_pts = ch_corners.reshape(-1, 2)
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_ITERATIVE
        )
        retval["ok_pose"] = bool(ok)
        if ok:
            retval["rvec"] = rvec
            retval["tvec"] = tvec

    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
    if retval["ok_pose"]:
        try:
            cv2.drawFrameAxes(bgr, K, D, retval["rvec"], retval["tvec"], 0.1)
        except Exception:
            pass
    return retval, bgr

# ---------- Geometry helpers ----------
def quat_xyzw_to_R(qx, qy, qz, qw):
    nq = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if nq == 0:
        raise ValueError("Zero quaternion")
    qx, qy, qz, qw = qx / nq, qy / nq, qz / nq, qw / nq
    R = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=float,
    )
    return R

def R_to_quat_xyzw(R: np.ndarray):
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S
    q = np.array([x, y, z, w], dtype=float)
    q = q / np.linalg.norm(q)
    return q

def rvec_tvec_to_RT(rvec, tvec):
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float))
    t = np.asarray(tvec, dtype=float).reshape(3, 1)
    return R, t

def RT_to_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)
    return T

def euler_ypr(R):
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    pitch = math.degrees(math.atan2(-R[2, 0], math.hypot(R[2, 1], R[2, 2])))
    roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
    return yaw, pitch, roll

# ---------- Manifest helpers ----------
def _manifest_yaml_path(sess_path: str):
    p1 = os.path.join(sess_path, "manifest.yaml")
    p2 = os.path.join(sess_path, "manifest.yml")
    return p1 if os.path.isfile(p1) else (p2 if os.path.isfile(p2) else None)

def _manifest_json_path(sess_path: str):
    p = os.path.join(sess_path, "manifest.json")
    return p if os.path.isfile(p) else None

def _load_captures_from_yaml(sess_path: str, stream: str):
    # Expected schema (as you provided):
    # session: { path: <abs> }
    # captures:
    #   - ir:   ir_raw/ir_i0.png
    #     rgb:  color_raw/rgb_i0.png
    #     pose_tool0: { orientation_xyzw: [...], position: [...] }
    import yaml
    with open(_manifest_yaml_path(sess_path), "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    caps_in = data.get("captures", [])
    key = "ir" if stream == "ir" else "rgb"
    caps = []
    for c in caps_in:
        img_rel = c.get(key)
        pose = c.get("pose_tool0") or {}
        q = pose.get("orientation_xyzw")
        p = pose.get("position")
        if img_rel is None or q is None or p is None:
            continue
        img_path = img_rel if os.path.isabs(img_rel) else os.path.join(sess_path, img_rel)
        caps.append({
            "image_path": img_path,
            "q": tuple(q),
            "p": tuple(p),
        })
    return caps

def _load_captures_from_json(sess_path: str, stream: str):
    with open(_manifest_json_path(sess_path), "r", encoding="utf-8") as f:
        data = json.load(f)
    caps_in = data.get("captures", [])
    key = "image_ir" if stream == "ir" else "image_color"
    alt_key = "ir" if stream == "ir" else "rgb"
    caps = []
    for c in caps_in:
        img_rel = c.get(key) or c.get(alt_key) or c.get("img_ir") or c.get("img_color") or c.get("color")
        pose = c.get("pose_tool0") or c.get("tool0_pose") or c.get("tcp_pose") or {}
        q = pose.get("orientation_xyzw") or pose.get("orientation")
        p = pose.get("position")
        if img_rel is None or q is None or p is None:
            continue
        img_path = img_rel if os.path.isabs(img_rel) else os.path.join(sess_path, img_rel)
        caps.append({
            "image_path": img_path,
            "q": tuple(q),
            "p": tuple(p),
        })
    return caps

def _choose_intrinsics(sess_path: str, stream: str, override: str | None):
    if override:
        return override
    fixed = FIXED_INTRINSICS_IR if stream == "ir" else FIXED_INTRINSICS_COLOR
    if os.path.isfile(fixed):
        return fixed
    # fallback to legacy per-session calib file
    name = "ir_intrinsics.yaml" if stream == "ir" else "color_intrinsics.yaml"
    legacy = os.path.join(sess_path, "calib", name)
    return legacy
def euler_rpy_deg(R_3x3):
    # Intrinsic XYZ (roll→pitch→yaw)
    return R.from_matrix(R_3x3).as_euler('xyz', degrees=True)  # [roll, pitch, yaw]

def euler_rpy_rad(R_3x3):
    # Same sequence, in radians
    return R.from_matrix(R_3x3).as_euler('xyz', degrees=False) # [roll, pitch, yaw]
# ---------- Interactive tuning ----------
def interactive_tune_first_image(first_image_path, K, D, dictionary, board, captures=None):
    import os
    def nothing(x): pass

    cv2.namedWindow("tune", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("tune", 1200, 800)

    # --- Trackbars ---
    cv2.createTrackbar("invert", "tune", 0, 1, nothing)
    cv2.createTrackbar("use_clahe", "tune", 1, 1, nothing)
    cv2.createTrackbar("clipLimit", "tune", 25, 100, nothing)
    cv2.createTrackbar("tileGrid", "tune", 8, 20, nothing)
    cv2.createTrackbar("bil_d", "tune", 5, 15, nothing)
    cv2.createTrackbar("bil_sig", "tune", 25, 100, nothing)
    cv2.createTrackbar("sharp_w", "tune", 15, 30, nothing)
    cv2.createTrackbar("th_min", "tune", 5, 20, nothing)
    cv2.createTrackbar("th_max", "tune", 25, 50, nothing)
    cv2.createTrackbar("th_step", "tune", 2, 10, nothing)
    cv2.createTrackbar("minP", "tune", 4, 20, nothing)
    cv2.createTrackbar("maxP", "tune", 30, 100, nothing)

    idx = 0
    total = len(captures) if captures else 1

    while True:
        image_path = captures[idx]["image_path"] if captures else first_image_path
        bgr, gray0 = load_ir_image(image_path)
        if bgr is None:
            idx = (idx + 1) % total
            continue

        # --- Read trackbars ---
        invert = cv2.getTrackbarPos("invert", "tune") == 1
        use_clahe = cv2.getTrackbarPos("use_clahe", "tune") == 1
        clip = cv2.getTrackbarPos("clipLimit", "tune") / 10.0
        tile = max(1, cv2.getTrackbarPos("tileGrid", "tune"))
        bil_d = max(1, cv2.getTrackbarPos("bil_d", "tune"))
        bil_sig = cv2.getTrackbarPos("bil_sig", "tune")
        sharp_w = cv2.getTrackbarPos("sharp_w", "tune") / 10.0
        th_min = max(3, cv2.getTrackbarPos("th_min", "tune"))
        th_max = max(th_min + 2, cv2.getTrackbarPos("th_max", "tune"))
        th_step = max(1, cv2.getTrackbarPos("th_step", "tune"))
        minP = max(0.01, cv2.getTrackbarPos("minP", "tune") / 100.0)
        maxP = max(minP + 0.05, cv2.getTrackbarPos("maxP", "tune") / 100.0)

        # --- Preprocess ---
        g = gray0.copy()
        if invert:
            g = cv2.bitwise_not(g)
        g = cv2.bilateralFilter(g, bil_d, bil_sig, bil_sig)
        if use_clahe:
            clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile))
            g = clahe.apply(g)
        blur = cv2.GaussianBlur(g, (0, 0), 1.0)
        g = cv2.addWeighted(g, sharp_w, blur, -0.5, 0)

        # --- Detect ---
        params = cv2.aruco.DetectorParameters_create()
        params.adaptiveThreshWinSizeMin = th_min
        params.adaptiveThreshWinSizeMax = th_max
        params.adaptiveThreshWinSizeStep = th_step
        params.minMarkerPerimeterRate = minP
        params.maxMarkerPerimeterRate = maxP
        params.polygonalApproxAccuracyRate = 0.02
        params.minCornerDistanceRate = 0.05
        params.minDistanceToBorder = 3

        corners, ids, _ = cv2.aruco.detectMarkers(g, dictionary, parameters=params)
        img_show = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(img_show, corners, ids)
        cv2.putText(
            img_show,
            f"{idx+1}/{total}: {os.path.basename(image_path)}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
        )

        cv2.imshow("tune", img_show)
        k = cv2.waitKey(50) & 0xFF
        if k == 27:  # ESC -> finish
            break
        elif k in (ord("n"), ord(" ")):  # next
            idx = (idx + 1) % total
        elif k == ord("p"):  # previous
            idx = (idx - 1) % total

    cv2.destroyWindow("tune")
    return {
        "invert": invert,
        "use_clahe": use_clahe,
        "clipLimit": clip,
        "tileGrid": tile,
        "bil_d": bil_d,
        "bil_sig": bil_sig,
        "sharp_w": sharp_w,
        "th_min": th_min,
        "th_max": th_max,
        "th_step": th_step,
        "minP": minP,
        "maxP": maxP,
    }


# ---------- Main ----------
def main():
    parser = argparse.ArgumentParser(description="Hand–eye calibration (IR/RGB) with ChArUco.")
    parser.add_argument("--sess", default=SESS_PATH, help="Session folder path")
    parser.add_argument("--method", default=METHOD, choices=["Tsai","Park","Horaud","Andreff","Daniilidis"])
    parser.add_argument("--stream", default=STREAM_DEFAULT, choices=["ir","color"])
    parser.add_argument("--preview", action="store_true", default=PREVIEW)
    parser.add_argument("--undistort", action="store_true", help="Undistort before detection")
    parser.add_argument("--invert", action="store_true", help="Invert IR intensities (useful on some boards)")
    parser.add_argument("--no-clahe", action="store_true", help="Disable CLAHE on IR preprocessing")
    parser.add_argument("--intr", default=None, help="Override intrinsics YAML path")
    args = parser.parse_args()

    sess = os.path.abspath(args.sess)

    # Resolve intrinsics
    intr_path = _choose_intrinsics(sess, args.stream, args.intr)
    if not os.path.isfile(intr_path):
        print(f"[error] intrinsics not found: {intr_path}")
        return 3

    # Resolve manifest (YAML preferred, JSON fallback)
    mp_yaml = _manifest_yaml_path(sess)
    mp_json = _manifest_json_path(sess)
    if not mp_yaml and not mp_json:
        print(f"[error] manifest not found in: {sess}")
        return 2

    captures = _load_captures_from_yaml(sess, args.stream) if mp_yaml else _load_captures_from_json(sess, args.stream)
    if len(captures) < 2:
        print("[error] Need at least 2 captures for hand–eye")
        return 4

    # Calibration objects
    K, D = load_intrinsics(intr_path)
    dictionary = get_dict(ARUCO_DICT_NAME)
    board = make_board(dictionary)

    # ---------- Interactive tuning ----------
    tuned = interactive_tune_first_image(captures[0]["image_path"], K, D, dictionary, board, captures)

    # Override global references so detect_charuco_pose() uses the tuned logic
    globals()["preprocess_ir"] = lambda gray_u8, invert=tuned["invert"], use_clahe=tuned["use_clahe"]: (
        cv2.addWeighted(
            (lambda g:
                (cv2.createCLAHE(
                    clipLimit=tuned["clipLimit"],
                    tileGridSize=(tuned["tileGrid"], tuned["tileGrid"])
                ).apply(g))
                if tuned["use_clahe"] else g
            )(
                cv2.bilateralFilter(
                    cv2.bitwise_not(gray_u8) if tuned["invert"] else gray_u8,
                    tuned["bil_d"], tuned["bil_sig"], tuned["bil_sig"]
                )
            ),
            tuned["sharp_w"], 
            cv2.GaussianBlur(gray_u8, (0,0), 1.0), 
            -0.5, 0
        )
    )

    def make_detector_params():
        p = cv2.aruco.DetectorParameters_create()
        p.adaptiveThreshWinSizeMin = tuned["th_min"]
        p.adaptiveThreshWinSizeMax = tuned["th_max"]
        p.adaptiveThreshWinSizeStep = tuned["th_step"]
        p.minMarkerPerimeterRate = tuned["minP"]
        p.maxMarkerPerimeterRate = tuned["maxP"]
        return p
    globals()["make_detector_params"] = make_detector_params

    # ---------- Main calibration loop ----------
    R_gripper2base, t_gripper2base = [], []
    R_target2cam, t_target2cam = [], []
    used = 0

    for cap in captures:
        img_path = cap["image_path"]
        qx, qy, qz, qw = cap["q"]
        px, py, pz = cap["p"]
                    # print K and D values for debugging
        # print(f"[debug] Using intrinsics K:\n{K}\nD:\n{D}")

        if not os.path.isfile(img_path):
            print(f"[warn] Missing image file: {img_path}")
            continue

        R_b_t = quat_xyzw_to_R(qx, qy, qz, qw)
        t_b_t = np.array([[px], [py], [pz]], dtype=float)

        det, img_annot = detect_charuco_pose(
            img_path, K, D, dictionary, board,
            stream=args.stream, undistort=args.undistort,
            invert=args.invert, no_clahe=args.no_clahe

        

        )
        if not det["ok_pose"]:
            print(
                f"[warn] Skip {os.path.basename(img_path)}: "
                f"markers={det['n_markers']} charuco={det['n_charuco']}"
            )
            if args.preview and img_annot is not None:
                out_img = os.path.join(os.path.dirname(img_path), f"annot_2{os.path.basename(img_path)}")
                try:
                    cv2.imwrite(out_img, img_annot)
                except Exception:
                    pass
            continue



        R_c_brd, t_c_brd = rvec_tvec_to_RT(det["rvec"], det["tvec"])
        R_gripper2base.append(R_b_t)
        t_gripper2base.append(t_b_t)
        R_target2cam.append(R_c_brd)
        t_target2cam.append(t_c_brd)
        used += 1

        # --- Save T_cam→board for this capture ---
        T_cam2board = RT_to_T(R_c_brd, t_c_brd)
        if "T_cam2board_all" not in locals():
            T_cam2board_all = []
        T_cam2board_all.append({
            "image_path": img_path,
            "T_cam2board": T_cam2board.tolist()
        })

        if args.preview and img_annot is not None:
            out_img = os.path.join(os.path.dirname(img_path), f"annot_2{os.path.basename(img_path)}")
            try:
                cv2.imwrite(out_img, img_annot)
            except Exception:
                pass

    if used < 2:
        print("[error] Not enough valid detections")
        return 5

    method_map = {
        "Tsai": cv2.CALIB_HAND_EYE_TSAI,
        "Park": cv2.CALIB_HAND_EYE_PARK,
        "Horaud": cv2.CALIB_HAND_EYE_HORAUD,
        "Andreff": cv2.CALIB_HAND_EYE_ANDREFF,
        "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    try:
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base,
            t_gripper2base,
            R_target2cam,
            t_target2cam,
            method=method_map[args.method],
        )
    except AttributeError:
        print("[error] Your OpenCV build lacks calibrateHandEye.")
        return 6

    # tool0 <- cam
    R_t_c = R_cam2gripper
    t_t_c = t_cam2gripper

    yaw, pitch, roll = euler_ypr(R_t_c)
    quat_xyzw = R_to_quat_xyzw(R_t_c)
    rpy_deg = euler_rpy_deg(R_t_c)
    rpy_rad = euler_rpy_rad(R_t_c)

    print("[ok] Hand–eye (tool0 <- cam):")
    print("  t [m]:", t_t_c.ravel())
    print(f"  Euler YPR [deg]: [{yaw:.5f}, {pitch:.5f}, {roll:.5f}]")
    print(f"  Euler RPY [deg]: [{rpy_deg[0]:.5f}, {rpy_deg[1]:.5f}, {rpy_deg[2]:.5f}]")
    print(f"  Euler RPY [rad]: [{rpy_rad[0]:.7f} {rpy_rad[1]:.7f} {rpy_rad[2]:.7f}]")
    print("  quat_xyzw:", quat_xyzw)
    print(f"  detections used: {used} / {len(captures)}")

    # Residuals AX ≈ XB
    Ts_b_t = [RT_to_T(R_gripper2base[i], t_gripper2base[i]) for i in range(used)]
    Ts_c_b = [RT_to_T(R_target2cam[i], t_target2cam[i]) for i in range(used)]
    T_t_c = RT_to_T(R_t_c, t_t_c)

    rot_errs, trans_errs = [], []
    for i in range(1, used):
        A = np.linalg.inv(Ts_b_t[i - 1]) @ Ts_b_t[i]
        B = np.linalg.inv(Ts_c_b[i - 1]) @ Ts_c_b[i]
        lhs = A @ T_t_c
        rhs = T_t_c @ B
        dT = np.linalg.inv(rhs) @ lhs
        dR = dT[:3, :3]
        angle = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2))))
        dt = np.linalg.norm(dT[:3, 3])
        rot_errs.append(angle)
        trans_errs.append(dt)

    if rot_errs:
        med_ang = float(np.median(rot_errs))
        med_mm = float(np.median(trans_errs) * 1000)
        print(f"[residuals] median rotation: {med_ang:.3f} deg, median translation: {med_mm:.2f} mm")

    # Save outputs
    os.makedirs(os.path.join(sess, "calib"), exist_ok=True)
    out_json = os.path.join(sess, "calib", "handeye_result.json")
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(
            {
                "method": args.method,
                "stream": args.stream,
                "tool0_from_cam": {
                    "R": R_t_c.tolist(),
                    "t": t_t_c.reshape(3).tolist(),
                    "euler_ypr_deg": [yaw, pitch, roll],
                    "orientation_xyzw": quat_xyzw.tolist(),
                },
                "detections_used": used,
                "intrinsics_yaml": intr_path,
            },
            f,
            indent=2,
        )
    print(f"[ok] Saved JSON: {out_json}")

    yaml_path = os.path.join(sess, "calib", "tool0_from_cam.yaml")
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_WRITE)
    fs.write("R", np.asarray(R_t_c, dtype=float))
    fs.write("t", np.asarray(t_t_c, dtype=float))
    fs.write("orientation_xyzw", np.asarray(quat_xyzw, dtype=float))
    fs.release()
    print(f"[ok] Saved YAML: {yaml_path}")

    if "T_cam2board_all" in locals():
        out_cam2board = os.path.join(sess, "calib", "T_cam2board_list.json")
        with open(out_cam2board, "w", encoding="utf-8") as f:
            json.dump(T_cam2board_all, f, indent=2)
        print(f"[ok] Saved T_cam→board list: {out_cam2board}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
