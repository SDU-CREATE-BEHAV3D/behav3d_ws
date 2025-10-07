#!/usr/bin/env python3
import os, sys, json, math, yaml
import numpy as np
import cv2

# ---------- Config ----------
SQUARES_X = 12
SQUARES_Y = 9
SQUARE_LENGTH_M = 0.03   # Charuco square size in meters
MARKER_LENGTH_M = 0.022  # Charuco marker size in meters
ARUCO_DICT_NAME = "DICT_5X5_100"

# Hardcoded path to the current session manifest
MANIFEST_PATH = "/home/lab/behav3d_ws/captures/session-20251003_113628_mancap/manifest.json"

# Resolve session_dir from the manifest (source of truth for per-session files)
with open(MANIFEST_PATH, "r") as f:
    _tmp = json.load(f)
SESSION_DIR = _tmp.get("session_dir", os.path.dirname(MANIFEST_PATH))

# Prefer CameraManager’s per-session intrinsics
CAMERA_INFO_YAML = os.path.join(SESSION_DIR, "calib", "color_intrinsics.yaml")

# Save the hand-eye result in the same session under calib/
OUT_JSON = os.path.join(SESSION_DIR, "calib", "handeye_result_final.json")
# ----------------------------

def quat_xyzw_to_R(q):
    """Convert quaternion [x,y,z,w] to a 3x3 rotation matrix."""
    x,y,z,w = q
    n = math.sqrt(x*x+y*y+z*z+w*w)
    if n == 0:
        return np.eye(3)
    x,y,z,w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]
    ])

def R_to_quat(R):
    """Convert 3x3 rotation matrix to quaternion [x,y,z,w]."""
    qw = math.sqrt(max(0, 1 + R[0,0] + R[1,1] + R[2,2])) / 2
    qx = math.sqrt(max(0, 1 + R[0,0] - R[1,1] - R[2,2])) / 2
    qy = math.sqrt(max(0, 1 - R[0,0] + R[1,1] - R[2,2])) / 2
    qz = math.sqrt(max(0, 1 - R[0,0] - R[1,1] + R[2,2])) / 2
    qx = math.copysign(qx, R[2,1] - R[1,2])
    qy = math.copysign(qy, R[0,2] - R[2,0])
    qz = math.copysign(qz, R[1,0] - R[0,1])
    return [qx, qy, qz, qw]

# --- Manifest reader (new, strict) ---

def load_manifest(path):
    """Load the new JSON manifest with fields: session_dir, captures (list)."""
    with open(path, "r") as f:
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

        # Resolve relative image paths against session root
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
    import yaml, io
    import numpy as np

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

    # Normalize OpenCV header "%YAML:1.0" -> "%YAML 1.0" for PyYAML
    with open(path, "r", encoding="utf-8") as f:
        txt = f.read()
    if txt.startswith("%YAML:"):
        txt = txt.replace("%YAML:", "%YAML ", 1)

    y = yaml.load(io.StringIO(txt), Loader=OpenCVLoader)

    # ----- K -----
    K = None
    if "K" in y: K = y["K"]
    elif "k" in y: K = y["k"]
    elif "camera_matrix" in y:
        cm = y["camera_matrix"]
        if isinstance(cm, np.ndarray):
            K = cm
        elif isinstance(cm, dict) and "data" in cm:
            K = np.array(cm["data"], dtype=np.float64).reshape(3, 3)

    # ----- D -----
    D = None
    if "D" in y: D = y["D"]
    elif "d" in y: D = y["d"]
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

def make_board():
    """Create Charuco board and detector with modern or legacy ArUco API."""
    dict_id = getattr(cv2.aruco, ARUCO_DICT_NAME)
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    board = cv2.aruco.CharucoBoard_create(SQUARES_X, SQUARES_Y,
                                          SQUARE_LENGTH_M, MARKER_LENGTH_M, aruco_dict)
    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        return board, aruco_dict, detector, None
    else:
        params = cv2.aruco.DetectorParameters_create()
        return board, aruco_dict, None, params

def collect_pairs():
    """Collect (R_bt, t_bt, R_tc, t_tc) pairs from images + poses."""
    manifest = load_manifest(MANIFEST_PATH)
    K, D = load_cam_info(CAMERA_INFO_YAML)
    board, aruco_dict, detector, legacy_params = make_board()

    pairs = []
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
        rvec = np.zeros((3,1)); tvec = np.zeros((3,1))
        try:
            ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(ch_corners, ch_ids, board, K, D)
        except cv2.error:
            ok = cv2.aruco.estimatePoseCharucoBoard(ch_corners, ch_ids, board, K, D, rvec, tvec)
        if not ok:
            continue

        R_tc,_ = cv2.Rodrigues(rvec)
        t_tc = tvec

        # base -> tool0 (from manifest pose)
        t_bt = np.array(pose["position"], dtype=np.float64).reshape(3,1)
        R_bt = quat_xyzw_to_R(pose["orientation_xyzw"])

        pairs.append((R_bt, t_bt, R_tc, t_tc))
    return pairs

def main():
    pairs = collect_pairs()
    print(f"[INFO] valid pairs: {len(pairs)}")
    if len(pairs) < 3:
        print("Too few pairs!"); sys.exit(1)

    # Use base->tool0 as gripper->base directly
    Rg2b, tg2b, Rt2c, tt2c = [], [], [], []
    for R_bt, t_bt, R_tc, t_tc in pairs:
        Rg2b.append(R_bt)
        tg2b.append(t_bt)
        Rt2c.append(R_tc)
        tt2c.append(t_tc)

    R, t = cv2.calibrateHandEye(Rg2b, tg2b, Rt2c, tt2c, method=cv2.CALIB_HAND_EYE_TSAI)
    q = R_to_quat(R)

    result = {
        "translation_m": t.ravel().tolist(),
        "quaternion_xyzw": q
    }

    print("\n=== Hand-Eye Result (eye-in-hand) ===")
    print("t (m):", result["translation_m"])
    print("quat [x,y,z,w]:", result["quaternion_xyzw"])

    # Ensure calib directory exists (just in case) and write JSON there
    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    with open(OUT_JSON, "w") as f:
        json.dump(result, f, indent=2)
    print("\n[INFO] saved to", OUT_JSON)

if __name__ == "__main__":
    main()
