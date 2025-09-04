#!/usr/bin/env python3
import os, sys, json, math, yaml
import numpy as np
import cv2

# ---------- Config ----------
SQUARES_X = 12
SQUARES_Y = 9
SQUARE_LENGTH_M = 0.03   # mide con regla, en metros
MARKER_LENGTH_M = 0.022
ARUCO_DICT_NAME = "DICT_5X5_100"

MANIFEST_PATH = "/home/lab/behav3d_ws/captures/session-20250904_170940_mancap/manifest.json"
CAMERA_INFO_YAML = "/home/lab/behav3d_ws/color_camera_info.yaml"
OUT_JSON = "handeye_result_final.json"
# ----------------------------

def quat_xyzw_to_R(q):
    x,y,z,w = q
    n = math.sqrt(x*x+y*y+z*z+w*w)
    if n == 0: return np.eye(3)
    x,y,z,w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]
    ])

def R_to_quat(R):
    """Convierte matriz 3x3 a quaternion (x,y,z,w)."""
    qw = math.sqrt(max(0, 1 + R[0,0] + R[1,1] + R[2,2])) / 2
    qx = math.sqrt(max(0, 1 + R[0,0] - R[1,1] - R[2,2])) / 2
    qy = math.sqrt(max(0, 1 - R[0,0] + R[1,1] - R[2,2])) / 2
    qz = math.sqrt(max(0, 1 - R[0,0] - R[1,1] + R[2,2])) / 2
    qx = math.copysign(qx, R[2,1] - R[1,2])
    qy = math.copysign(qy, R[0,2] - R[2,0])
    qz = math.copysign(qz, R[1,0] - R[0,1])
    return [qx, qy, qz, qw]

def load_manifest(path):
    with open(path) as f: txt = f.read()
    caps, buf, depth = [], [], 0
    for ch in txt:
        if ch == "{":
            if depth == 0: buf = []
            depth += 1
        if depth > 0: buf.append(ch)
        if ch == "}":
            depth -= 1
            if depth == 0:
                try:
                    obj = json.loads("".join(buf))
                    if "index" in obj: caps.append(obj)
                except: pass
    return {"session_dir": os.path.dirname(path), "captures": caps}

def iter_caps(manifest):
    root = manifest["session_dir"]
    for cap in manifest["captures"]:
        img = cap.get("image_color")
        if img and not os.path.isabs(img):
            img = os.path.join(root, img)
        if not (img and os.path.exists(img)):
            cand = os.path.join(root, "color_raw", os.path.basename(img))
            if os.path.exists(cand): img = cand
        if img and os.path.exists(img):
            yield cap["index"], img, cap.get("pose_tool0")

def load_cam_info(path):
    with open(path) as f: y = yaml.safe_load(f)
    K = np.array(y.get("k") or y.get("K"), dtype=np.float64).reshape(3,3)
    D = np.array(y.get("d") or y.get("D"), dtype=np.float64).reshape(-1)
    return K, D

def make_board():
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
    manifest = load_manifest(MANIFEST_PATH)
    K,D = load_cam_info(CAMERA_INFO_YAML)
    board, aruco_dict, detector, legacy_params = make_board()

    pairs = []
    for idx, img_path, pose in iter_caps(manifest):
        img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        if img is None or pose is None: continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if detector:
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=legacy_params)
        if ids is None or len(ids) == 0: continue

        ret, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
        if ch_ids is None or len(ch_ids) < 6: continue

        rvec = np.zeros((3,1)); tvec = np.zeros((3,1))
        try:
            ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(ch_corners, ch_ids, board, K, D)
        except cv2.error:
            ok = cv2.aruco.estimatePoseCharucoBoard(ch_corners, ch_ids, board, K, D, rvec, tvec)
        if not ok: continue

        R_tc,_ = cv2.Rodrigues(rvec)
        t_tc = tvec

        t_bt = np.array(pose["position"], dtype=np.float64).reshape(3,1)
        R_bt = quat_xyzw_to_R(pose["orientation_xyzw"])

        pairs.append((R_bt, t_bt, R_tc, t_tc))
    return pairs

def main():
    pairs = collect_pairs()
    print(f"[INFO] pares válidos: {len(pairs)}")
    if len(pairs) < 3:
        print("muy pocos pares!"); sys.exit(1)

    # usamos directamente base->tool como gripper->base
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

    with open(OUT_JSON,"w") as f: json.dump(result,f,indent=2)
    print("\n[INFO] guardado en", OUT_JSON)

if __name__ == "__main__":
    main()
