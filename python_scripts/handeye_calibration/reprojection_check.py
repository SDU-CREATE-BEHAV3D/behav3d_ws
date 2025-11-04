#!/usr/bin/env python3
"""
Reprojection validation for eye-in-hand hand–eye calibration using existing dataset.

- Carga handeye_result_final.json en formato cam->tool0 (como lo da OpenCV).
- Invierte a tool0->cam para encadenar con base->tool0 del manifest.
- Calcula reprojection RMSE por frame y global.

Usage:
  /usr/bin/python3 reprojection_check.py
"""

import os, sys, json, math, yaml
from statistics import mean
import numpy as np
import cv2

# ---------- CONFIG ----------
SQUARES_X = 12
SQUARES_Y = 9
SQUARE_LENGTH_M = 0.03   # metros
MARKER_LENGTH_M = 0.022  # metros
ARUCO_DICT_NAME = "DICT_5X5_100"

MANIFEST_PATH = "/home/lab/behav3d_ws/captures/session-20250904_170940_mancap/manifest.json"
CAMERA_INFO_YAML = "/home/lab/behav3d_ws/color_camera_info.yaml"
HANDEYE_JSON = "/home/lab/behav3d_ws/handeye_result_final.json"
OUT_REPORT = "reprojection_report.txt"
# ----------------------------

# ---------- Helpers ----------
def quat_xyzw_to_R(q):
    x,y,z,w = q
    n = math.sqrt(x*x+y*y+z*z+w*w)
    if n == 0: 
        return np.eye(3)
    x,y,z,w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]
    ], dtype=np.float64)

def R_to_rvec(R):
    rvec, _ = cv2.Rodrigues(R)
    return rvec

def invert(R, t):
    Rinv = R.T
    tinv = -Rinv @ t
    return Rinv, tinv

def compose(R_ab, t_ab, R_bc, t_bc):
    R_ac = R_ab @ R_bc
    t_ac = R_ab @ t_bc + t_ab
    return R_ac, t_ac

# Manifest loader
def load_manifest(path):
    with open(path, "r") as f:
        txt = f.read()
    caps, buf, depth = [], [], 0
    for ch in txt:
        if ch == '{':
            if depth == 0: buf = []
            depth += 1
        if depth > 0: buf.append(ch)
        if ch == '}':
            if depth > 0:
                depth -= 1
                if depth == 0:
                    try:
                        obj = json.loads("".join(buf))
                        if isinstance(obj, dict) and "index" in obj:
                            caps.append(obj)
                    except Exception:
                        pass
    if not caps:
        raise RuntimeError("No capture objects found in manifest.")
    return {"session_dir": os.path.dirname(path), "captures": caps}

def iter_caps(manifest):
    root = manifest["session_dir"]
    for cap in manifest["captures"]:
        img = cap.get("image_color")
        if img and not os.path.isabs(img):
            img = os.path.join(root, img)
        if not (img and os.path.exists(img)):
            cand = os.path.join(root, "color_raw", os.path.basename(img or ""))
            if os.path.exists(cand):
                img = cand
        if img and os.path.exists(img):
            yield cap["index"], img, cap.get("pose_tool0")

def load_cam_info(path):
    with open(path, "r") as f:
        y = yaml.safe_load(f)
    K = np.array(y.get("k") or y.get("K"), dtype=np.float64).reshape(3,3)
    D = np.array(y.get("d") or y.get("D"), dtype=np.float64).reshape(-1)
    return K, D

def make_board_and_detector():
    dict_id = getattr(cv2.aruco, ARUCO_DICT_NAME)
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    board = cv2.aruco.CharucoBoard_create(
        SQUARES_X, SQUARES_Y, SQUARE_LENGTH_M, MARKER_LENGTH_M, aruco_dict
    )
    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = getattr(cv2.aruco, "CORNER_REFINE_SUBPIX", 1)
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        return board, aruco_dict, detector, None
    else:
        params = cv2.aruco.DetectorParameters_create()
        try:
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        except Exception:
            pass
        return board, aruco_dict, None, params

def detect_charuco_pose(img, board, aruco_dict, detector, legacy_params, K, D):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detectar ArUco
    if detector is not None:
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=legacy_params)
    if ids is None or len(ids) == 0:
        return None

    # Interpolar esquinas de Charuco
    ret, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
        markerCorners=corners, markerIds=ids, image=gray, board=board
    )
    if ch_ids is None or len(ch_ids) < 6:
        return None

    # Pose del Charuco (API OpenCV 4.6 requiere rvec/tvec como entrada)
    rvec = np.zeros((3,1), np.float64)
    tvec = np.zeros((3,1), np.float64)
    ok = cv2.aruco.estimatePoseCharucoBoard(
        ch_corners, ch_ids, board, K, D, rvec, tvec
    )
    if not ok:
        return None

    R_tc, _ = cv2.Rodrigues(rvec)
    return {
        "R_tc": R_tc,
        "t_tc": tvec.reshape(3,1),
        "charuco_ids": ch_ids.copy(),
        "charuco_corners": ch_corners.copy()
    }

# ---------- MAIN ----------
def main():
    manifest = load_manifest(MANIFEST_PATH)
    K, D = load_cam_info(CAMERA_INFO_YAML)
    board, aruco_dict, detector, legacy_params = make_board_and_detector()

    # Hand-eye JSON (OpenCV da cam->tool0)
    with open(HANDEYE_JSON, "r") as f:
        he = json.load(f)
    R_cg = quat_xyzw_to_R(he["quaternion_xyzw"])
    t_cg = np.array(he["translation_m"], dtype=np.float64).reshape(3,1)

    # Frames con base->tool0 y Charuco
    frames = []
    for idx, img_path, pose in iter_caps(manifest):
        if pose is None: continue
        img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        if img is None: continue
        obs = detect_charuco_pose(img, board, aruco_dict, detector, legacy_params, K, D)
        if obs is None: continue
        R_bt = quat_xyzw_to_R(pose["orientation_xyzw"])
        t_bt = np.array(pose["position"], dtype=np.float64).reshape(3,1)
        frames.append({
            "index": idx, "img": img_path,
            "R_bt": R_bt, "t_bt": t_bt,
            "R_tc_obs": obs["R_tc"], "t_tc_obs": obs["t_tc"],
            "ch_ids": obs["charuco_ids"], "ch_corners": obs["charuco_corners"]
        })

    if len(frames) < 3:
        print("[ERR] Not enough valid frames"); sys.exit(1)

    # Flags de prueba
    TEST_SHIFT = 0      # 0 = normal, 1 = usar pose siguiente, -1 = anterior
    USE_OPTICAL = False # True = aplicar rotación ROS optical

    # Convención ROS optical (X->Z, Y->-X, Z->-Y)
    R_opt = np.array([[0,0,1],
                      [-1,0,0],
                      [0,-1,0]], dtype=np.float64)

    def rmse_for_candidate(R_gc, t_gc):
        Rbc_list, tbc_list = [], []
        for i,f in enumerate(frames):
            j = min(max(i+TEST_SHIFT,0), len(frames)-1)
            f_pose = frames[j]
            R_bc, t_bc = compose(f_pose["R_bt"], f_pose["t_bt"], R_gc, t_gc)
            if USE_OPTICAL:
                R_bc, t_bc = compose(R_bc, t_bc, R_opt, np.zeros((3,1)))
            Rbc_list.append(R_bc); tbc_list.append(t_bc)

        R_cb0, t_cb0 = invert(Rbc_list[0], tbc_list[0])
        R_tb, t_tb   = compose(frames[0]["R_tc_obs"], frames[0]["t_tc_obs"], R_cb0, t_cb0)

        rmses = []
        for (R_bc, t_bc), f in zip(zip(Rbc_list, tbc_list), frames):
            R_tc_pred, t_tc_pred = compose(R_tb, t_tb, R_bc, t_bc)
            rvec = R_to_rvec(R_tc_pred)
            ids = f["ch_ids"].flatten()
            obj_pts = board.chessboardCorners[ids, :]
            img_pred, _ = cv2.projectPoints(obj_pts, rvec, t_tc_pred, K, D)
            img_pred = img_pred.reshape(-1,2)
            img_obs  = f["ch_corners"].reshape(-1,2)
            err = np.linalg.norm(img_pred - img_obs, axis=1)
            rmses.append(float(np.sqrt(np.mean(err**2))))
        return float(np.mean(rmses)), float(min(rmses)), float(max(rmses))

    # Candidatos tool0->cam
    R_gc_A, t_gc_A = invert(R_cg, t_cg) # invertido
    R_gc_B, t_gc_B = R_cg, t_cg         # directo

    rmseA, bestA, worstA = rmse_for_candidate(R_gc_A, t_gc_A)
    rmseB, bestB, worstB = rmse_for_candidate(R_gc_B, t_gc_B)

    print(f"[CMP] A) Invert JSON: RMSE={rmseA:.3f} px (best {bestA:.3f}, worst {worstA:.3f})")
    print(f"[CMP] B) Direct JSON: RMSE={rmseB:.3f} px (best {bestB:.3f}, worst {worstB:.3f})")
    print(f"[INFO] TEST_SHIFT={TEST_SHIFT}, USE_OPTICAL={USE_OPTICAL}")


if __name__ == "__main__":
    main()
