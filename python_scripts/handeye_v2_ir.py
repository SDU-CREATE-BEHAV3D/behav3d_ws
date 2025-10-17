#!/usr/bin/env python3
# handeye_quick_ir.py
# Eye-in-hand hand–eye calibration with ChArUco for RGB or IR streams.
#
# Assumed folder structure:
#   SESS_PATH/
#     ├─ manifest.json                  <-- robot poses + per-capture info
#     ├─ calib/
#     │    ├─ color_intrinsics.yaml    <-- OpenCV intrinsics (RGB)
#     │    └─ ir_intrinsics.yaml       <-- OpenCV intrinsics (IR)
#     ├─ color_raw/ ... (optional)
#     └─ ir_raw/    ... (optional)
#
# Output:
#   SESS_PATH/calib/handeye_result.json
#   SESS_PATH/calib/tool0_from_cam.yaml
#   (optional) annotated previews alongside source images

import os
import sys
import json
import math
import argparse
import numpy as np
import cv2

# ---------- Session / options ----------
SESS_PATH = "/home/lab/behav3d_ws/captures/session-20251009_181643_mancap/"
PREVIEW = True
METHOD = "Park"  # Tsai | Park | Horaud | Andreff | Daniilidis
STREAM_DEFAULT = "ir"  # "ir" or "color"

# ---------- Board config ----------
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
    if invert:
        g = cv2.bitwise_not(g)
    g = cv2.medianBlur(g, 3)
    if use_clahe:
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        g = clahe.apply(g)
    blur = cv2.GaussianBlur(g, (0,0), 1.0)
    g = cv2.addWeighted(g, 1.5, blur, -0.5, 0)
    return g

def make_detector_params():
    p = cv2.aruco.DetectorParameters_create()
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 33
    p.adaptiveThreshWinSizeStep = 2
    p.adaptiveThreshConstant = 7
    p.minMarkerPerimeterRate = 0.02
    p.maxMarkerPerimeterRate = 0.40
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 5
    p.cornerRefinementMaxIterations = 50
    p.cornerRefinementMinAccuracy = 0.01
    p.minOtsuStdDev = 5.0
    p.perspectiveRemovePixelPerCell = 8
    p.polygonalApproxAccuracyRate = 0.03
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
    if isinstance(out, tuple) and len(out) >= 3:
        _, ch_corners, ch_ids = out
    else:
        ch_corners, ch_ids = out

    if ch_corners is None or ch_ids is None or len(ch_ids) < 4:
        retval["n_charuco"] = 0 if ch_ids is None else int(len(ch_ids))
        # draw markers only
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

# ---------- Helpers for manifest/image/intrinsics ----------
def manifest_image_key(stream: str):
    # Prefer explicit keys if present; fallback to legacy names.
    return "image_ir" if stream == "ir" else "image_color"

def default_intrinsics_path(sess_path: str, stream: str):
    name = "ir_intrinsics.yaml" if stream == "ir" else "color_intrinsics.yaml"
    return os.path.join(sess_path, "calib", name)

def main():
    parser = argparse.ArgumentParser(description="Hand–eye calibration (RGB/IR) with ChArUco.")
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
    manifest_path = os.path.join(sess, "manifest.json")
    intr_path = args.intr or default_intrinsics_path(sess, args.stream)
    out_json = os.path.join(sess, "calib", "handeye_result.json")

    if not os.path.isfile(manifest_path):
        print(f"[error] manifest not found: {manifest_path}")
        return 2
    if not os.path.isfile(intr_path):
        print(f"[error] intrinsics not found: {intr_path}")
        return 3

    with open(manifest_path, "r", encoding="utf-8") as f:
        manifest = json.load(f)
    captures = manifest.get("captures", [])
    if len(captures) < 2:
        print("[error] Need at least 2 captures for hand–eye")
        return 4

    K, D = load_intrinsics(intr_path)
    dictionary = get_dict(ARUCO_DICT_NAME)
    board = make_board(dictionary)

    R_gripper2base, t_gripper2base = [], []
    R_target2cam, t_target2cam = [], []

    used = 0
    img_key = manifest_image_key(args.stream)

    for cap in captures:
        # Camera image path (prefer explicit key; fallback to inferred)
        if img_key in cap:
            img_path = cap[img_key]
        else:
            # Fallback: try to infer from the other stream path if present
            alt_key = "image_color" if args.stream == "ir" else "image_ir"
            if alt_key in cap:
                base = cap[alt_key]
                if args.stream == "ir":
                    img_path = base.replace("color_raw", "ir_raw").replace("color_", "ir_")
                else:
                    img_path = base.replace("ir_raw", "color_raw").replace("ir_", "color_")
            else:
                print(f"[warn] capture missing image path for stream '{args.stream}', skipping.")
                continue

        if not os.path.isabs(img_path):
            img_path = os.path.join(sess, img_path)

        pose = cap["pose_tool0"]
        qx, qy, qz, qw = pose["orientation_xyzw"]
        px, py, pz = pose["position"]
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
            # Optionally still save annotated to review failed frames
            if args.preview and img_annot is not None:
                out_img = os.path.join(os.path.dirname(img_path), f"annot_{os.path.basename(img_path)}")
                cv2.imwrite(out_img, img_annot)
            continue

        R_c_brd, t_c_brd = rvec_tvec_to_RT(det["rvec"], det["tvec"])

        R_gripper2base.append(R_b_t)
        t_gripper2base.append(t_b_t)
        R_target2cam.append(R_c_brd)
        t_target2cam.append(t_c_brd)
        used += 1

        if args.preview and img_annot is not None:
            out_img = os.path.join(os.path.dirname(img_path), f"annot_{os.path.basename(img_path)}")
            cv2.imwrite(out_img, img_annot)

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
        print(
            "[error] Your OpenCV build lacks calibrateHandEye. "
            "Install opencv-contrib or a newer OpenCV build."
        )
        return 6

    # tool0 <- cam
    R_t_c = R_cam2gripper
    t_t_c = t_cam2gripper

    yaw, pitch, roll = euler_ypr(R_t_c)
    quat_xyzw = R_to_quat_xyzw(R_t_c)

    print("[ok] Hand–eye (tool0 <- cam):")
    print("  t [m]:", t_t_c.ravel())
    print(f"  Euler YPR [deg]: [{yaw:.3f}, {pitch:.3f}, {roll:.3f}]")
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
        angle = math.degrees(
            math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2)))
        )
        dt = np.linalg.norm(dT[:3, 3])
        rot_errs.append(angle)
        trans_errs.append(dt)

    if rot_errs:
        med_ang = float(np.median(rot_errs))
        med_mm = float(np.median(trans_errs) * 1000)
        print(
            f"[residuals] median rotation: {med_ang:.3f} deg, "
            f"median translation: {med_mm:.2f} mm"
        )

    # Save outputs
    os.makedirs(os.path.join(sess, "calib"), exist_ok=True)
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

    return 0

if __name__ == "__main__":
    sys.exit(main())
