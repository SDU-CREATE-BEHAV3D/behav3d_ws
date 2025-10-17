#This code meant for debugging loads a charuco board image, detects the markers, interpolates the corners, estimates the pose of the board, and visualizes the results.

#!/usr/bin/env python3
# charuco_viewer.py (OpenCV 4.6 robust – chessboardCorners fix)
import argparse, os, sys, math
from typing import Tuple
import numpy as np
import cv2

# ---------- Detector config ----------
SQUARES_X = 6
SQUARES_Y = 5
SQUARE_LENGTH_M = 0.055
MARKER_LENGTH_M = 0.041
ARUCO_DICT_NAME = "DICT_5X5_100"

# ---------- Quick-use paths ----------
SESS_PATH = "/home/lab/behav3d_ws/captures/session-20251009_153835_mancap/"
IMG_NAME  = "color_t1.png"

IMG_PATH  = os.path.join(SESS_PATH, "color_raw", IMG_NAME)
INTR_PATH = os.path.join(SESS_PATH, "calib", "color_intrinsics.yaml")

OUT_PATH  = os.path.join(SESS_PATH, "calib",
                         f"annotated_{os.path.splitext(IMG_NAME)[0]}.png")


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

def _get_dict(name: str):
    return cv2.aruco.getPredefinedDictionary(_ARUCO_DICTS[name])

def _load_intrinsics(fs_path: str) -> Tuple[np.ndarray, np.ndarray]:
    fs = cv2.FileStorage(fs_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise IOError(f"Failed to open intrinsics YAML: {fs_path}")
    try:
        K = fs.getNode("camera_matrix").mat()
        D = fs.getNode("distortion_coefficients").mat()
        if K is None or D is None:
            raise ValueError("Missing camera_matrix or distortion_coefficients in YAML.")
        K = np.asarray(K, dtype=np.float64).reshape(3,3)
        D = np.asarray(D, dtype=np.float64).ravel()
        return K, D
    finally:
        fs.release()

def _make_board(dictionary):
    return cv2.aruco.CharucoBoard_create(
        squaresX=SQUARES_X,
        squaresY=SQUARES_Y,
        squareLength=SQUARE_LENGTH_M,
        markerLength=MARKER_LENGTH_M,
        dictionary=dictionary
    )

def _detect_markers(gray, dictionary):
    params = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    return corners, ids

def _interpolate_charuco(gray, corners, ids, board, K, D):
    if ids is None or len(ids) == 0:
        return None, None
    out = cv2.aruco.interpolateCornersCharuco(
        markerCorners=corners, markerIds=ids, image=gray, board=board, cameraMatrix=K, distCoeffs=D
    )
    # OpenCV 4.6 returns (retval, charucoCorners, charucoIds)
    if isinstance(out, tuple) and len(out) >= 3:
        _, ch_corners, ch_ids = out
    else:
        ch_corners, ch_ids = out
    if ch_corners is not None:
        ch_corners = np.asarray(ch_corners, dtype=np.float32)
    if ch_ids is not None:
        ch_ids = np.asarray(ch_ids, dtype=np.int32)
    return ch_corners, ch_ids

def _board_charuco_objpoints(board, charuco_ids):
    """Get 3D object points for given ChArUco corner IDs (OpenCV 4.6: board.chessboardCorners)."""
    # chessboardCorners is Nx3 or Nx1x3 depending on build; normalize to (N,3)
    corners3d = np.array(board.chessboardCorners, dtype=np.float32).reshape(-1, 3)
    sel = charuco_ids.ravel().astype(int)
    return corners3d[sel]

def estimate_pose_charuco(charuco_corners, charuco_ids, board, K, D):
    # Require min 4 corners
    if charuco_corners is None or charuco_ids is None or len(charuco_ids) < 4:
        return None, None
    # Try OpenCV helper first (if signature fits)
    try:
        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charucoCorners=charuco_corners, charucoIds=charuco_ids, board=board, cameraMatrix=K, distCoeffs=D
        )
        if ok:
            return rvec, tvec
    except Exception:
        pass
    # Fallback: solvePnP against board 3D points
    obj_pts = _board_charuco_objpoints(board, charuco_ids)
    img_pts = charuco_corners.reshape(-1, 2)
    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    return (rvec, tvec) if ok else (None, None)

def infer_intr_from_image(img_path: str) -> str:
    try:
        base = os.path.dirname(os.path.dirname(img_path))
        cand = os.path.join(base, "calib", "color_intrinsics.yaml")
        return cand
    except Exception:
        return INTR_PATH

def main():
    parser = argparse.ArgumentParser(description="ChArUco board detector & pose viewer")
    parser.add_argument("--image", "-i", default=IMG_PATH, help="Path to input image")
    parser.add_argument("--intr", "-k", default=INTR_PATH, help="Path to intrinsics YAML (OpenCV format)")
    parser.add_argument("--out", "-o", default=OUT_PATH, help="Path to save annotated image")
    parser.add_argument("--undistort", action="store_true", help="Undistort preview before detection (optional)")
    parser.add_argument("--axis", type=float, default=0.1, help="Axis length in meters for drawFrameAxes")
    args = parser.parse_args()

    img = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if img is None:
        print(f"[error] Could not load image: {args.image}", file=sys.stderr)
        return 2
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    intr_path = args.intr
    if not os.path.isfile(intr_path):
        alt = infer_intr_from_image(args.image)
        if os.path.isfile(alt):
            intr_path = alt
            print(f"[info] Using inferred intrinsics: {intr_path}")
        else:
            print(f"[error] Intrinsics not found: {args.intr}", file=sys.stderr)
            return 3

    try:
        K, D = _load_intrinsics(intr_path)
    except Exception as e:
        print(f"[error] Failed to load intrinsics: {e}", file=sys.stderr)
        return 3

    if args.undistort:
        img = cv2.undistort(img, K, D)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    dictionary = _get_dict(ARUCO_DICT_NAME)
    board = _make_board(dictionary)

    corners, ids = _detect_markers(gray, dictionary)
    annotated = img.copy()
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

    charuco_corners, charuco_ids = _interpolate_charuco(gray, corners, ids, board, K, D)

    if (charuco_corners is not None and charuco_ids is not None 
        and len(charuco_corners) > 0 and len(charuco_corners) == len(charuco_ids)):
        try:
            cv2.aruco.drawDetectedCornersCharuco(annotated, charuco_corners, charuco_ids, (0, 255, 0))
        except Exception:
            for p in charuco_corners.reshape(-1,2):
                cv2.circle(annotated, (int(p[0]), int(p[1])), 3, (0,255,0), -1)

    rvec, tvec = estimate_pose_charuco(charuco_corners, charuco_ids, board, K, D)
    if rvec is not None and tvec is not None:
        try:
            cv2.drawFrameAxes(annotated, K, D, rvec, tvec, args.axis)
        except Exception as e:
            print(f"[warn] drawFrameAxes failed: {e}", file=sys.stderr)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    cv2.imwrite(args.out, annotated)

    n_markers = 0 if ids is None else len(ids)
    n_char = 0 if charuco_ids is None else len(charuco_ids)
    print(f"[ok] Saved: {args.out}")
    print(f"Markers: {n_markers}, ChArUco corners: {n_char}")
    if rvec is not None and tvec is not None:
        R, _ = cv2.Rodrigues(rvec)
        print("Rotation matrix:\n", R)

        yaw = math.degrees(math.atan2(R[1,0], R[0,0]))
        pitch = math.degrees(math.atan2(-R[2,0], math.hypot(R[2,1], R[2,2])))
        roll = math.degrees(math.atan2(R[2,1], R[2,2]))
        print("Pose (cam <- board):")
        print(f"  tvec [m]: {tvec.ravel()}")
        print(f"  rvec [rad]: {rvec.ravel()}")
        print(f"  Euler YPR [deg]: [{yaw:.3f}, {pitch:.3f}, {roll:.3f}]")
    else:
        print("[info] Pose not estimated (insufficient corners).")
    return 0

if __name__ == "__main__":
    sys.exit(main())
