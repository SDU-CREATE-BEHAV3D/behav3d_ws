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
IMG_NAME  = "ir_t4.png"

IMG_PATH  = os.path.join(SESS_PATH, "ir_raw", IMG_NAME)
INTR_PATH = os.path.join(SESS_PATH, "calib", "ir_intrinsics.yaml")

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
# --- IR utilities ---
def _load_ir_image(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """
    Load IR image preserving depth; return (bgr_for_viz, gray_u8).
    Supports 8UC1/8UC3/16UC1. Normalizes 16-bit to 8-bit with robust percentiles.
    """
    img_raw = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img_raw is None:
        return None, None

    if img_raw.dtype == np.uint16:  # 16-bit IR
        ir16 = img_raw
        p1, p99 = np.percentile(ir16, (1, 99))
        if p99 <= p1:
            p1, p99 = ir16.min(), ir16.max()
        ir16 = np.clip(ir16, p1, p99)
        gray = ((ir16 - p1) * (255.0 / (p99 - p1 + 1e-9))).astype(np.uint8)
        bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    elif len(img_raw.shape) == 2:  # 8-bit grayscale
        gray = img_raw.astype(np.uint8, copy=False)
        bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    else:  # 8-bit BGR
        bgr = img_raw
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return bgr, gray

def _preprocess_ir(gray_u8: np.ndarray, *, invert: bool=False, use_clahe: bool=True) -> np.ndarray:
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

def _make_detector_params() -> cv2.aruco.DetectorParameters:
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
        cand = os.path.join(base, "calib", "ir_intrinsics.yaml")
        return cand
    except Exception:
        return INTR_PATH
# --- IR utilities ---
def _load_ir_image(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """
    Load IR image preserving depth; return (bgr_for_viz, gray_u8).
    Supports 8UC1/8UC3/16UC1. Normalizes 16-bit to 8-bit with robust percentiles.
    """
    img_raw = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img_raw is None:
        return None, None

    if img_raw.dtype == np.uint16:  # 16-bit IR
        ir16 = img_raw
        p1, p99 = np.percentile(ir16, (1, 99))
        if p99 <= p1:
            p1, p99 = ir16.min(), ir16.max()
        ir16 = np.clip(ir16, p1, p99)
        gray = ((ir16 - p1) * (255.0 / (p99 - p1 + 1e-9))).astype(np.uint8)
        bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    elif len(img_raw.shape) == 2:  # 8-bit grayscale
        gray = img_raw.astype(np.uint8, copy=False)
        bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    else:  # 8-bit BGR
        bgr = img_raw
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return bgr, gray

def _preprocess_ir(gray_u8: np.ndarray, *, invert: bool=False, use_clahe: bool=True) -> np.ndarray:
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

def _make_detector_params() -> cv2.aruco.DetectorParameters:
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

def main():
    parser = argparse.ArgumentParser(description="ChArUco board detector & pose viewer (RGB/IR)")
    parser.add_argument("--image", "-i", default=IMG_PATH)
    parser.add_argument("--intr", "-k", default=INTR_PATH)
    parser.add_argument("--out", "-o", default=OUT_PATH)
    parser.add_argument("--undistort", action="store_true")
    parser.add_argument("--axis", type=float, default=0.1)
    parser.add_argument("--invert", action="store_true", help="Invert IR intensities if board appears low-contrast")
    parser.add_argument("--no-clahe", action="store_true", help="Disable CLAHE preprocessing")
    args = parser.parse_args()

    bgr, gray0 = _load_ir_image(args.image)
    if bgr is None:
        print(f"[error] Could not load image: {args.image}", file=sys.stderr)
        return 2

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
        bgr = cv2.undistort(bgr, K, D)
        gray0 = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    gray = _preprocess_ir(gray0, invert=args.invert, use_clahe=not args.no_clahe)

    dictionary = _get_dict(ARUCO_DICT_NAME)
    board = _make_board(dictionary)
    params = _make_detector_params()

    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

    annotated = bgr.copy()
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
