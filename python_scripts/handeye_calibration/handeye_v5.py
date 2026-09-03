import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[1]))

import cv2
import copy
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from utils.session import Session
from utils.manifest import read_manifest, load_robot_poses_decomposed,load_robot_poses, construct_image_paths
from utils.image_loader import load_ir_image, preprocess_percentile_ir, preprocess_threshold_ir, load_color_image
from utils.transforms import rotmat_to_quat_xyzw, rotmat_to_rpy,compose_T
from utils.intrinsics import load_intrinsics, intrinsics_matrix
from utils.extrinsics import load_extrinsics

DEBUG_IR = False
DEBUG_COLOR = False
VALIDATION = True
REFINE_ALL_CAPTURES = True
ROTATION_WEIGHT_M_PER_RAD = 0.05
CLOSURE_TRANSLATION_GAIN = 2.0
CLOSURE_ROTATION_WEIGHT_M_PER_RAD = 0.05
REFINE_LOSS = "huber"
REFINE_F_SCALE_M = 0.005
REFINE_MAX_NFEV = 2000
ENABLE_OUTLIER_REJECTION = True
MIN_KEPT_CAPTURES = 12
MIN_SCORE_IMPROVEMENT_M = 1e-5
SCORE_CLOSURE_TRANSLATION_WEIGHT = 2.0
SCORE_RMS_SPREAD_WEIGHT = 1.0
SCORE_CLOSURE_ROTATION_WEIGHT_M_PER_RAD = 0.05

SESSION_PATH = "/home/lab/behav3d_ws/captures/260903_132200"

scan_folder = "test_capture"

my_session = Session(SESSION_PATH, scan_folder)

# 1) Resolve intrinsics paths
ir_width, ir_height, ir_K, ir_D = load_intrinsics(my_session.ir_intrinsics_path)
color_width, color_height, color_K, color_D = load_intrinsics(my_session.color_intrinsics_path)

# 2) Load Manifest
manifest = read_manifest(my_session.path, my_session._scan_folder)

#3 Load Captures Paths
t_base_tool0,r_base_tool0  = load_robot_poses_decomposed(manifest)
T_base_tool0_list = load_robot_poses(manifest)
ir_img_path = construct_image_paths(manifest, my_session, image_type="ir")
color_img_path = construct_image_paths(manifest, my_session, image_type="color")

# 4) Charuco Board config
# ---- Handeye methods----
method_map = {
    "Tsai": cv2.CALIB_HAND_EYE_TSAI,
    "Park": cv2.CALIB_HAND_EYE_PARK,
    "Horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "Andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}
method = "Park"

# ---- Board parameters----
SQUARES_X = 6
SQUARES_Y = 5
SQUARE_LENGTH_M = 0.055
MARKER_LENGTH_M = 0.041

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)

board = cv2.aruco.CharucoBoard.create(SQUARES_X, SQUARES_Y, SQUARE_LENGTH_M, MARKER_LENGTH_M, dictionary) 

#Draw for debug:
# LENGTH_PX = 640   # total length of the page in pixels
# MARGIN_PX = 20    # size of the margin in pixels
#board_img = cv2.aruco.CharucoBoard.draw(board, (LENGTH_PX, int(LENGTH_PX*(SQUARES_Y / SQUARES_X))), marginSize=MARGIN_PX)

def detect_charuco(img, K, D, board, dictionary, axis_len=0.1, refine_corners_kernel=None, debug=bool):
    """
    Inputs:
      img: uint8 grayscale OR color image (BGR/BGRA/RGB).
      K, D: intrinsics (3x3, dist vector)
      board: cv2.aruco_CharucoBoard
      dictionary: cv2.aruco_Dictionary

    Returns dict:
      {
        "n_markers": int,
        "n_charuco": int,
        "ok_pose": bool,
        "rvec": np.ndarray | None,
        "tvec": np.ndarray | None,
        "annot": np.ndarray (BGR) | None
      }
    """
    if img is None:
        return {"n_markers": 0, "n_charuco": 0, "ok_pose": False,
                "rvec": None, "tvec": None, "annot": None}

    # -- Ensure we have uint8 grayscale for detection, and BGR for annotation
    if img.ndim == 3:
        # Handle possible 4-channel or non-BGR inputs conservatively
        if img.shape[2] == 4:
            bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        else:
            # Assume BGR; if RGB was provided upstream, ensure conversion before calling
            bgr = img
        gray_u8 = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    else:
        # Single channel input
        gray_u8 = img
        bgr = cv2.cvtColor(gray_u8, cv2.COLOR_GRAY2BGR)

    # gray_u8 = cv2.GaussianBlur(gray_u8, (5, 5), 1)
    
    # 1) Detect ArUco markers
    params = cv2.aruco.DetectorParameters_create()
    # Subpixel corner refinement (Not working fine)
    if not refine_corners_kernel is None:
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = refine_corners_kernel          # typical: 5–9
        params.cornerRefinementMaxIterations = 100  # typical: 50–300
        params.cornerRefinementMinAccuracy = 1e-3   # typical: 1e-4–1e-2

    corners, ids, _ = cv2.aruco.detectMarkers(gray_u8, dictionary, parameters=params)

    out = {"n_markers": 0 if ids is None else len(ids),
           "n_charuco": 0, "ok_pose": False,
           "rvec": None, "tvec": None, "annot": bgr}

    if ids is None or len(ids) == 0:
        return out

    cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
    # 2) Interpolate ChArUco corners
    interp = cv2.aruco.interpolateCornersCharuco(
        markerCorners=corners, markerIds=ids, image=gray_u8,
        board=board, cameraMatrix=K, distCoeffs=D
    )
    _, ch_corners, ch_ids = interp  # version-friendly unpack

    if ch_corners is None or ch_ids is None or len(ch_ids) < 4:
        out["n_charuco"] = 0 if ch_ids is None else int(len(ch_ids))
        return out

    out["n_charuco"] = int(len(ch_ids))
    # 3) Pose estimation
    ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        charucoCorners=ch_corners,
        charucoIds=ch_ids,
        board=board,
        cameraMatrix=K,
        distCoeffs=D,
        rvec=None,
        tvec=None,
        useExtrinsicGuess=False
    )
    out["ok_pose"] = bool(ok)
    if ok:
        out["rvec"], out["tvec"] = rvec, tvec
        try:
            cv2.drawFrameAxes(bgr, K, D, rvec, tvec, axis_len, 2)
        except Exception:
            pass

    if debug:
        cv2.imshow("annot", bgr)
        cv2.waitKey(-1)
        #input("Press Enter to continue...")
        cv2.destroyWindow("annot")
    return out

    # TODO: Maybe implement? cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color); *https://docs.opencv.org/3.4/df/d4a/tutorial_charuco_detection.html*


def subset_by_indices(seq, indices):
    return [seq[i] for i in indices]


def rotmat_to_rotvec(Rm):
    return cv2.Rodrigues(Rm)[0].reshape(3)


def rotvec_to_rotmat(rotvec):
    return cv2.Rodrigues(np.asarray(rotvec, dtype=float).reshape(3, 1))[0]


def pack_pose(Rm, tvec):
    return np.hstack([rotmat_to_rotvec(Rm), np.asarray(tvec, dtype=float).reshape(3)])


def unpack_pose(params):
    Rm = rotvec_to_rotmat(params[:3])
    tvec = np.asarray(params[3:6], dtype=float).reshape(3, 1)
    return Rm, tvec


def compute_base_board_poses(T_base_tool0_keep, T_cam_board_list, r_tool0_cam, t_tool0_cam):
    T_tool0_cam = compose_T(r_tool0_cam, t_tool0_cam)
    T_base_board_all = []

    for i, T_base_tool0 in enumerate(T_base_tool0_keep):
        T_base_cam = T_base_tool0 @ T_tool0_cam
        T_base_board = T_base_cam @ T_cam_board_list[i]
        T_base_board_all.append(T_base_board)

    return T_base_board_all


def compute_board_consistency_metrics(T_base_board_all):
    positions = np.array([T[:3, 3] for T in T_base_board_all], dtype=float)
    rotations = np.array([T[:3, :3] for T in T_base_board_all], dtype=float)

    mean_pos = positions.mean(axis=0)
    std_pos = positions.std(axis=0)
    radial_errors = np.linalg.norm(positions - mean_pos, axis=1)
    rms_spread = float(np.sqrt(np.mean(radial_errors ** 2)))
    mean_spread = float(np.mean(radial_errors))
    max_spread = float(np.max(radial_errors))

    mean_rot = R.from_matrix(rotations).mean().as_matrix()
    rot_errors_deg = []
    for R_i in rotations:
        R_err = mean_rot.T @ R_i
        rot_errors_deg.append(np.degrees(np.linalg.norm(rotmat_to_rotvec(R_err))))
    rot_errors_deg = np.array(rot_errors_deg, dtype=float)

    T_board_base = np.linalg.inv(T_base_board_all[0])
    closure = T_board_base @ T_base_board_all[-1]
    closure_rot_rad = float(np.linalg.norm(rotmat_to_rotvec(closure[:3, :3])))
    closure_rot_deg = float(np.degrees(closure_rot_rad))
    closure_trans_m = float(np.linalg.norm(closure[:3, 3]))

    return {
        "mean_pos": mean_pos,
        "std_pos": std_pos,
        "mean_spread_m": mean_spread,
        "rms_spread_m": rms_spread,
        "max_spread_m": max_spread,
        "mean_rot_deg": float(np.mean(rot_errors_deg)),
        "rms_rot_deg": float(np.sqrt(np.mean(rot_errors_deg ** 2))),
        "max_rot_deg": float(np.max(rot_errors_deg)),
        "loop_closure_translation_error_m": closure_trans_m,
        "loop_closure_rotation_error_rad": closure_rot_rad,
        "loop_closure_rotation_error_deg": closure_rot_deg,
    }


def compute_solution_score(metrics):
    return (
        SCORE_CLOSURE_TRANSLATION_WEIGHT * metrics["loop_closure_translation_error_m"]
        + SCORE_RMS_SPREAD_WEIGHT * metrics["rms_spread_m"]
        + SCORE_CLOSURE_ROTATION_WEIGHT_M_PER_RAD * metrics["loop_closure_rotation_error_rad"]
    )


def print_board_consistency_metrics(label, metrics):
    print(f"\n[{label}] board consistency metrics")
    print(f"Board position mean [m]: {metrics['mean_pos']}")
    print(f"Board position std [m]:  {metrics['std_pos']}")
    print(f"Board position mean spread [mm]: {metrics['mean_spread_m'] * 1000:.3f}")
    print(f"Board position RMS spread [mm]:  {metrics['rms_spread_m'] * 1000:.3f}")
    print(f"Board position max spread [mm]:  {metrics['max_spread_m'] * 1000:.3f}")
    print(f"Board orientation mean err [deg]: {metrics['mean_rot_deg']:.4f}")
    print(f"Board orientation RMS err [deg]:  {metrics['rms_rot_deg']:.4f}")
    print(f"Board orientation max err [deg]:  {metrics['max_rot_deg']:.4f}")
    print(f"Loop closure rotation error [deg]: {metrics['loop_closure_rotation_error_deg']:.4f}")
    print(f"Loop closure translation error [mm]: {metrics['loop_closure_translation_error_m'] * 1000:.3f}")


def initial_board_pose_from_handeye(T_base_tool0_keep, T_cam_board_list, r_tool0_cam, t_tool0_cam):
    T_base_board_all = compute_base_board_poses(
        T_base_tool0_keep, T_cam_board_list, r_tool0_cam, t_tool0_cam
    )
    positions = np.array([T[:3, 3] for T in T_base_board_all], dtype=float)
    rotations = np.array([T[:3, :3] for T in T_base_board_all], dtype=float)

    mean_pos = positions.mean(axis=0).reshape(3, 1)
    mean_rot = R.from_matrix(rotations).mean().as_matrix()
    return mean_rot, mean_pos


def handeye_fixed_board_residuals(params, T_base_tool0_keep, T_cam_board_list):
    r_tool0_cam, t_tool0_cam = unpack_pose(params[:6])
    r_base_board, t_base_board = unpack_pose(params[6:12])

    T_tool0_cam = compose_T(r_tool0_cam, t_tool0_cam)
    T_base_board = compose_T(r_base_board, t_base_board)
    T_base_board_all = []

    residuals = []
    for i, T_base_tool0 in enumerate(T_base_tool0_keep):
        T_pred = T_base_tool0 @ T_tool0_cam @ T_cam_board_list[i]
        T_base_board_all.append(T_pred)

        trans_res = T_pred[:3, 3] - T_base_board[:3, 3]
        rot_res = rotmat_to_rotvec(T_base_board[:3, :3].T @ T_pred[:3, :3])

        residuals.append(trans_res)
        residuals.append(ROTATION_WEIGHT_M_PER_RAD * rot_res)

    if len(T_base_board_all) >= 2:
        closure = np.linalg.inv(T_base_board_all[0]) @ T_base_board_all[-1]
        closure_trans_res = CLOSURE_TRANSLATION_GAIN * closure[:3, 3]
        closure_rot_res = CLOSURE_ROTATION_WEIGHT_M_PER_RAD * rotmat_to_rotvec(closure[:3, :3])
        residuals.append(closure_trans_res)
        residuals.append(closure_rot_res)

    return np.concatenate(residuals)


def solve_handeye_all_captures(r_base_tool0_keep, t_base_tool0_keep, T_base_tool0_keep,
                               r_cam_board_list, t_cam_board_list, T_cam_board_list,
                               method, label="handeye", capture_ids=None, verbose=True):
    if capture_ids is None:
        capture_ids = list(range(len(T_base_tool0_keep)))

    try:
        r_tool0_cam_init, t_tool0_cam_init = cv2.calibrateHandEye(
            r_base_tool0_keep, t_base_tool0_keep,
            r_cam_board_list, t_cam_board_list,
            method=method_map[method]
        )
    except cv2.error:
        return None

    init_board_R, init_board_t = initial_board_pose_from_handeye(
        T_base_tool0_keep, T_cam_board_list, r_tool0_cam_init, t_tool0_cam_init
    )
    x0 = np.hstack([
        pack_pose(r_tool0_cam_init, t_tool0_cam_init),
        pack_pose(init_board_R, init_board_t),
    ])

    initial_board_metrics = compute_board_consistency_metrics(
        compute_base_board_poses(T_base_tool0_keep, T_cam_board_list, r_tool0_cam_init, t_tool0_cam_init)
    )
    initial_residuals = handeye_fixed_board_residuals(x0, T_base_tool0_keep, T_cam_board_list)
    initial_plain_cost = 0.5 * float(np.dot(initial_residuals, initial_residuals))

    if REFINE_ALL_CAPTURES:
        result = least_squares(
            handeye_fixed_board_residuals,
            x0,
            args=(T_base_tool0_keep, T_cam_board_list),
            method="trf",
            loss=REFINE_LOSS,
            f_scale=REFINE_F_SCALE_M,
            max_nfev=REFINE_MAX_NFEV,
        )
        r_tool0_cam_refined, t_tool0_cam_refined = unpack_pose(result.x[:6])
        r_base_board_refined, t_base_board_refined = unpack_pose(result.x[6:12])
        final_board_metrics = compute_board_consistency_metrics(
            compute_base_board_poses(T_base_tool0_keep, T_cam_board_list, r_tool0_cam_refined, t_tool0_cam_refined)
        )
        final_residuals = handeye_fixed_board_residuals(result.x, T_base_tool0_keep, T_cam_board_list)
        final_plain_cost = 0.5 * float(np.dot(final_residuals, final_residuals))
        final_score = compute_solution_score(final_board_metrics)

        if verbose:
            print(f"\n[{label}] global refinement status={result.status} success={result.success} nfev={result.nfev}")
            print(f"[{label}] plain cost initial={initial_plain_cost:.6e} final={final_plain_cost:.6e}")
            print(f"[{label}] optimizer reported cost={result.cost:.6e} loss={REFINE_LOSS}")

        return {
            "r_tool0_cam": r_tool0_cam_refined,
            "t_tool0_cam": t_tool0_cam_refined,
            "r_base_board": r_base_board_refined,
            "t_base_board": t_base_board_refined,
            "initial_board_metrics": initial_board_metrics,
            "final_board_metrics": final_board_metrics,
            "optimizer_result": result,
            "score": final_score,
            "capture_ids": list(capture_ids),
        }

    final_score = compute_solution_score(initial_board_metrics)
    return {
        "r_tool0_cam": r_tool0_cam_init,
        "t_tool0_cam": t_tool0_cam_init,
        "r_base_board": init_board_R,
        "t_base_board": init_board_t,
        "initial_board_metrics": initial_board_metrics,
        "final_board_metrics": initial_board_metrics,
        "optimizer_result": None,
        "score": final_score,
        "capture_ids": list(capture_ids),
    }


def solve_handeye_with_outlier_rejection(r_base_tool0_keep, t_base_tool0_keep, T_base_tool0_keep,
                                         r_cam_board_list, t_cam_board_list, T_cam_board_list,
                                         method, capture_ids, label="handeye"):
    current_local_indices = tuple(range(len(T_base_tool0_keep)))
    dropped_capture_ids = []

    best_solution = solve_handeye_all_captures(
        r_base_tool0_keep, t_base_tool0_keep, T_base_tool0_keep,
        r_cam_board_list, t_cam_board_list, T_cam_board_list,
        method,
        label=label,
        capture_ids=capture_ids,
        verbose=True,
    )
    if best_solution is None:
        raise RuntimeError(f"{label}: initial hand-eye solve failed")

    current_score = best_solution["score"]
    print(
        f"[{label}] initial score={current_score * 1000:.3f} "
        f"(kept={len(current_local_indices)} captures, ids={best_solution['capture_ids']})"
    )

    if not ENABLE_OUTLIER_REJECTION:
        best_solution["dropped_capture_ids"] = dropped_capture_ids
        return best_solution

    improved = True
    while improved and len(current_local_indices) > MIN_KEPT_CAPTURES:
        improved = False
        best_candidate_solution = None
        best_candidate_indices = None
        best_candidate_score = current_score
        dropped_capture_id = None

        for drop_pos, local_idx_to_drop in enumerate(current_local_indices):
            candidate_indices = current_local_indices[:drop_pos] + current_local_indices[drop_pos + 1:]

            candidate_solution = solve_handeye_all_captures(
                subset_by_indices(r_base_tool0_keep, candidate_indices),
                subset_by_indices(t_base_tool0_keep, candidate_indices),
                subset_by_indices(T_base_tool0_keep, candidate_indices),
                subset_by_indices(r_cam_board_list, candidate_indices),
                subset_by_indices(t_cam_board_list, candidate_indices),
                subset_by_indices(T_cam_board_list, candidate_indices),
                method,
                label=label,
                capture_ids=subset_by_indices(capture_ids, candidate_indices),
                verbose=False,
            )
            if candidate_solution is None:
                continue

            candidate_score = candidate_solution["score"]
            if candidate_score < (best_candidate_score - MIN_SCORE_IMPROVEMENT_M):
                best_candidate_solution = candidate_solution
                best_candidate_indices = candidate_indices
                best_candidate_score = candidate_score
                dropped_capture_id = capture_ids[local_idx_to_drop]
                improved = True

        if improved:
            current_local_indices = best_candidate_indices
            best_solution = best_candidate_solution
            current_score = best_candidate_score
            dropped_capture_ids.append(dropped_capture_id)
            metrics = best_solution["final_board_metrics"]
            print(
                f"[{label}] dropped capture {dropped_capture_id}; "
                f"kept={len(current_local_indices)} score={current_score * 1000:.3f} "
                f"closure_mm={metrics['loop_closure_translation_error_m'] * 1000:.3f} "
                f"rms_spread_mm={metrics['rms_spread_m'] * 1000:.3f}"
            )

    best_solution["dropped_capture_ids"] = dropped_capture_ids
    return best_solution

def main():

#IR Handeye main ingredients
    r_cam_board_list_ir = []
    t_cam_board_list_ir = []
    T_cam_board_list_ir = []

    r_base_tool0_keep_ir = []
    t_base_tool0_keep_ir = []
    T_base_tool0_keep_ir = []
    capture_ids_keep_ir = []

    
#Color Handeye main ingredients
    r_cam_board_list_color = []
    t_cam_board_list_color = []
    T_cam_board_list_color = []

    r_base_tool0_keep_color = []
    t_base_tool0_keep_color = []
    T_base_tool0_keep_color = []
    capture_ids_keep_color = []

    # IR Execute detection and Collect ir pairs 

    skip_idx = {0,16,19,22,23}  # Index to skip

    for i, p in enumerate(ir_img_path):
        if i in skip_idx:
            continue

        gray = load_ir_image(p)
        pre = preprocess_threshold_ir(gray, 2500)
        charuco_res = detect_charuco(pre, ir_K, ir_D, board, dictionary, axis_len=0.05, debug=DEBUG_IR)

        if charuco_res["ok_pose"] and charuco_res["rvec"] is not None and charuco_res["tvec"] is not None:
            r_cam_board, _ = cv2.Rodrigues(charuco_res["rvec"])
            t_cam_board = charuco_res["tvec"].reshape(3,1)

            r_cam_board_list_ir.append(r_cam_board)
            t_cam_board_list_ir.append(t_cam_board)
            T_cam_board = compose_T(r_cam_board, t_cam_board)
            T_cam_board_list_ir.append(T_cam_board)
            r_base_tool0_keep_ir.append(r_base_tool0[i])
            t_base_tool0_keep_ir.append(t_base_tool0[i].reshape(3,1))
            T_base_tool0_keep_ir.append(T_base_tool0_list[i])
            capture_ids_keep_ir.append(i)

# Color charuco detection and Collect color pose pairs 

    for i, p in enumerate(color_img_path):
        if i in skip_idx:
            continue

        img = load_color_image(p)
        charuco_res = detect_charuco(img, color_K, color_D, board, dictionary, axis_len=0.05, refine_corners_kernel=5, debug=DEBUG_COLOR)
        if charuco_res["ok_pose"] and charuco_res["rvec"] is not None and charuco_res["tvec"] is not None:
            r_cam_board, _ = cv2.Rodrigues(charuco_res["rvec"])
            t_cam_board = charuco_res["tvec"].reshape(3,1)

            r_cam_board_list_color .append(r_cam_board)
            t_cam_board_list_color .append(t_cam_board)
            T_cam_board = compose_T(r_cam_board,t_cam_board)
            T_cam_board_list_color.append(T_cam_board)
            r_base_tool0_keep_color .append(r_base_tool0[i])
            t_base_tool0_keep_color .append(t_base_tool0[i].reshape(3,1))
            T_base_tool0_keep_color .append(T_base_tool0_list[i])
            capture_ids_keep_color.append(i)
           
#Handeye solveR

    ir_solution = solve_handeye_with_outlier_rejection(
        r_base_tool0_keep_ir, t_base_tool0_keep_ir, T_base_tool0_keep_ir,
        r_cam_board_list_ir, t_cam_board_list_ir, T_cam_board_list_ir,
        method,
        capture_ids_keep_ir,
        label="IR",
    )
    r_tool0_cam_ir = ir_solution["r_tool0_cam"]
    t_tool0_cam_ir = ir_solution["t_tool0_cam"]

    color_solution = solve_handeye_with_outlier_rejection(
        r_base_tool0_keep_color, t_base_tool0_keep_color, T_base_tool0_keep_color,
        r_cam_board_list_color, t_cam_board_list_color, T_cam_board_list_color,
        method,
        capture_ids_keep_color,
        label="COLOR",
    )
    r_tool0_cam_color = color_solution["r_tool0_cam"]
    t_tool0_cam_color = color_solution["t_tool0_cam"]
#Format to quat and rpy
    ir_quat= rotmat_to_quat_xyzw(r_tool0_cam_ir)
    ir_rpy =rotmat_to_rpy(r_tool0_cam_ir)
    ir_rpy_deg =rotmat_to_rpy(r_tool0_cam_ir,degrees=True)
    color_quat= rotmat_to_quat_xyzw(r_tool0_cam_color)
    color_rpy =rotmat_to_rpy(r_tool0_cam_color)
    color_rpy_deg =rotmat_to_rpy(r_tool0_cam_color,degrees=True)
    print ("Starting hand-eye calibration process...")

    print(f"Loaded {len(t_base_tool0)} robot poses from manifest.")

    print ("\n------IR calibration results------\n")
    print(f"Detected {len(t_base_tool0_keep_ir)} IR robot/camera pose pairs before rejection.")
    print(f"Using {len(ir_solution['capture_ids'])} IR pose pairs after rejection.")
    print(f"IR kept capture ids: {ir_solution['capture_ids']}")
    print(f"IR dropped capture ids: {ir_solution['dropped_capture_ids']}")
    print(f"IR rotation matrix:\n{r_tool0_cam_ir}")
    print("IR quaternion:\n", *ir_quat)
    print("IR RPY Rad:\n", *ir_rpy)
    print("IR RPY Deg:\n", *ir_rpy_deg)
    print("IR translation XYZ:\n", *t_tool0_cam_ir.flatten())
    
    print ("\n------Color calibration results------\n")
    print(f"Detected {len(t_base_tool0_keep_color)} color robot/camera pose pairs before rejection.")
    print(f"Using {len(color_solution['capture_ids'])} color pose pairs after rejection.")
    print(f"Color kept capture ids: {color_solution['capture_ids']}")
    print(f"Color dropped capture ids: {color_solution['dropped_capture_ids']}")
    print(f"Color rotation matrix:\n{r_tool0_cam_color}")
    print("Color quaternion:\n", *color_quat)
    print("Color RPY Rad:\n", *color_rpy)
    print("Color RPY Deg:\n", *color_rpy_deg)
    print("Color translation XYZ:\n", *t_tool0_cam_color.flatten())
  #  print (color_img_path)

    # ---------- Validation loop ----------
    if VALIDATION == True:
        print("\n=== Validation results ===")
        print_board_consistency_metrics("IR initial", ir_solution["initial_board_metrics"])
        print_board_consistency_metrics("IR final", ir_solution["final_board_metrics"])
        print_board_consistency_metrics("COLOR initial", color_solution["initial_board_metrics"])
        print_board_consistency_metrics("COLOR final", color_solution["final_board_metrics"])

    print(cv2.__version__)

    return 0


if __name__ == "__main__":
    sys.exit(main())
