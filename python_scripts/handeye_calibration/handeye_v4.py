import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[1]))

import cv2
import copy
import numpy as np
from utils.session import Session
from utils.manifest import read_manifest, load_robot_poses_decomposed,load_robot_poses, construct_image_paths
from utils.image_loader import load_ir_image, preprocess_percentile_ir, preprocess_threshold_ir, load_color_image
from utils.transforms import rotmat_to_quat_xyzw, rotmat_to_rpy,compose_T
from utils.intrinsics import load_intrinsics, intrinsics_matrix
from utils.extrinsics import load_extrinsics

DEBUG_IR = True
DEBUG_COLOR = False
VALIDATION = True

SESSION_PATH = "/home/lab/behav3d_ws/captures/251113_151155"

scan_folder = "scan_fib_simple"

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

def main():

#IR Handeye main ingredients
    r_cam_board_list_ir = []
    t_cam_board_list_ir = []
    T_cam_board_list_ir = []

    r_base_tool0_keep_ir = []
    t_base_tool0_keep_ir = []
    T_base_tool0_keep_ir = []

    
#Color Handeye main ingredients
    r_cam_board_list_color = []
    t_cam_board_list_color = []
    T_cam_board_list_color = []

    r_base_tool0_keep_color = []
    t_base_tool0_keep_color = []
    T_base_tool0_keep_color = []

    # IR Execute detection and Collect ir pairs 

    skip_idx = {10,11,12,13,14,15,17}  # Index to skip

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

# Color charuco detection and Collect color pose pairs 

    for i, p in enumerate(color_img_path):
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
           
#Handeye solveR

    r_tool0_cam_ir, t_tool0_cam_ir = cv2.calibrateHandEye(
        r_base_tool0_keep_ir , t_base_tool0_keep_ir ,
        r_cam_board_list_ir ,   t_cam_board_list_ir ,
        method=method_map[method]
    )

    r_tool0_cam_color, t_tool0_cam_color = cv2.calibrateHandEye(
        r_base_tool0_keep_color , t_base_tool0_keep_color ,
        r_cam_board_list_color ,   t_cam_board_list_color ,
        method=method_map[method]
    )
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
    print(f"Using {len(t_base_tool0_keep_ir)} robot poses after IR detection.")
    print(f"Using {len(t_cam_board_list_ir )} IR camera poses from Charuco.")
    print(f"IR rotation matrix:\n{r_tool0_cam_ir}")
    print("IR quaternion:\n", *ir_quat)
    print("IR RPY Rad:\n", *ir_rpy)
    print("IR RPY Deg:\n", *ir_rpy_deg)
    print("IR translation XYZ:\n", *t_tool0_cam_ir.flatten())
    
    print ("\n------Color calibration results------\n")
    print(f"Using {len(t_base_tool0_keep_color)} robot poses after color detection.")
    print(f"Using {len(t_cam_board_list_color)} color camera poses from Charuco.")
    print(f"Color rotation matrix:\n{r_tool0_cam_color}")
    print("Color quaternion:\n", *color_quat)
    print("Color RPY Rad:\n", *color_rpy)
    print("Color RPY Deg:\n", *color_rpy_deg)
    print("Color translation XYZ:\n", *t_tool0_cam_color.flatten())
  #  print (color_img_path)

    # ---------- Validation loop ----------
    if VALIDATION == True:
        T_tool0_cam_ir = compose_T(r_tool0_cam_ir,t_tool0_cam_ir)
        T_tool0_cam_color = compose_T(r_tool0_cam_color,t_tool0_cam_color)
        
        T_base_board_all = []
        
        for i, T_base_tool0 in enumerate(T_base_tool0_keep_ir):

            # Full chain
            T_base_cam = T_base_tool0 @ T_tool0_cam_ir
            T_base_board = T_base_cam @ T_cam_board_list_ir[i]
            T_base_board_all.append(T_base_board)

        # ---------- Check consistency ----------
        positions = np.array([T[:3, 3] for T in T_base_board_all])
        mean_pos = positions.mean(axis=0)
        std_pos = positions.std(axis=0)

        print("=== Validation results ===")
        print(f"Board position mean [m]: {mean_pos}")
        print(f"Board position std [m]:  {std_pos}")
        print(f"Average distance spread: {np.linalg.norm(std_pos):.5f} m")

        # Optional loop closure check (board as world)
        T_board_base = np.linalg.inv(T_base_board_all[0])
        closure = T_board_base @ T_base_board_all[-1]
        rot_err = np.degrees(np.arccos(np.clip((np.trace(closure[:3,:3]) - 1) / 2, -1, 1)))
        trans_err = np.linalg.norm(closure[:3,3])
        print(f"Loop closure rotation error: {rot_err:.3f} deg")
        print(f"Loop closure translation error: {trans_err*1000:.3f} mm")

    print(cv2.__version__)

    return 0


if __name__ == "__main__":
    sys.exit(main())

