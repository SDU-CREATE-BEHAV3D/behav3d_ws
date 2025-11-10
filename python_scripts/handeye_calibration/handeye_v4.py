import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[1]))

import cv2

from utils.session import Session
from utils.manifest import read_manifest, load_robot_poses, transform_robot_to_camera_pose
from utils.intrinsics import load_intrinsics, intrinsics_matrix
from utils.extrinsics import load_extrinsics

SESSION_PATH = "/home/lab/behav3d_ws/captures/251104_140521/"
scan_folder = "manual_caps"

my_session = Session(SESSION_PATH, scan_folder)

# 1) Resolve intrinsics paths
ir_width, ir_height, ir_K, ir_D = load_intrinsics(my_session.ir_intrinsics_path)

def main():

    print ("Starting hand-eye calibration process...")
    print (ir_width)
    # 2) Load captures
  
    # 3) Load intrinsics

    # 4) Build dictionary and board
  
    # 5) (Optional) interactive tuning → params
 

    # 6) Collect pairs
  
    # 7) Solve hand–eye

    # 8) Residuals AX ≈ XB
  
    # residuals = compute_residuals_AX_XB(Ts_b_t, Ts_c_b, T_t_c)

    # 9) Persist outputs (J)

    return 0


if __name__ == "__main__":
    sys.exit(main())

