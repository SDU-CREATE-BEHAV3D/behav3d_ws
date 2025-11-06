# set src path for utils import
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[1]))

import utils
import cv2
SESSION_PATH = "/home/lab/behav3d_ws/captures/251105_132939"

# ### Load intirensics test
# depth_width, depth_height, depth_k, depth_d = utils.load_intrinsics(SESSION_PATH, type="depth")
# print(depth_width, depth_height)
# print("Depth_K:\n", depth_k)
# print("Depth_D:\n", depth_d)

# color_width, color_height, color_k, color_d = utils.load_intrinsics(SESSION_PATH, type="color")
# print(color_width, color_height)
# print("color_K:\n", color_k)
# print("color_D:\n", color_d)


# ### Load extrinsics test
T_tool0_color = utils.load_extrinsics(SESSION_PATH, frame_key="T_tool0_color")
# print("T_tool0_color:\n", T_tool0_color)

# T_tool0_ir = utils.load_extrinsics(SESSION_PATH, frame_key="T_tool0_ir")
# print("T_tool0_ir:\n", T_tool0_ir)

### Load manifest test
captures = utils.load_manifest(SESSION_PATH, scan_folder="manual_caps")
# print the pose matrix of the first capture
print("First capture pose matrix:\n", captures[0]["T_base_tool0"])



transformed_captures = utils.transform_robot_to_camera_pose(captures,T_tool0_color ,type="T_tool0_color")
# print the camera pose matrix of the first capture
print("First capture camera pose matrix:\n", transformed_captures[0]["T_base_color"])
print("rgb_path of the first capture:\n", transformed_captures[0]["rgb_path"])

images = utils.load_images(transformed_captures, image_type="rgb", library="cv2")
print("Loaded images count:", len(images))

# display the first image using cv2
cv2.imshow("First RGB Image", images[0])
cv2.waitKey(2000)  
cv2.destroyAllWindows()




