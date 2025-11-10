import cv2
import numpy as np
import open3d as o3d

### load intrinsics
def load_intrinsics(file_path):

    fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise IOError(f"Failed to open: {file_path}")

    try:
        K = fs.getNode("camera_matrix").mat()
        D = fs.getNode("distortion_coefficients").mat()
        width = int(fs.getNode("image_width").real() or 0)
        height = int(fs.getNode("image_height").real() or 0)
        K = np.asarray(K, dtype=np.float64).reshape(3, 3)
        D = np.asarray(D, dtype=np.float64).ravel()
        return width, height, K, D
    finally:
        fs.release()

### create intrinsics matrix
def intrinsics_matrix(width, height,K):
    intrinsics_matrix = o3d.camera.PinholeCameraIntrinsic(
        width, height, K[0,0], K[1,1], K[0,2], K[1,2]
    )
    return intrinsics_matrix

    
