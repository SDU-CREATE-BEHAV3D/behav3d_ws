import numpy as np
from scipy.spatial.transform import Rotation as R

def rotmat_to_quat_xyzw(Rm: np.ndarray, reorthonormalize: bool = True) -> np.ndarray:
    """
    Convert a 3x3 rotation matrix to a quaternion in (x, y, z, w) order.
    - Rm: 3x3 rotation matrix
    - reorthonormalize: if True, fixes small numeric drift via SVD
    Returns: np.ndarray shape (4,) as [x, y, z, w]
    """
    Rm = np.asarray(Rm, dtype=float)
    assert Rm.shape == (3, 3), "Rm must be 3x3"
    if reorthonormalize:
        U, _, Vt = np.linalg.svd(Rm)
        Rm = U @ Vt
        if np.linalg.det(Rm) < 0:  # ensure right-handed
            U[:, -1] *= -1
            Rm = U @ Vt
    return R.from_matrix(Rm).as_quat()  # xyzw


def rotmat_to_rpy(Rm: np.ndarray, degrees: bool = False, reorthonormalize: bool = True) -> np.ndarray:
    """
    Convert a 3x3 rotation matrix to Euler angles in ROS RPY convention.
    - Uses intrinsic rotations about X, then Y, then Z → 'xyz'
    - Rm: 3x3 rotation matrix
    - degrees: True for degrees, False for radians
    - reorthonormalize: if True, fixes small numeric drift via SVD
    Returns: np.ndarray shape (3,) as [roll, pitch, yaw]
    """
    Rm = np.asarray(Rm, dtype=float)
    assert Rm.shape == (3, 3), "Rm must be 3x3"
    if reorthonormalize:
        U, _, Vt = np.linalg.svd(Rm)
        Rm = U @ Vt
        if np.linalg.det(Rm) < 0:
            U[:, -1] *= -1
            Rm = U @ Vt
    return R.from_matrix(Rm).as_euler('xyz', degrees=degrees)

def compose_T(rvec, tvec):
    T = np.eye(4)
    T[:3, :3] = rvec
    T[:3, 3] = tvec.reshape(3)
    return T
# --- Example usage ---
# Rm = np.array([[-0.99996449, -0.00820984, -0.00190121],
#                [-0.00201247,  0.01356393,  0.99990598],
#                [-0.00818328,  0.99987430, -0.01357997]])
# q_xyzw = rotmat_to_quat_xyzw(Rm)
# rpy_rad = rotmat_to_rpy(Rm, degrees=False)
# rpy_deg = rotmat_to_rpy(Rm, degrees=True)
