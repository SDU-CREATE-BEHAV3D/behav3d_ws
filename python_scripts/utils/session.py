import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R

import cv2

class Session:
    def __init__(self, path, init_scan_folder=None, config_folder="config"):

        print(f"Initializing session")

        # Folder structure
        self.path = path
        self._scan_folder = init_scan_folder
        self._config_folder = config_folder

        # Load camera intrinsics
        self._camera_intrinsics_path = self._get_intrinsics_path()
        self.camera_intrinsics_W = None
        self.camera_intrinsics_H = None
        self.camera_intrinsics_K = None
        self.camera_intrinsics_D = None
        self.load_camera_intrinsics()

        # Load camera extrinsics
        self._camera_extrinsics_path = self._get_extrinsics_path()
        # self._camera_extrinsics = self.load_camera_extrinsics() #TODO: implement loading

        self._manifest = None
        self._captures = []

        # Image file paths
        self.depth_file_paths = []
        self.color_file_paths = []
        self.ir_file_paths = []

        # Pre-load scan folder if provided
        if self._scan_folder is not None:
            self.load_scan_folder(self._scan_folder)
        else:
            print("No scan folder provided during initialization.")
        
        self.T_base_tool0 = []


    def load_scan_folder(self, scan_folder):
        print(f"Loading scan folder: {scan_folder}")
        self._scan_folder = scan_folder

        self._manifest = self._read_manifest()
        self._captures = self._manifest.get("captures", [])
        
        self.depth_file_paths = self._get_image_file_paths(image_type="depth")
        self.color_file_paths = self._get_image_file_paths(image_type="rgb") # TODO: replace "rgb" with "color"?
        self.ir_file_paths = self._get_image_file_paths(image_type="ir")
    
    def _read_manifest(self):
        print(f"Reading manifest for scan folder: {self._scan_folder}")
        manifest_path = os.path.join(self.path, self._scan_folder, "manifest.yaml")
        with open(manifest_path, 'r') as f:
            manifest = yaml.safe_load(f)
        return manifest
    
    def _get_image_file_paths(self, image_type):
        file_paths = []
        if image_type in ["depth", "rgb", "ir"]:
            for capture in self._captures:
                file_name = capture.get(image_type)
                if file_name:
                    file_path = os.path.join(self.path, self._scan_folder, file_name)
                    file_paths.append(file_path)
        print(f"Found {len(file_paths)} {image_type} images in scan folder: {self._scan_folder}")
        return file_paths

    def _get_extrinsics_path(self):
        extrinsics_path = os.path.join(self.path, self._config_folder, "extrinsics.yaml")
        if os.path.exists(extrinsics_path):
            return extrinsics_path
        return None # Raise error?
    
    def _get_intrinsics_path(self, optical_frame="depth"):
        intrinsics_path = os.path.join(self.path, self._config_folder, f"{optical_frame}_intrinsics.yaml")
        if os.path.exists(intrinsics_path):
            return intrinsics_path
        return None # Raise error?

    def load_camera_extrinsics(self): #TODO: check implementation and implement parsing
        if self._camera_extrinsics_path is None:
            raise FileNotFoundError("Camera extrinsics file not found.")
        
        with open(self._camera_extrinsics_path, 'r') as f:
            extrinsics = yaml.safe_load(f)
        
        rotation = R.from_euler('xyz', extrinsics['rotation'], degrees=True).as_matrix()
        translation = np.array(extrinsics['translation']).reshape((3, 1))
        
        T_tool0_camera = np.eye(4)
        T_tool0_camera[:3, :3] = rotation
        T_tool0_camera[:3, 3:] = translation
        
        self.T_tool0_camera = T_tool0_camera

    def load_camera_intrinsics(self): #TODO: check implementation and implement parsing
        print(f"Loading camera intrinsics from: {self._camera_intrinsics_path}...", end='')
        if not os.path.isfile(self._camera_intrinsics_path):
            raise FileNotFoundError(f"Intrinsics file not found: {self._camera_intrinsics_path}")

        fs = cv2.FileStorage(self._camera_intrinsics_path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise IOError(f"Failed to open: {self._camera_intrinsics_path}")

        try:
            self.camera_intrinsics_W = int(fs.getNode("image_width").real() or 0)
            self.camera_intrinsics_H = int(fs.getNode("image_height").real() or 0)
            self.camera_intrinsics_K = np.asarray(fs.getNode("camera_matrix").mat(),
                                                   dtype=np.float64).reshape(3, 3)
            self.camera_intrinsics_D = np.asarray(fs.getNode("distortion_coefficients").mat(),
                                                   dtype=np.float64).ravel()
            print("Done.")
            return self.camera_intrinsics_W,\
                   self.camera_intrinsics_H,\
                   self.camera_intrinsics_K,\
                   self.camera_intrinsics_D
        finally:
            fs.release()
    
