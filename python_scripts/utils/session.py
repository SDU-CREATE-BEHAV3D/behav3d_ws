import os
import numpy as np
from scipy.spatial.transform import Rotation as R

class Session:
    def __init__(self, path, init_scan_folder=None, config_folder="config"):

        print(f"Initializing session")

        # Folder structure
        self.path = path
        self._scan_folder = init_scan_folder
        self._config_folder = config_folder

        self.color_intrinsics_path = self._get_intrinsics_path(optical_frame="color")
        self.ir_intrinsics_path = self._get_intrinsics_path(optical_frame="ir")
        self.depth_intrinsics_path = self._get_intrinsics_path(optical_frame="depth")
        
        self._camera_extrinsics_path = self._get_extrinsics_path()

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

    def load_scan_folder(self, scan_folder):
        print(f"Loading scan folder: {scan_folder}")
        self._scan_folder = scan_folder
        
        self.depth_file_paths = self._get_image_file_paths(image_type="depth")
        self.color_file_paths = self._get_image_file_paths(image_type="color") 
        self.ir_file_paths = self._get_image_file_paths(image_type="ir")
    
    def _get_image_file_paths(self, image_type):
        file_paths = []
        if image_type in ["depth", "color", "ir"]:
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
