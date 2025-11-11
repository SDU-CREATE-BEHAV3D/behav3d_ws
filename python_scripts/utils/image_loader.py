import os
import cv2
import open3d as o3d


### load images from absolute paths
def load_images(image_paths, image_type, library):
    images = []
    for img_path in image_paths:
        if not os.path.isfile(img_path):
            raise FileNotFoundError(f"Image file not found: {img_path}")
        
        if library == "cv2":
            if image_type == "depth":
                img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
            else:
                img = cv2.imread(img_path, cv2.IMREAD_COLOR)
            images.append(img)
        elif library == "o3d":
            img = o3d.io.read_image(img_path)
            images.append(img)
        else:
            raise ValueError(f"Unsupported library: {library}")
    return images


