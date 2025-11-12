import os
import cv2
import open3d as o3d
import numpy as np

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
# ---------- Color loading ----------
def load_color_image(path: str):
    """Load RGB/BGR image as uint8 BGR. If 16-bit, normalize (1–99 pct) to 8-bit."""
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None:
        return None

    # Ensure 3 channels
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # Normaliza if it is 16-bit (usa percentiles globales para simplicidad)
    if img.dtype == np.uint16:
        p1, p99 = np.percentile(img, (1, 99))
        if p99 <= p1:
            p1, p99 = float(img.min()), float(img.max())
        img = np.clip(img, p1, p99)
        img = ((img - p1) * (255.0 / (p99 - p1 + 1e-9))).astype(np.uint8)
    else:
        img = img.astype(np.uint8, copy=False)

    return img

# ---------- IR-aware loading + preprocessing ----------
def load_ir_image(path: str):
    """For cv2: Load IR image and return a single-channel uint8 grayscale image (normalized if 16-bit)."""
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None:
        return None
    # Ensure single-channel
    if img.ndim == 3:  # e.g., BGR
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if img.dtype == np.uint16:
        p1, p99 = np.percentile(img, (1, 99))
        if p99 <= p1:
            p1, p99 = float(img.min()), float(img.max())
        img = np.clip(img, p1, p99)
        gray = ((img - p1) * (255.0 / (p99 - p1 + 1e-9))).astype(np.uint8)
    else:
        gray = img.astype(np.uint8, copy=False)
    return gray

def preprocess_ir(gray, ir_threshold=None, clip=3.0, tile=(8,8), p_low=2, p_high=98, gamma=None, alpha=None, beta=0):
    """
    Steps:
      1) Optional cap at ir_threshold
      2) Robust contrast stretch via percentiles (p_low..p_high)
      3) Optional gamma (nonlinear)
      4) Optional linear gain/offset with convertScaleAbs (alpha, beta)
      5) CLAHE
    Returns uint8.
    """
    g = gray
    # Ensure single-channel
    if g.ndim == 3:
        g = cv2.cvtColor(g, cv2.COLOR_BGR2GRAY)
    # 1) Optional cap
    if ir_threshold is not None:
        g = np.minimum(g, ir_threshold)
    # Convert to float for math
    g = g.astype(np.float32, copy=False)
    # 2) Robust percentile stretch to 0..255
    lo, hi = np.percentile(g, [p_low, p_high])
    if hi <= lo:   # fallback to global min/max if degenerate
        lo, hi = float(g.min()), float(g.max()) if g.size else (0.0, 1.0)
    g = np.clip((g - lo) / max(hi - lo, 1e-6), 0, 1) * 255.0
    # 3) Optional gamma (apply before CLAHE)
    if gamma is not None and gamma > 0:
        g = 255.0 * np.power(g / 255.0, gamma)
    # 4) Optional linear scale/offset (OpenCV-style)
    if alpha is not None:
        g = cv2.convertScaleAbs(g, alpha=alpha, beta=beta)
    else:
        g = g.astype(np.uint8, copy=False)
    # 5) CLAHE (smaller tiles = more local contrast; lower clip = less noise amplification)
    clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=tile)
    g = clahe.apply(g)
    return g
