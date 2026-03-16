# strip_core.py
import numpy as np
import cv2

class StripMap:
    def __init__(self, z_min=0.0, z_max=0.6, dz=0.01, width_cols=600,
                 range_min=0.1, range_max=6.0):
        self.z_min = z_min
        self.z_max = z_max
        self.dz = dz
        self.range_min = range_min
        self.range_max = range_max
        self.z_bins = int(np.ceil((z_max - z_min) / dz))
        self.width = width_cols
        self.data = np.full((self.z_bins, width_cols), np.nan, dtype=np.float32)

    def add_column(self, depths, heights):
        """Add a new LiDAR scan column (depth vs z)."""
        col = np.full(self.z_bins, np.nan, dtype=np.float32)
        z_idx = ((heights - self.z_min) / self.dz).astype(int)
        keep = (z_idx >= 0) & (z_idx < self.z_bins)
        for d, zi in zip(depths[keep], z_idx[keep]):
            if np.isnan(col[zi]):
                col[zi] = d
        self.data[:, :-1] = self.data[:, 1:]
        self.data[:, -1] = col

    def to_image(self):
        s = np.copy(self.data)
        s[np.isnan(s)] = self.range_max
        norm = (self.range_max - s) / (self.range_max - self.range_min + 1e-6)
        img = (norm * 255).astype(np.uint8)
        img = np.flipud(img)
        return img

    def apply_median_filter(self, ksize=3):
        valid = np.where(np.isnan(self.data), self.range_max, self.data)
        filtered = cv2.medianBlur(valid.astype(np.float32), ksize)
        self.data = filtered
