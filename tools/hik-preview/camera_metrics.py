"""Exposure and focus feedback shared by the two cameras."""

import threading
import time
import winsound

import cv2
import numpy as np


class FocusTone:
    def __init__(self):
        self.last_beep = 0.0
        self.thread = None

    def update(self, score, ratio, saturation):
        now = time.monotonic()
        if (score <= 1 or saturation >= .8 or now - self.last_beep <= .4
                or (self.thread is not None and self.thread.is_alive())):
            return
        self.last_beep = now
        frequency = int(350 + 900 * np.clip(ratio, 0, 1))
        self.thread = threading.Thread(target=winsound.Beep, args=(frequency, 90), daemon=True)
        self.thread.start()


def focus_score(gray):
    gray = np.asarray(gray, dtype=np.uint8)
    if gray.size < 400:
        return 0.0, 0.0
    saturation = float(np.mean((gray <= 5) | (gray >= 250)))
    score = float(cv2.Laplacian(cv2.GaussianBlur(gray, (3, 3), 0), cv2.CV_64F).var())
    return score, saturation


def roi_brightness(gray, roi):
    height, width = gray.shape
    x0, y0, x1, y1 = roi
    left = min(width - 1, int(x0 * width))
    top = min(height - 1, int(y0 * height))
    right = max(left + 1, int(x1 * width))
    bottom = max(top + 1, int(y1 * height))
    region = gray[top:bottom, left:right]
    if region.size < 400:
        raise ValueError('测光框过小，请框住更大的物体区域')
    sample = region[::max(1, region.shape[0] // 400), ::max(1, region.shape[1] // 400)]
    return float(np.percentile(sample, 90)), float(np.mean(sample >= 250))


def exposure_score(p90, clipped):
    return max(0., clipped - .05), abs(p90 - 190)
