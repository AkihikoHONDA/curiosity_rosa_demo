from __future__ import annotations

from typing import Optional

import cv2
import numpy as np


def apply_darkening_and_overlay(
    image_bgr: np.ndarray,
    *,
    score: Optional[float],
    is_good: Optional[bool],
    is_pending: bool,
) -> np.ndarray:
    output = image_bgr.copy()
    if not is_pending and score is not None:
        score_clamped = max(0.0, min(1.0, float(score)))
        factor = 0.2 + 0.8 * score_clamped
        output = (output.astype(np.float32) * factor).clip(0, 255).astype(np.uint8)
        label = "GOOD" if is_good else "BAD"
        text = f"score={score_clamped:.2f} {label}"
        color = (0, 255, 0) if is_good else (0, 0, 255)
    else:
        text = "score=-- PENDING"
        color = (0, 255, 255)

    cv2.putText(
        output,
        text,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        color,
        2,
        cv2.LINE_AA,
    )
    return output
