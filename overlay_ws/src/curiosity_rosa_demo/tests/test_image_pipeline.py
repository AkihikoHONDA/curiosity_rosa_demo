import numpy as np

from curiosity_rosa_demo.sim.image_pipeline import apply_darkening_and_overlay


def test_apply_darkening_reduces_brightness():
    image = np.full((64, 64, 3), 200, dtype=np.uint8)
    darkened = apply_darkening_and_overlay(
        image,
        score=0.0,
        is_good=False,
        is_pending=False,
    )
    assert darkened.shape == image.shape
    assert darkened.mean() < image.mean()


def test_pending_keeps_brightness():
    image = np.full((64, 64, 3), 120, dtype=np.uint8)
    pending = apply_darkening_and_overlay(
        image,
        score=None,
        is_good=None,
        is_pending=True,
    )
    assert pending.shape == image.shape
    assert pending.mean() >= image.mean() - 1.0
