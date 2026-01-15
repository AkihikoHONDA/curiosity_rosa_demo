from __future__ import annotations

from curiosity_rosa_demo.domain.models import LightScore


class LightModel:
    def __init__(self, *, x_min: float, x_good: float, score_threshold: float) -> None:
        if x_good <= x_min:
            raise ValueError("x_good must be greater than x_min")
        self._x_min = float(x_min)
        self._x_good = float(x_good)
        self._score_threshold = float(score_threshold)

    def compute(self, rover_x: float) -> LightScore:
        score = (float(rover_x) - self._x_min) / (self._x_good - self._x_min)
        score = max(0.0, min(1.0, score))
        is_good = score >= self._score_threshold
        return LightScore(score=score, is_good=is_good)
