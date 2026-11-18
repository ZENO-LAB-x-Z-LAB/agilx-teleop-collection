import math
from typing import Optional


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def formal_width_from_status(status, gripper_range: float) -> Optional[float]:
    if status is None:
        return None
    mode = str(getattr(status, "mode", "")).strip().lower()
    raw_width = float(
        getattr(status, "width", math.nan)
        if not math.isnan(float(getattr(status, "width", math.nan)))
        else getattr(status, "position", math.nan)
    )
    if mode == "width":
        if not math.isfinite(raw_width):
            return None
        return clamp(raw_width, 0.0, float(gripper_range))

    if mode == "angle":
        raw_position = float(getattr(status, "position", math.nan))
        if not math.isfinite(raw_position):
            return None
        # Some width-command feedback keeps reporting mode=angle while carrying
        # a stroke-like value in millimeters.
        width_candidate = raw_position * 1e-3
        if 0.0 <= width_candidate <= float(gripper_range):
            return clamp(width_candidate, 0.0, float(gripper_range))
    return None


def require_formal_width_from_status(
    status,
    gripper_range: float,
    source: str = "gripper",
) -> Optional[float]:
    if status is None:
        return None
    mode = str(getattr(status, "mode", "")).strip().lower()
    width = formal_width_from_status(status, gripper_range)
    if width is None and mode != "width":
        raise ValueError(
            f"{source} feedback must be width mode or stroke-like angle fallback, "
            f"got mode={mode!r}. "
            "Refusing to mix gripper units."
        )
    if width is None:
        raise ValueError(
            f"{source} feedback is in width mode but has no finite width value."
        )
    return width


def master_to_follower_opening(
    master_opening: float,
    geom_scale: float,
    geom_offset: float,
    gripper_range: float,
) -> float:
    """Map master logical opening to follower logical opening.

    The fitted geometry model is:
        follower_opening ~= geom_scale * master_opening + geom_offset
    """
    return clamp(
        float(geom_scale) * float(master_opening) + float(geom_offset),
        0.0,
        float(gripper_range),
    )


def follower_to_master_opening(
    follower_opening: float,
    geom_scale: float,
    geom_offset: float,
    gripper_range: float,
) -> float:
    """Map follower logical opening back into the master coordinate."""
    scale = float(geom_scale)
    if abs(scale) < 1e-9:
        scale = 1.0
    return clamp(
        (float(follower_opening) - float(geom_offset)) / scale,
        0.0,
        float(gripper_range),
    )
