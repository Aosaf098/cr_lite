"""Static workspace and furniture keep-out checks."""

import yaml


def load_safety(config_path):
    with open(config_path) as f:
        return yaml.safe_load(f)["safety"]


def point_in_box(point, bounds):
    """True when a point is inside or on an AABB boundary."""
    return all(
        bounds[i] <= point[i] <= bounds[i + 3]
        for i in range(3)
    )


def check_position(point, safety):
    """Return (is_safe, reason)."""
    if not point_in_box(point, safety["workspace_bounds"]):
        return False, "workspace"

    for box in safety["keepout_boxes"]:
        if point_in_box(point, box["bounds"]):
            return False, box["name"]

    return True, None


def segment_intersects_box(start, end, bounds):
    """Check a line segment against an axis-aligned box."""
    t_min, t_max = 0.0, 1.0

    for i in range(3):
        delta = end[i] - start[i]
        lo, hi = bounds[i], bounds[i + 3]

        if abs(delta) < 1e-12:
            if start[i] < lo or start[i] > hi:
                return False
            continue

        a = (lo - start[i]) / delta
        b = (hi - start[i]) / delta

        t_min = max(t_min, min(a, b))
        t_max = min(t_max, max(a, b))

        if t_min > t_max:
            return False

    return True


def check_motion(start, end, safety):
    """Validate endpoints and the entire movement segment."""
    for point in (start, end):
        safe, reason = check_position(point, safety)
        if not safe:
            return False, reason

    for box in safety["keepout_boxes"]:
        if segment_intersects_box(start, end, box["bounds"]):
            return False, box["name"]

    return True, None
