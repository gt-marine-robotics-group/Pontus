from pontus_perception.slalom_detector import SlalomPair, ThresholdDetection

import math
from image_geometry import PinholeCameraModel


# Initialize once from a CameraInfo message
model = PinholeCameraModel()
model.fromCameraInfo()  # subscribe to /camera_info


def heading_to_pole(box, model: PinholeCameraModel) -> float:
    u = box.x + 0.5 * box.w
    # bearing around vertical axis (right positive)
    return math.atan2(u - model.cx(), model.fx())


def distance_to_pole(box, model: PinholeCameraModel, pole_diameter: float) -> float:
    u_l = box.x
    u_r = box.x + box.w
    a_l = math.atan2(u_l - model.cx(), model.fx())
    a_r = math.atan2(u_r - model.cx(), model.fx())
    alpha = abs(a_r - a_l)
    # guard tiny angles
    alpha = max(alpha, 1e-6)
    return (pole_diameter / 2.0) / math.tan(alpha / 2.0)
