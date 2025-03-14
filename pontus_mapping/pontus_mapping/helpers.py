# Helper functions for polygon membership and fov polygon generation

import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
import tf_transformations


PERCEPTION_FOV = 0.8
PERCEPTION_DISTANCE = 5.0


def get_fov_polygon(current_odometry: Odometry) -> Polygon:
    """
    Return a polygon in the map frame that defines our fov.

    Args:
    ----
        current_odometry (nav_msgs/Odometry): our current odometry
        perception_fov (float): the angle of our perception fov in radians
        perception_distance (float): the max distance we can reliablely see in meters

    Return:
    ------
        (geometry_msgs/Polygon) : the polygon representing our FOV

    """
    fov_polygon = Polygon()
    current_x = current_odometry.pose.pose.position.x
    current_y = current_odometry.pose.pose.position.y

    fov_polygon.points.append(Point32(x=np.float32(current_x), y=np.float32(current_y)))
    quat = [current_odometry.pose.pose.orientation.x,
            current_odometry.pose.pose.orientation.y,
            current_odometry.pose.pose.orientation.z,
            current_odometry.pose.pose.orientation.w]
    _, _, current_yaw = tf_transformations.euler_from_quaternion(quat)
    r0, theta0 = PERCEPTION_DISTANCE, current_yaw - PERCEPTION_FOV / 2
    r1, theta1 = PERCEPTION_DISTANCE, current_yaw + PERCEPTION_FOV / 2

    x0 = np.float32(current_x + r0 * np.cos(theta0))
    y0 = np.float32(current_y + r0 * np.sin(theta0))
    x1 = np.float32(current_x + r1 * np.cos(theta1))
    y1 = np.float32(current_y + r1 * np.sin(theta1))
    fov_polygon.points.append(Point32(x=x0, y=y0))
    fov_polygon.points.append(Point32(x=x1, y=y1))
    return fov_polygon


def polygon_contained(polygon: Polygon, point: tuple[float, float]) -> bool:
    """
    Determine if a given point is within a polygon.

    This uses the ray-casting algorithm

    Args:
    ----
        polygon (Polygon): the bounding polygon
        point (tuple[float, float]): the point to be checked if inside the polygon

    Return:
    ------
        bool: whether or not the point is contained within the polygon

    """
    x, y = point[0], point[1]
    vertices = polygon.points
    n = len(vertices)
    inside = False
    j = n - 1
    # If a ray casted by the point has an even number of intersections with the polygon,
    # then it is outside of the polygon
    for i in range(n):
        xi, yi = vertices[i].x, vertices[i].y
        xj, yj = vertices[j].x, vertices[j].y
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside
