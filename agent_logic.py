import random
import math
import numpy as np
from shapely.geometry import LineString, Point

def line_circle_intersection(points_bound, points_inner, line, circle_center, circle_radius, set_points, index):
    line_start = points_bound[int(line[0])] if line[0] > -0.9 else points_inner[int(line[1])]
    line_end = points_bound[int(line[2])] if line[2] > -0.9 else points_inner[int(line[3])]
    
    line_segment = LineString([line_start, line_end])
    circle = Point(circle_center).buffer(circle_radius).boundary
    intersection = line_segment.intersection(circle)
    
    if intersection.is_empty:
        return set_points
    elif intersection.geom_type == "Point":
        set_points.append((intersection.x, intersection.y, index))
    elif intersection.geom_type == "MultiPoint":
        for point in intersection.geoms:
            set_points.append((point.x, point.y, index))
    return set_points

def choose_intersection_randomly(intersections):
    if not intersections:
        return None
    return random.choice(list(intersections))

def calculate_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


def e_calculate_distance(line, bound_point, move_point):
    """
    calculate real distance
    """
    # 시작점 좌표 결정
    if line[0] > -0.9:
        line_start = bound_point[int(line[0])]
    else:
        line_start = move_point[int(line[1])]

    # 끝점 좌표 결정
    if line[2] > -0.9:
        line_end = bound_point[int(line[2])]
    else:
        line_end = move_point[int(line[3])]

    return math.sqrt((line_end[0] - line_start[0])**2 + (line_end[1] - line_start[1])**2)
