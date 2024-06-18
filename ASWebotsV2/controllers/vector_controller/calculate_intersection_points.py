import numpy as np
import math

def calculate_intersection_points(coord1, coord2, radius):
    """Calculate the intersection points of two bots given their positions and direction vectors."""
    coord1 = np.array(coord1)
    coord2 = np.array(coord2)
    
    d = np.linalg.norm(coord1 - coord2)
    
    # No intersection if distance is greater than 2 times the radius or zero
    if d > 2 * radius or d == 0:
        return None
    
    radius_squared = radius**2
    a = d / 2
    h = math.sqrt(radius_squared - a**2)
    
    midpoint = (coord1 + coord2) / 2
    
    direction = (coord2 - coord1) / d
    
    perpendicular = np.array([-direction[1], direction[0]])
    
    intersection1 = midpoint + h * perpendicular
    intersection2 = midpoint - h * perpendicular
    
    # Calculating the midpoint
    intersection_midpoint = (intersection1 + intersection2) / 2
    
    return tuple(intersection_midpoint)
coord1 = (0, 0)
coord2 = (4, 0)
radius = 2
print(calculate_intersection_points(coord1, coord2, radius))
