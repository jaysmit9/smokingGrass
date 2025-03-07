import numpy as np
import math

def generate_line_path(start_x, start_y, end_x, end_y, spacing=0.5):
    """Generate a straight-line path with given spacing"""
    dist = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
    num_points = max(2, int(dist/spacing))
    
    x = np.linspace(start_x, end_x, num_points)
    y = np.linspace(start_y, end_y, num_points)
    
    return np.column_stack((x, y))

def generate_circle_path(center_x, center_y, radius, spacing=0.5):
    """Generate a circular path with given spacing"""
    circumference = 2 * math.pi * radius
    num_points = max(8, int(circumference/spacing))
    
    angles = np.linspace(0, 2*math.pi, num_points)
    x = center_x + radius * np.cos(angles)
    y = center_y + radius * np.sin(angles)
    
    return np.column_stack((x, y))

def generate_rectangle_path(center_x, center_y, width, height, spacing=0.5):
    """Generate a rectangular path"""
    half_width = width / 2
    half_height = height / 2
    
    # Corners of the rectangle
    corners = [
        (center_x - half_width, center_y - half_height),
        (center_x + half_width, center_y - half_height),
        (center_x + half_width, center_y + half_height),
        (center_x - half_width, center_y + half_height),
        (center_x - half_width, center_y - half_height)  # Close the loop
    ]
    
    path = []
    for i in range(len(corners)-1):
        segment = generate_line_path(corners[i][0], corners[i][1], 
                                    corners[i+1][0], corners[i+1][1], spacing)
        if i > 0:
            segment = segment[1:]  # Avoid duplicating points
        path.extend(segment)
        
    return np.array(path)