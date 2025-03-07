import matplotlib.pyplot as plt
import numpy as np
import math

def plot_vehicle(ax, vehicle, color='blue'):
    """Plot the skid-steer vehicle as a rectangle with direction"""
    # Calculate the four corners of the vehicle
    corners = []
    
    # Get vehicle dimensions and position
    width = vehicle.width
    length = vehicle.length
    x = vehicle.x
    y = vehicle.y
    yaw = vehicle.yaw
    
    # Vehicle coordinates (local frame)
    corners_local = [
        [-length/2, -width/2],  # rear left
        [length/2, -width/2],   # front left
        [length/2, width/2],    # front right
        [-length/2, width/2],   # rear right
        [-length/2, -width/2]   # back to first point to close the rectangle
    ]
    
    # Transform to global coordinates
    for cx, cy in corners_local:
        # Rotate
        rotated_x = cx * math.cos(yaw) - cy * math.sin(yaw)
        rotated_y = cx * math.sin(yaw) + cy * math.cos(yaw)
        # Translate
        global_x = rotated_x + x
        global_y = rotated_y + y
        corners.append([global_x, global_y])
    
    # Extract x, y coordinates for plotting
    xs, ys = zip(*corners)
    
    # Plot vehicle body
    ax.plot(xs, ys, color=color)
    
    # Plot direction indicator (from center to front)
    front_x = x + 0.6 * length/2 * math.cos(yaw)
    front_y = y + 0.6 * length/2 * math.sin(yaw)
    ax.plot([x, front_x], [y, front_y], color='red', linewidth=2)
    
def init_plot():
    """Initialize the matplotlib plot"""
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Skid-Steer Stanley Controller Simulation')
    return fig, ax

def update_plot(ax, vehicle, path, nearest_idx=None, crosstrack_error=None):
    """Update the plot with current vehicle and path state"""
    ax.clear()
    ax.grid(True)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Skid-Steer Stanley Controller Simulation')
    
    # Plot the path
    ax.plot(path[:, 0], path[:, 1], 'g--', label='Path')
    
    # Plot vehicle
    plot_vehicle(ax, vehicle)
    
    # Mark the origin
    ax.plot(0, 0, 'ko', markersize=10)
    
    # Mark the nearest path point if provided
    if nearest_idx is not None:
        ax.plot(path[nearest_idx, 0], path[nearest_idx, 1], 'ro', markersize=8)
    
    # Show crosstrack error if provided
    if crosstrack_error is not None and nearest_idx is not None:
        # Plot line representing crosstrack error
        if nearest_idx < len(path) - 1:
            path_heading = math.atan2(path[nearest_idx+1, 1] - path[nearest_idx, 1],
                                    path[nearest_idx+1, 0] - path[nearest_idx, 0])
            # Create a perpendicular line to the path
            normal_angle = path_heading + math.pi/2
            error_dx = abs(crosstrack_error) * math.cos(normal_angle)
            error_dy = abs(crosstrack_error) * math.sin(normal_angle)
            
            if crosstrack_error < 0:
                error_dx = -error_dx
                error_dy = -error_dy
                
            ax.plot([vehicle.x, vehicle.x - error_dx],
                   [vehicle.y, vehicle.y - error_dy],
                   'r-', linewidth=2, label=f'Crosstrack: {abs(crosstrack_error):.2f}m')
    
    ax.legend()
    ax.set_xlim([vehicle.x - 10, vehicle.x + 10])
    ax.set_ylim([vehicle.y - 10, vehicle.y + 10])
    
    plt.pause(0.001)