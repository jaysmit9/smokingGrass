import numpy as np
import matplotlib.pyplot as plt
import time
import math

from skid_steer_vehicle import SkidSteerVehicle
from stanley_controller import StanleyController
from path_generator import generate_rectangle_path, generate_circle_path
from visualization import init_plot, update_plot

def find_nearest_point_index(vehicle, path):
    """Find the index of the path point nearest to the vehicle"""
    # Calculate distances from vehicle to all path points
    dx = [vehicle.x - point[0] for point in path]
    dy = [vehicle.y - point[1] for point in path]
    distances = [math.sqrt(idx**2 + idy**2) for idx, idy in zip(dx, dy)]
    
    # Return the index of the minimum distance
    return distances.index(min(distances))

def main():
    """Main simulation loop"""
    print("Starting Skid-Steer Stanley Controller Simulation")
    
    # Create a vehicle instance
    vehicle = SkidSteerVehicle(x=0.0, y=0.0, yaw=0.0, width=1.0, length=1.0)
    
    # Create a controller
    controller = StanleyController(k_crosstrack=20.5, k_heading=80.0)
    
    # Generate a sample path (rectangle)
    path = generate_rectangle_path(0, 0, 10, 8, spacing=0.2)
    
    # Initialize plot
    fig, ax = init_plot()
    
    # Simulation parameters
    dt = 0.1  # time step
    max_steps = 1000
    target_speed = 0.5  # m/s
    
    # Record data for analysis
    time_data = []
    crosstrack_errors = []
    heading_errors = []
    x_positions = []
    y_positions = []
    
    # Main simulation loop
    for step in range(max_steps):
        # Find nearest point on path
        nearest_idx = find_nearest_point_index(vehicle, path)
        
        # Controller calculates wheel speeds
        left_speed, right_speed, crosstrack_error, heading_error = controller.control(
            vehicle, path, nearest_idx, target_speed
        )
        
        # Scale speeds directly while preserving their ratio
        if left_speed != 0 and right_speed != 0:  # Avoid division by zero
            # Remember the original ratio between wheels
            ratio = right_speed / left_speed
            
            # Make speed proportional to error magnitude
            error_magnitude = abs(crosstrack_error) + abs(heading_error)
            speed_factor = 1.0 + min(1.5, error_magnitude)
            
            # Apply speed factor to base speed while preserving ratio
            left_speed *= speed_factor
            right_speed = left_speed * ratio  # This maintains the exact turning ratio
        else:
            # Simple scaling when one wheel is stopped
            error_magnitude = abs(crosstrack_error) + abs(heading_error)
            speed_factor = 1.0 + min(1.5, error_magnitude)
            left_speed *= speed_factor
            right_speed *= speed_factor
        
        # Move vehicle
        vehicle.move(left_speed, right_speed, dt)
        
        # Record data
        time_data.append(step * dt)
        crosstrack_errors.append(crosstrack_error)
        heading_errors.append(heading_error)
        x_positions.append(vehicle.x)
        y_positions.append(vehicle.y)
        
        # Visualization update every few steps
        if step % 5 == 0:
            update_plot(ax, vehicle, path, nearest_idx, crosstrack_error)
            plt.pause(0.01)
        
        # Check if we've reached the end of the path
        # (within a threshold of the last point)
        dist_to_end = math.sqrt((vehicle.x - path[-1, 0])**2 + (vehicle.y - path[-1, 1])**2)
        if dist_to_end < 0.5 and nearest_idx > len(path) - 5:
            print(f"Reached end of path in {step} steps!")
            break
    
    # Final plot update
    update_plot(ax, vehicle, path, nearest_idx, crosstrack_error)
    
    # Plot error history
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(time_data, crosstrack_errors)
    plt.grid(True)
    plt.xlabel('Time [s]')
    plt.ylabel('Crosstrack Error [m]')
    plt.title('Crosstrack Error History')
    
    plt.subplot(2, 1, 2)
    plt.plot(time_data, [math.degrees(err) for err in heading_errors])
    plt.grid(True)
    plt.xlabel('Time [s]')
    plt.ylabel('Heading Error [deg]')
    plt.title('Heading Error History')
    
    plt.tight_layout()
    
    # Plot vehicle path
    plt.figure(figsize=(10, 8))
    plt.plot(path[:, 0], path[:, 1], 'g--', linewidth=2, label='Target Path')
    plt.plot(x_positions, y_positions, 'b-', linewidth=2, label='Vehicle Path')
    plt.grid(True)
    plt.axis('equal')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Path Following Performance')
    plt.legend()
    
    plt.show()

if __name__ == "__main__":
    main()