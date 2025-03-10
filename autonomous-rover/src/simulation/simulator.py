import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import logging
from geopy.distance import geodesic

logger = logging.getLogger("rover.simulation")

def haversine_distance(coord1, coord2):
    """Calculate the great circle distance between two points in meters"""
    return geodesic(coord1, coord2).meters

def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]"""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle

def simulate(waypoints, x0, y0, yaw0, v0, target_idx=1, k=4.8, k_e=10.0, 
             target_speed=2.0, dt=0.1, L=1.0, max_steer=np.radians(50), 
             max_simulation_steps=1000, waypoint_reach_threshold=1.0,
             METERS_PER_LAT_DEGREE=111000):
    """Simulate rover movement with Stanley controller"""
    # State variables
    x = x0
    y = y0
    yaw = yaw0
    v = v0
    
    # For storing results
    times = [0.0]
    xs = [x]
    ys = [y]
    yaws = [yaw]
    vs = [v]
    targets = [target_idx]
    steers = [0.0]
    ctes = [0.0]
    
    # Simulation loop
    for i in range(max_simulation_steps):
        # Calculate meters per lon degree based on current latitude
        meters_per_lon_degree = METERS_PER_LAT_DEGREE * np.cos(np.radians(x))
        
        # Target waypoint
        current_waypoint = waypoints[max(0, target_idx-1)]
        target_waypoint = waypoints[target_idx]
        
        # Calculate heading error - angle to the target
        dx = target_waypoint[0] - x
        dy = target_waypoint[1] - y
        desired_course_angle = np.arctan2(dy * meters_per_lon_degree, dx * METERS_PER_LAT_DEGREE)
        heading_error = normalize_angle(desired_course_angle - yaw)
        
        # Calculate cross-track error
        # Vector from previous waypoint to current position
        vx = (x - current_waypoint[0]) * METERS_PER_LAT_DEGREE
        vy = (y - current_waypoint[1]) * meters_per_lon_degree
        
        # Vector representing the path
        wx = (target_waypoint[0] - current_waypoint[0]) * METERS_PER_LAT_DEGREE
        wy = (target_waypoint[1] - current_waypoint[1]) * meters_per_lon_degree
        
        # Path length
        path_length_squared = wx * wx + wy * wy
        
        # Calculate cross-track error
        if path_length_squared < 1e-6:
            cte = haversine_distance((x, y), current_waypoint)
        else:
            # Calculate projection
            proj = (wx * vx + wy * vy) / path_length_squared
            proj = max(0.0, min(1.0, proj))  # Clamp to [0, 1]
            
            # Calculate closest point on the path
            cx = current_waypoint[0] + proj * (target_waypoint[0] - current_waypoint[0])
            cy = current_waypoint[1] + proj * (target_waypoint[1] - current_waypoint[1])
            
            # Cross-track error in meters
            cte = haversine_distance((x, y), (cx, cy))
            
            # Determine sign of cross-track error
            cross_product = wx * vy - wy * vx
            if cross_product > 0:
                cte = -cte  # Left of path
        
        # Stanley control law
        delta_heading = k * heading_error
        delta_cte = np.arctan2(k_e * cte, v + 0.1)  # Adding small value to avoid division by zero
        delta = delta_heading + delta_cte
        delta = np.clip(delta, -max_steer, max_steer)
        
        # Update velocity
        v += 0.5 * (target_speed - v) * dt  # Simple P controller
        v = max(0.0, v)  # Ensure non-negative speed
        
        # Update position using bicycle model
        x += v * np.cos(yaw) * dt / METERS_PER_LAT_DEGREE
        y += v * np.sin(yaw) * dt / meters_per_lon_degree
        yaw += v * np.tan(delta) * dt / L
        yaw = normalize_angle(yaw)
        
        # Check if reached target waypoint
        distance_to_target = haversine_distance((x, y), (target_waypoint[0], target_waypoint[1]))
        
        if distance_to_target < waypoint_reach_threshold and target_idx < len(waypoints) - 1:
            target_idx += 1
            logger.info(f"Reached waypoint {target_idx-1}")
        
        # Store results
        times.append(times[-1] + dt)
        xs.append(x)
        ys.append(y)
        yaws.append(yaw)
        vs.append(v)
        targets.append(target_idx)
        steers.append(delta)
        ctes.append(cte)
        
        # Stop if we've reached the final waypoint
        if target_idx == len(waypoints) - 1 and distance_to_target < waypoint_reach_threshold:
            logger.info("Reached final waypoint")
            break
    
    return {
        'time': times,
        'x': xs,
        'y': ys,
        'yaw': yaws,
        'v': vs,
        'target_idx': targets,
        'steer': steers,
        'cte': ctes
    }

def plot_results(results, waypoints):
    """Plot the simulation results"""
    plt.figure(figsize=(15, 10))
    
    # Plot path and waypoints
    plt.subplot(2, 2, 1)
    plt.plot(waypoints[:, 1], waypoints[:, 0], 'b-', label='Path')
    plt.plot(waypoints[:, 1], waypoints[:, 0], 'r.', label='Waypoints')
    plt.plot(results['y'], results['x'], 'g-', label='Rover')
    plt.legend()
    plt.title('Path and Trajectory')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.grid(True)
    plt.axis('equal')
    
    # Plot steering angle
    plt.subplot(2, 2, 2)
    plt.plot(results['time'], np.degrees(results['steer']), 'r-')
    plt.title('Steering Angle')
    plt.xlabel('Time [s]')
    plt.ylabel('Steering Angle [deg]')
    plt.grid(True)
    
    # Plot velocity
    plt.subplot(2, 2, 3)
    plt.plot(results['time'], results['v'], 'b-')
    plt.title('Velocity')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.grid(True)
    
    # Plot cross-track error
    plt.subplot(2, 2, 4)
    plt.plot(results['time'], results['cte'], 'k-')
    plt.title('Cross-Track Error')
    plt.xlabel('Time [s]')
    plt.ylabel('CTE [m]')
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('simulation_results.png')
    plt.show()

def create_animation(results, waypoints):
    """Create an animation of the rover's movement"""
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Plot the waypoints and path
    ax.plot(waypoints[:, 1], waypoints[:, 0], 'r-', label='Path')
    ax.plot(waypoints[:, 1], waypoints[:, 0], 'k.', markersize=8, label='Waypoints')
    
    # Rover marker
    rover, = ax.plot([], [], 'go', markersize=10, label='Rover')
    
    # Rover direction indicator
    direction, = ax.plot([], [], 'g-', linewidth=2)
    
    # Set plot limits
    x_min, x_max = min(waypoints[:, 1]) - 0.0005, max(waypoints[:, 1]) + 0.0005
    y_min, y_max = min(waypoints[:, 0]) - 0.0005, max(waypoints[:, 0]) + 0.0005
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_title('Rover Simulation')
    ax.legend()
    ax.grid(True)
    
    def update(frame):
        if frame < len(results['x']):
            # Update rover position
            rover.set_data(results['y'][frame], results['x'][frame])
            
            # Update direction indicator
            yaw = results['yaw'][frame]
            arrow_len = 0.0002  # Length of the direction indicator
            dx = arrow_len * np.sin(yaw)
            dy = arrow_len * np.cos(yaw)
            direction.set_data([results['y'][frame], results['y'][frame] + dx],
                               [results['x'][frame], results['x'][frame] + dy])
        return rover, direction
    
    ani = FuncAnimation(fig, update, frames=len(results['x']), interval=50, blit=True)
    ani.save('rover_animation.gif', writer='pillow', fps=20)
    plt.show()