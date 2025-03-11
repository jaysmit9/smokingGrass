import argparse
import numpy as np
import json
import time
import sys
import os
from datetime import datetime
import logging
from geopy.distance import geodesic  # Add this import for geodesic calculations

# Import Stanley controller components
from controllers.stanley_controller import StanleyController

# Import hardware components
from hardware.gps_monitor import GPSMonitor  # Add this import
from hardware.motor_controller import get_motor_controller  # Add this import

# Conditionally import visualization modules for simulation mode
try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(f"rover_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("rover")

def load_waypoints(waypoints_file):
    """Load waypoints from file with better validation."""
    try:
        logger.info(f"Loading waypoints from: {waypoints_file}")
        waypoints = []
        
        # Determine file type and load accordingly
        if waypoints_file.endswith('.json'):
            with open(waypoints_file, 'r') as f:
                data = json.load(f)
                if isinstance(data, list) and len(data) > 0:
                    # Extract waypoints based on format
                    if isinstance(data[0], list):  # Simple format [[lat, lon], ...]
                        waypoints = np.array(data, dtype=np.float64)
                    elif isinstance(data[0], dict) and 'lat' in data[0] and 'lon' in data[0]:
                        waypoints = np.array([[point['lat'], point['lon']] for point in data], dtype=np.float64)
        else:
            # Assume it's a CSV or similar text format
            with open(waypoints_file, 'r') as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) >= 2:
                        try:
                            lat = float(parts[0])
                            lon = float(parts[1])
                            waypoints.append([lat, lon])
                        except ValueError:
                            continue
            waypoints = np.array(waypoints, dtype=np.float64)
        
        # Validate waypoints
        if len(waypoints) < 2:
            logger.error(f"Not enough valid waypoints found (need at least 2, found {len(waypoints)})")
            return np.array([[0, 0], [0.0001, 0.0001]])  # Provide dummy waypoints
        
        # Log all waypoints after loading (IMPORTANT)
        logger.info(f"Successfully loaded {len(waypoints)} waypoints:")
        for i, wp in enumerate(waypoints):
            logger.info(f"WP {i}: ({wp[0]:.7f}, {wp[1]:.7f})")
            
        return waypoints
        
    except Exception as e:
        logger.error(f"Error loading waypoints: {e}")
        # Return dummy waypoints if loading fails
        return np.array([[0, 0], [0.0001, 0.0001]])

def run_simulation(waypoints_file, config=None):
    """Run the rover in simulation mode"""
    if not MATPLOTLIB_AVAILABLE:
        logger.error("Matplotlib is required for simulation mode")
        return
    
    # Default configuration updated for tanh-based controller
    if config is None:
        config = {
            'dt': 0.1,                      # Time step (seconds)
            'max_steer': np.radians(90),    # Maximum steering angle
            'target_speed': 1.0,            # Target speed (m/s)
            'extension_distance': 1.0,      # Extension distance beyond final waypoint
            'waypoint_reach_threshold': 1.0, # Distance threshold to consider waypoint reached
            'steering_sensitivity': np.pi/36  # Denominator for tanh function (lower = more aggressive)
        }
    
    # Load waypoints
    waypoints = load_waypoints(waypoints_file)
    
    # Initialize the controller with new parameters
    controller = StanleyController(
        waypoints=waypoints,
        max_steer=config['max_steer'],
        waypoint_reach_threshold=config['waypoint_reach_threshold'],
        steering_sensitivity=config.get('steering_sensitivity', np.pi/3)
    )
    
    # Add extension waypoint
    waypoints = controller.add_extension_waypoint(config['extension_distance'])
    
    # Rest of the function remains the same
    # Initial state setup
    x = waypoints[0, 0]  # Initial latitude
    y = waypoints[0, 1]  # Initial longitude
    v = 0.0              # Initial velocity
    
    # Set initial yaw to face the first waypoint
    dx = waypoints[1, 0] - x
    dy = waypoints[1, 1] - y
    yaw = np.arctan2(dy, dx)
    
    # Constants for geographic coordinate conversion
    METERS_PER_LAT_DEGREE = 111000
    
    # Run simulation
    logger.info("Starting simulation...")
    
    # Import simulation specific functions
    from simulation.simulator import simulate, plot_results, create_animation
    
    results = simulate(
        waypoints, x, y, yaw, v, 
        target_idx=1,  # Start with second waypoint as target
        target_speed=config['target_speed'],
        dt=config['dt'],
        max_steer=config['max_steer'],
        steering_sensitivity=config['steering_sensitivity'],  # New parameter
        waypoint_reach_threshold=config['waypoint_reach_threshold'],
        METERS_PER_LAT_DEGREE=METERS_PER_LAT_DEGREE
    )
    
    # Check if we should replay the simulation with progress logs
    replay_simulation = '--replay' in sys.argv
    
    if replay_simulation:
        logger.info("Replaying simulation with progress logging...")
        replay_duration = min(30.0, len(results['time']) * config['dt'])  # Max 30 seconds of replay
        replay_interval = 0.1  # Update every 100ms
        
        # Calculate how many steps to skip to fit within replay_duration
        total_steps = len(results['time'])
        step_skip = max(1, int(total_steps * replay_interval / replay_duration))
        
        # Create a trajectory file for the simulation too
        trajectory_file = open(f"sim_trajectory_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "w")
        trajectory_file.write("timestamp,lat,lon,heading,speed,target_idx,distance,cte\n")
        
        last_progress_log_time = time.time()
        
        for i in range(0, total_steps, step_skip):
            # Get state at this step
            x = results['x'][i]
            y = results['y'][i]
            yaw = results['yaw'][i]
            v = results['v'][i]
            target_idx = results['target_idx'][i]
            delta = results['delta'][i]
            cte = results['cte'][i]
            
            # Get distance to current waypoint
            if target_idx < len(waypoints):
                target_wp = waypoints[target_idx]
                distance = geodesic((x, y), (target_wp[0], target_wp[1])).meters
            else:
                distance = 0.0
            
            # Convert yaw to heading (0-360 degrees)
            heading = (np.degrees(yaw) + 360) % 360
            
            # Convert steering angle to degrees
            steering_degrees = np.degrees(delta)
            
            # Write to trajectory file
            current_time = time.time()
            trajectory_file.write(f"{current_time},{x},{y},{heading},"
                                 f"{v},{target_idx},{distance},{cte}\n")
            
            # Progress logging (once per second)
            if current_time - last_progress_log_time >= 1.0:
                last_progress_log_time = current_time
                
                # Calculate percentage to next waypoint if we know the previous waypoint
                progress_percent = "unknown"
                if target_idx > 0 and target_idx < len(waypoints):
                    # Calculate distance between previous and current waypoint
                    prev_wp = waypoints[target_idx - 1]
                    target_wp = waypoints[target_idx]
                    total_segment_dist = geodesic((prev_wp[0], prev_wp[1]), 
                                                (target_wp[0], target_wp[1])).meters
                    
                    # Calculate progress percentage
                    if total_segment_dist > 0:
                        dist_from_prev = geodesic((prev_wp[0], prev_wp[1]), 
                                                (x, y)).meters
                        progress = min(100, (dist_from_prev / total_segment_dist) * 100)
                        progress_percent = f"{progress:.1f}%"
                
                # Get bearing to target waypoint
                if target_idx < len(waypoints):
                    target_wp = waypoints[target_idx]
                    bearing_to_target = calculate_bearing(x, y, target_wp[0], target_wp[1])
                    
                    # Create progress bar
                    progress_bar_len = 20
                    if distance > 0:
                        threshold = config['waypoint_reach_threshold']
                        progress_normalized = min(1.0, max(0, (threshold - min(threshold, distance)) / threshold))
                        filled = int(progress_normalized * progress_bar_len)
                        progress_bar = f"[{'#' * filled}{'-' * (progress_bar_len - filled)}]"
                    else:
                        progress_bar = "[--------------------]"
                    
                    logger.info(f"\n===== SIMULATION PROGRESS =====")
                    logger.info(f"Waypoint: {target_idx}/{len(waypoints)-1} {progress_bar}")
                    logger.info(f"Position: {x:.6f}, {y:.6f} | Progress: {progress_percent}")
                    logger.info(f"Heading: {heading:.1f}° | Bearing to target: {bearing_to_target:.1f}°")
                    logger.info(f"Distance to waypoint: {distance:.2f}m | Cross-track error: {cte:.2f}m")
                    logger.info(f"Steering: {steering_degrees:.1f}° | Speed: {v:.2f} m/s")
                    logger.info(f"Time step: {i}/{total_steps} ({i/total_steps*100:.1f}%)")
                    logger.info(f"===============================")
            
            # Small delay to make the replay feel real-time
            time.sleep(replay_interval)
        
        # Close trajectory file
        trajectory_file.close()
        logger.info("Replay completed")
    
    # Plot results
    plot_results(results, waypoints)
    
    # Create animation if requested
    if '--animate' in sys.argv:
        create_animation(results, waypoints)
    
    logger.info("Simulation completed")

def run_hardware(waypoints_file, config=None):
    """Run the rover using real hardware"""
    # Default configuration
    if config is None:
        config = {
            'max_steer': np.radians(90),   # Maximum steering angle
            'target_speed': 0.30,          # Target speed (m/s)
            'update_rate': 10,             # Control loop update rate (Hz)
            'extension_distance': 1.0,     # Extension distance beyond final waypoint
            'waypoint_reach_threshold': 1.0,  # Distance threshold to consider waypoint reached
            'steering_sensitivity': np.pi/3   # Denominator for tanh function
        }
    
    try:
        # CRITICAL FIX: Make sure waypoints_file exists and is the absolute path
        if not os.path.isabs(waypoints_file):
            # Check current directory
            if os.path.exists(waypoints_file):
                waypoints_file = os.path.abspath(waypoints_file)
            else:
                # Try data subdirectory
                data_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", waypoints_file)
                if os.path.exists(data_path):
                    waypoints_file = data_path
                else:
                    # Try parent's data directory
                    parent_data_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", waypoints_file)
                    if os.path.exists(parent_data_path):
                        waypoints_file = parent_data_path
        
        # CRITICAL: Check if the file exists now
        if not os.path.exists(waypoints_file):
            logger.critical(f"ERROR: Waypoints file {waypoints_file} not found!")
            logger.critical("Please specify an existing waypoints file with --waypoints.")
            return
        
        logger.critical(f"Using waypoints file: {waypoints_file}")
        
        # Load waypoints directly from the file to guarantee it works
        with open(waypoints_file, 'r') as f:
            logger.critical(f"Successfully opened waypoints file")
            raw_data = f.read()
            logger.critical(f"Raw file content (first 100 chars): {raw_data[:100]}...")
            
            data = json.loads(raw_data)
            if isinstance(data, list) and len(data) > 0:
                # Directly load waypoints based on format
                if isinstance(data[0], list):  # Simple format [[lat, lon], ...]
                    waypoints = np.array(data, dtype=np.float64)
                elif isinstance(data[0], dict) and 'lat' in data[0] and 'lon' in data[0]:
                    waypoints = np.array([[point['lat'], point['lon']] for point in data], dtype=np.float64)
                else:
                    logger.critical(f"Unknown waypoint format: {data[0]}")
                    return
            else:
                logger.critical("Waypoints file doesn't contain an array or is empty.")
                return
        
        # Check loaded waypoints
        logger.critical("==== LOADED WAYPOINTS ====")
        for i, wp in enumerate(waypoints):
            logger.critical(f"WP {i}: ({wp[0]:.7f}, {wp[1]:.7f})")
        logger.critical("========================")
        
        # Create controller AFTER successfully loading waypoints
        controller = StanleyController(
            waypoints=waypoints,
            max_steer=config['max_steer'],
            waypoint_reach_threshold=config['waypoint_reach_threshold'],
            steering_sensitivity=config['steering_sensitivity']
        )
        
        # VERIFY controller's waypoints are correct
        logger.critical("==== CONTROLLER WAYPOINTS ====")
        for i in range(len(controller.waypoints)):
            logger.critical(f"WP {i}: ({controller.waypoints[i][0]:.7f}, {controller.waypoints[i][1]:.7f})")
        logger.critical("========================")
        
        # Initialize GPS and motor controller
        gps = GPSMonitor()
        motors = get_motor_controller(max_speed=0.3)  # Set to 0.3 explicitly

        # IMPORTANT: Add debugging to verify it took effect
        logger.critical(f"MOTOR CONTROLLER MAX SPEED: {motors.max_speed}")
        
        # Constants
        update_interval = 1.0 / config['update_rate']
        last_update_time = 0
        
        # Create trajectory file
        trajectory_file = open(f"trajectory_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "w")
        trajectory_file.write("timestamp,lat,lon,heading,speed,target_idx,distance,cte\n")
        
        logger.info("Starting hardware control loop. Press Ctrl+C to stop.")
        
        try:
            while True:
                current_time = time.time()
                
                # Check if it's time for an update
                if current_time - last_update_time >= update_interval:
                    last_update_time = current_time
                    
                    try:
                        # Add this debug call before getting steering command
                        debug_heading(gps, controller, waypoints)
                        
                        # Get GPS data
                        lat, lon, heading, speed = gps.get_position_and_heading()
                        
                        if lat is not None and lon is not None and heading is not None:
                            # DEBUG - Print the target coordinates and the bearing calculation steps
                            logger.warning("=== COORDINATE DEBUG ===")
                            logger.warning(f"Current position: ({lat:.7f}, {lon:.7f}), Heading: {heading:.1f}°")
                            
                            # Get current target from controller
                            target_idx = controller.target_idx
                            if target_idx < len(waypoints):
                                tx, ty = waypoints[target_idx]
                                logger.warning(f"Target waypoint {target_idx}: ({tx:.7f}, {ty:.7f})")
                                
                                # Calculate bearing using our consistent method (same as map_visualizer)
                                import math
                                lat1_rad = math.radians(lat)
                                lon1_rad = math.radians(lon)
                                lat2_rad = math.radians(tx) 
                                lon2_rad = math.radians(ty)
                                
                                dlon = lon2_rad - lon1_rad
                                y_val = math.sin(dlon) * math.cos(lat2_rad)
                                x_val = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
                                bearing_rad = math.atan2(y_val, x_val)
                                bearing_deg = math.degrees(bearing_rad)
                                bearing_deg = (bearing_deg + 360) % 360
                                
                                logger.warning(f"DIRECT BEARING CALCULATION: {bearing_deg:.1f}°")
                                
                                # Compare with controller's calculation
                                controller_bearing = controller._calculate_bearing(lat, lon, tx, ty)
                                logger.warning(f"CONTROLLER BEARING CALCULATION: {controller_bearing:.1f}°")
                                
                                # Also check if parameter order matters
                                alt_bearing = controller._calculate_bearing(lat, lon, ty, tx)  # Swapped tx,ty
                                logger.warning(f"WITH SWAPPED TARGET COORDS: {alt_bearing:.1f}°")
                                
                                # Check with swapped current position
                                alt_bearing2 = controller._calculate_bearing(lon, lat, tx, ty)  # Swapped lat,lon
                                logger.warning(f"WITH SWAPPED CURRENT COORDS: {alt_bearing2:.1f}°")
                            
                            logger.warning("=== END DEBUG ===")
                        
                        # Then continue with the original code
                        yaw = np.radians(heading)
                        # CRITICAL FIX: Swap the lat/lon parameters when calling stanley_control
                        # The parameter expectation is (lon, lat, yaw, v) despite the x,y naming
                        delta, target_idx, distance, cte, yaw_error = controller.stanley_control(lon, lat, yaw, speed)
                        
                        # Limit steering angle
                        delta = np.clip(delta, -config['max_steer'], config['max_steer'])
                        
                        # Adjust speed based on steering angle (just calculate it, don't apply yet)
                        current_target_speed = config['target_speed']
                        if abs(delta) > np.radians(30):
                            current_target_speed *= 0.5  # Slow down for sharp turns
                        elif abs(delta) > np.radians(15):
                            current_target_speed *= 0.7  # Moderate speed for moderate turns

                        # Convert steering angle to degrees for the motor controller
                        steering_degrees = np.degrees(delta)

                        # Apply steering ONCE (and only once)
                        motors.set_steering(steering_degrees)
                        motors.set_heading_error(np.degrees(yaw_error))  # Store heading error for logging
                        
                        # IMPORTANT: Set the speed AFTER setting steering!
                        motors.set_speed(current_target_speed)  # THIS WAS MISSING! Apply the calculated speed
                        
                        # Get motor status for logging
                        motor_status = motors.get_status()
                        
                        # Write to trajectory file
                        trajectory_file.write(f"{current_time},{lat},{lon},{heading},"
                                             f"{speed},{target_idx},{distance},{cte}\n")
                        trajectory_file.flush()
                        
                        # Check if we've reached the final waypoint (which is the extension point)
                        if target_idx == len(waypoints) - 1 and distance < config['waypoint_reach_threshold']:
                            logger.info("✓✓ REACHED FINAL WAYPOINT")
                            motors.stop()
                            break
                            
                        if lat is not None and lon is not None and heading is not None:
                            # Calculate and show distance to target in a prominent way
                            current_target_idx = controller.target_idx
                            if current_target_idx < len(waypoints):
                                target_lat, target_lon = waypoints[current_target_idx]
                                distance_to_target = geodesic((lat, lon), (target_lat, target_lon)).meters
                                
                                # Create a visual distance indicator
                                max_display_distance = 20.0  # meters
                                display_dist = min(distance_to_target, max_display_distance)
                                bar_length = 20
                                filled = bar_length - int((display_dist / max_display_distance) * bar_length)
                                distance_bar = f"[{'#' * filled}{'-' * (bar_length - filled)}]"
                                
                                # Print prominent distance banner
                                logger.info("")
                                logger.info(f"DISTANCE TO TARGET: {distance_to_target:.2f}m {distance_bar}")
                                logger.info(f"WAYPOINT: {current_target_idx}/{len(waypoints)-1}")
                                logger.info("")
                        
                    except Exception as e:
                        logger.error(f"Error in control loop: {e}")
                        motors.stop()
                        time.sleep(1)  # Brief pause before retrying
                
                # Small delay to prevent CPU hogging
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            logger.info("Manual stop requested")
        
        finally:
            # Clean up
            logger.info("Stopping motors and cleaning up...")
            motors.stop()
            trajectory_file.close()
    except Exception as e:
        logger.error(f"Error in run_hardware: {e}")

# Add this helper function
def debug_heading(gps, controller, waypoints):
    """Debug function to verify heading calculations"""
    lat, lon, heading, speed = gps.get_position_and_heading()
    
    if controller.target_idx < len(waypoints):
        target_idx = controller.target_idx
        target = waypoints[target_idx]
        
        # Calculate bearing to target
        bearing = controller._calculate_bearing(lat, lon, target[0], target[1])
        
        # Calculate desired steering direction
        heading_rad = np.radians(heading)
        bearing_rad = np.radians(bearing)
        
        # Normalize the difference to [-pi, pi]
        diff_rad = ((bearing_rad - heading_rad + np.pi) % (2 * np.pi)) - np.pi
        diff_deg = np.degrees(diff_rad)
        
        # Get distance to target
        dist = geodesic((lat, lon), (target[0], target[1])).meters
        
        # Print debugging info with prominent waypoint number
        logger.info(f"DEBUG - TARGET WP: {target_idx} [{dist:.1f}m] | "
                   f"GPS Heading: {heading:.1f}°, "
                   f"Bearing to target: {bearing:.1f}°, "
                   f"Needed turn: {diff_deg:.1f}° {'RIGHT' if diff_deg > 0 else 'LEFT'}")

# Add this function to your main.py

def run_gps_test(waypoints_file, config=None):
    """Test GPS readings and bearing calculations matching map visualizer"""
    try:
        import math
        from geopy.distance import geodesic
        import json
        import os
        
        # Fix file path resolution
        if not os.path.exists(waypoints_file):
            # Try alternate locations
            possible_paths = [
                waypoints_file,
                os.path.join('data', waypoints_file),
                os.path.join('..', 'data', waypoints_file),
                os.path.join('..', '..', 'data', waypoints_file),
                os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', waypoints_file),
                os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', waypoints_file)
            ]
            
            for path in possible_paths:
                if os.path.exists(path):
                    waypoints_file = path
                    logger.info(f"Found waypoints file at: {path}")
                    break
                    
        if not os.path.exists(waypoints_file):
            logger.error(f"Could not find waypoints file: {waypoints_file}")
            return
        
        # Load waypoints exactly as the map visualizer does
        logger.info(f"Loading waypoints from {waypoints_file}")
        with open(waypoints_file, 'r') as f:
            waypoint_data = json.load(f)
        
        # Handle different JSON formats
        if isinstance(waypoint_data, list):
            # Direct array format
            if isinstance(waypoint_data[0], list):
                all_waypoints = waypoint_data
            # Object format with lat/lon
            elif isinstance(waypoint_data[0], dict) and 'lat' in waypoint_data[0]:
                all_waypoints = [[point['lat'], point['lon']] for point in waypoint_data]
            else:
                logger.error("Unknown waypoint format")
                return
        else:
            logger.error("Waypoint file doesn't contain an array")
            return
        
        # Ensure numpy array for consistent operation
        all_waypoints = np.array(all_waypoints)
        
        # Log loaded waypoints
        logger.info(f"Loaded {len(all_waypoints)} waypoints:")
        for i, wp in enumerate(all_waypoints):
            logger.info(f"WP {i}: ({wp[0]:.7f}, {wp[1]:.7f})")
        
        # Initialize GPS
        gps = GPSMonitor()
        
        # Constants
        update_interval = 1.0  # Update every 1 second
        last_update_time = 0
        
        # Calculate bearing EXACTLY as map_visualizer.py does
        def calculate_bearing(lat1, lon1, lat2, lon2):
            """Calculate bearing between two points exactly as in map_visualizer"""
            lat1_rad = math.radians(lat1)
            lon1_rad = math.radians(lon1)
            lat2_rad = math.radians(lat2)
            lon2_rad = math.radians(lon2)
            
            dlon = lon2_rad - lon1_rad
            y = math.sin(dlon) * math.cos(lat2_rad)
            x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
            bearing = math.atan2(y, x)  # Note: atan2(y,x) NOT atan2(x,y)
            
            bearing_deg = math.degrees(bearing)
            bearing_deg = (bearing_deg + 360) % 360
            return bearing_deg
        
        logger.info("Starting GPS test. Press Ctrl+C to exit.")
        
        while True:
            current_time = time.time()
            
            # Check if it's time to update
            if current_time - last_update_time >= update_interval:
                last_update_time = current_time
                
                # Get GPS data
                lat, lon, heading, speed = gps.get_position_and_heading()
                
                if lat is None or lon is None:
                    logger.warning("GPS position unavailable")
                    continue
                
                # Format output exactly like map_visualizer.py
                logger.info("\n=== WAYPOINT INFORMATION ===")
                logger.info(f"{'WP':<4}{'Distance (m)':<14}{'Bearing':<10}{'Turn'}")
                logger.info("-" * 40)
                
                for i, wp in enumerate(all_waypoints):
                    wp_lat, wp_lon = wp
                    dist = geodesic((lat, lon), (wp_lat, wp_lon)).meters
                    bearing = calculate_bearing(lat, lon, wp_lat, wp_lon)
                    
                    # Calculate turn direction if heading is available
                    turn_info = ""
                    if heading is not None:
                        relative_bearing = bearing - heading
                        relative_bearing = ((relative_bearing + 180) % 360) - 180  # Normalize to -180 to 180
                        
                        if abs(relative_bearing) < 1:
                            turn_direction = "↑"
                        elif relative_bearing > 0:
                            turn_direction = "→"
                        else:
                            turn_direction = "←"
                            
                        turn_info = f"{turn_direction} {abs(relative_bearing):.1f}°"
                    
                    logger.info(f"{i:<4}{dist:<14.1f}{bearing:<10.1f}°{turn_info}")
                
                logger.info("=" * 40)
            
            # Small delay to prevent CPU hogging
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        logger.info("GPS test stopped by user")
    except Exception as e:
        logger.error(f"Error in GPS test: {e}")
        import traceback
        logger.error(traceback.format_exc())

# Add helper function for bearing calculation
def calculate_bearing(lat1, lon1, lat2, lon2):
    """Calculate bearing between two points in degrees (0-360)"""
    import math
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.atan2(x, y)
    
    return (math.degrees(bearing) + 360) % 360


# Modify your main function to include the new GPS test mode
def main():
    """Main function to parse arguments and run the appropriate mode"""
    parser = argparse.ArgumentParser(description='Autonomous Rover Controller')
    parser.add_argument('--sim', action='store_true', help='Run in simulation mode')
    parser.add_argument('--real', action='store_true', help='Run with real hardware')
    parser.add_argument('--gps-test', action='store_true', help='Run in GPS test mode (no motors)')
    parser.add_argument('--waypoints', type=str, default='polygon_data.json',
                        help='JSON file containing waypoints')
    parser.add_argument('--config', type=str, help='JSON file containing configuration')
    parser.add_argument('--animate', action='store_true', help='Create animation in simulation mode')
    parser.add_argument('--replay', action='store_true', help='Replay simulation with progress logging')
    
    args = parser.parse_args()
    
    # Load configuration if provided
    config = None
    if args.config:
        try:
            with open(args.config) as f:
                config = json.load(f)
        except Exception as e:
            logger.error(f"Failed to load configuration: {e}")
            sys.exit(1)
    
    # Determine mode
    if args.sim and args.real:
        logger.error("Cannot specify both --sim and --real modes")
        sys.exit(1)
    elif args.sim:
        logger.info("Starting in SIMULATION mode")
        run_simulation(args.waypoints, config)
    elif args.real:
        logger.info("Starting in REAL HARDWARE mode")
        run_hardware(args.waypoints, config)
    elif args.gps_test:
        logger.info("Starting in GPS TEST mode")
        run_gps_test(args.waypoints, config)
    else:
        logger.error("Must specify either --sim, --real, or --gps-test mode")
        parser.print_help()
        sys.exit(1)

if __name__ == "__main__":
    main()