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
    """Load waypoints from a JSON file"""
    try:
        # Construct absolute path to the data directory
        if not os.path.isabs(waypoints_file):
            # Get the directory where the script is located
            script_dir = os.path.dirname(os.path.abspath(__file__))
            # Go up one level to the project root
            project_root = os.path.dirname(script_dir)
            # Construct the full path to the waypoints file
            full_path = os.path.join(project_root, "data", waypoints_file)
        else:
            full_path = waypoints_file
            
        logger.info(f"Loading waypoints from: {full_path}")
        
        with open(full_path) as f:
            data = json.load(f)
        
        # Extract coordinates from data
        waypoints = np.array([(point['lat'], point['lon']) for point in data])
        logger.info(f"Loaded {len(waypoints)} waypoints from {waypoints_file}")
        return waypoints
    except Exception as e:
        logger.error(f"Failed to load waypoints: {e}")
        sys.exit(1)

def run_simulation(waypoints_file, config=None):
    """Run the rover in simulation mode"""
    if not MATPLOTLIB_AVAILABLE:
        logger.error("Matplotlib is required for simulation mode")
        return
    
    # Default configuration
    if config is None:
        config = {
            'k': 15.8,                      # Stanley gain for heading error
            'k_e': 13.0,                   # Stanley gain for cross-track error
            'dt': 0.1,                     # Time step (seconds)
            'L': 1.0,                      # Wheelbase (meters)
            'max_steer': np.radians(90),   # Maximum steering angle
            'target_speed': 1.0,           # Target speed (m/s)
            'extension_distance': 1.0,     # Extension distance beyond final waypoint
            'waypoint_reach_threshold': 1.0  # Distance threshold to consider waypoint reached
        }
    
    # Load waypoints
    waypoints = load_waypoints(waypoints_file)
    
    # Add extension waypoint
    controller = StanleyController(
        waypoints, 
        k=config['k'], 
        k_e=config['k_e'], 
        L=config['L'], 
        max_steer=config['max_steer'],
        waypoint_reach_threshold=config['waypoint_reach_threshold']
    )
    waypoints = controller.add_extension_waypoint(config['extension_distance'])
    
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
        k=config['k'], 
        k_e=config['k_e'],
        target_speed=config['target_speed'],
        dt=config['dt'],
        L=config['L'],
        max_steer=config['max_steer'],
        waypoint_reach_threshold=config['waypoint_reach_threshold'],
        METERS_PER_LAT_DEGREE=METERS_PER_LAT_DEGREE
    )
    
    # Plot results
    plot_results(results, waypoints)
    
    # Create animation if requested
    if '--animate' in sys.argv:
        create_animation(results, waypoints)
    
    logger.info("Simulation completed")

def run_hardware(waypoints_file, config=None):
    """Run the rover with real hardware"""
    logger.info("Starting in REAL HARDWARE mode")
    
    # Default configuration
    if config is None:
        config = {
            'k': 6.8,                      # Stanley gain for heading error
            'k_e': 13.0,                   # Stanley gain for cross-track error
            'dt': 0.1,                     # Time step (seconds)
            'L': 1.0,                      # Wheelbase (meters)
            'max_steer': np.radians(50),   # Maximum steering angle
            'target_speed': 1.0,           # Target speed (m/s)
            'extension_distance': 1.0,     # Extension distance beyond final waypoint
            'waypoint_reach_threshold': 1.0,  # Distance threshold to consider waypoint reached
            'update_rate': 10.0            # Control loop update rate (Hz)
        }
    
    # Load waypoints
    waypoints = load_waypoints(waypoints_file)
    
    # Add extension waypoint
    controller = StanleyController(
        waypoints, 
        k=config['k'], 
        k_e=config['k_e'], 
        L=config['L'], 
        max_steer=config['max_steer'],
        waypoint_reach_threshold=config['waypoint_reach_threshold']
    )
    waypoints = controller.add_extension_waypoint(config['extension_distance'])
    
    # Import hardware interfaces
    try:
        from hardware.gps_monitor import GPSMonitor
        from hardware.motor_controller import MotorController
    except ImportError as e:
        logger.error(f"Failed to import hardware modules: {e}")
        logger.error("Make sure the hardware modules are in the Python path")
        return
    
    # Initialize hardware
    try:
        # Initialize GPS with simulation mode EXPLICITLY OFF
        gps = GPSMonitor(simulation_mode=False)  # <-- Force hardware mode
        motors = MotorController()
        
        logger.info("Hardware initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize hardware: {e}")
        return
    
    # Wait for valid GPS signal
    logger.info("Waiting for valid GPS position...")
    position_valid = False
    
    while not position_valid:
        try:
            lat, lon, heading, speed = gps.get_position_and_heading()
            if lat is not None and lon is not None and heading is not None:
                position_valid = True
                logger.info(f"Valid GPS position obtained: {lat}, {lon}, heading: {heading}°")
            else:
                logger.warning("No valid GPS position yet, retrying...")
                time.sleep(1)
        except Exception as e:
            logger.error(f"Error getting GPS position: {e}")
            time.sleep(1)
    
    # Initial state
    x = lat
    y = lon
    yaw = np.radians(heading)  # Convert heading from degrees to radians
    v = speed
    
    # Control loop
    target_idx = 1  # Start with second waypoint as target
    last_update_time = time.time()
    update_interval = 1.0 / config['update_rate']
    
    try:
        logger.info("Starting control loop...")
        
        # Constants for geographic coordinate conversion
        METERS_PER_LAT_DEGREE = 111000
        
        # Create log file for trajectory
        trajectory_file = open(f"trajectory_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "w")
        trajectory_file.write("timestamp,lat,lon,heading,speed,target_idx,distance,cte\n")
        
        # Add this at the beginning of the run_hardware control loop
        # (after creating motors but before the control loop starts)
        # This will help us see where set_speed is being called

        import traceback

        # Monkey patch the set_speed method to add more detailed logging
        original_set_speed = motors.set_speed

        def set_speed_with_trace(speed):
            stack = traceback.extract_stack()
            caller = stack[-2]  # The function that called this one
            logger.info(f"set_speed({speed}) called from {os.path.basename(caller.filename)}:{caller.lineno}")
            return original_set_speed(speed)

        motors.set_speed = set_speed_with_trace
        
        # Control loop
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
                    
                    # NO CHANGE TO HEADING - this is critical
                    x = lat
                    y = lon
                    yaw = np.radians(heading)  # Convert to radians without changing coordinate system
                    v = speed
                    
                    # Get steering command from Stanley controller
                    delta, target_idx, distance, cte, yaw_error = controller.stanley_control(x, y, yaw, v)
                    
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
                    
                    # Get motor status for logging
                    motor_status = motors.get_status()
                    
                    # Enhanced logging with clear heading information
                    logger.info(f"Pos: ({lat:.6f}, {lon:.6f}), Heading: {heading:.1f}°, "
                                f"Target: {target_idx}/{len(waypoints)-1}, "
                                f"Distance: {distance:.1f}m, CTE: {cte:.2f}m, "
                                f"Heading Error: {np.degrees(yaw_error):.1f}°, "
                                f"Steering: {np.degrees(delta):.1f}°")
                    
                    # Write to trajectory file
                    trajectory_file.write(f"{current_time},{lat},{lon},{heading},"
                                         f"{speed},{target_idx},{distance},{cte}\n")
                    trajectory_file.flush()
                    
                    # Check if we've reached the final waypoint (which is the extension point)
                    if target_idx == len(waypoints) - 1 and distance < config['waypoint_reach_threshold']:
                        logger.info("✓✓ REACHED FINAL WAYPOINT")
                        motors.stop()
                        break
                        
                except Exception as e:
                    logger.error(f"Error in control loop: {e}")
                    motors.stop()
                    time.sleep(1)  # Brief pause before retrying
            
            # Small delay to prevent CPU hogging
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        logger.info("Control loop interrupted by user")
    finally:
        # Clean up
        logger.info("Stopping motors and cleaning up...")
        motors.stop()
        trajectory_file.close()

# Add this helper function
def debug_heading(gps, controller, waypoints):
    """Debug function to verify heading calculations"""
    lat, lon, heading, speed = gps.get_position_and_heading()
    
    if controller.target_idx < len(waypoints):
        target = waypoints[controller.target_idx]
        
        # Calculate bearing to target
        bearing = controller._calculate_bearing(lat, lon, target[0], target[1])
        
        # Calculate desired steering direction
        heading_rad = np.radians(heading)
        bearing_rad = np.radians(bearing)
        
        # Normalize the difference to [-pi, pi]
        diff_rad = ((bearing_rad - heading_rad + np.pi) % (2 * np.pi)) - np.pi
        diff_deg = np.degrees(diff_rad)
        
        # Print debugging info
        logger.info(f"DEBUG - GPS Heading: {heading:.1f}°, "
                   f"Bearing to target: {bearing:.1f}°, "
                   f"Needed turn: {diff_deg:.1f}° {'RIGHT' if diff_deg > 0 else 'LEFT'}")

# Add this function to your main.py

def run_gps_test(waypoints_file, config=None):
    """
    Run in GPS test mode - motors stay at 0, shows all GPS data
    """
    logger.info("Starting in GPS TEST mode (no motors)")
    
    # Load waypoints
    waypoints = load_waypoints(waypoints_file)
    logger.info(f"Loaded {len(waypoints)} waypoints from {os.path.basename(waypoints_file)}")
    
    # Initialize hardware but don't activate motors
    try:
        # Get GPS
        from hardware.gps_monitor import GPSMonitor
        gps = GPSMonitor(simulation_mode='--sim' in sys.argv)
        
        # Initialize motors but don't use them
        from hardware.motor_controller import MotorController
        motors = MotorController()
        motors.set_speed(0)  # Ensure motors are stopped
        
        logger.info("Hardware initialized for testing")
        
        # Wait for valid GPS position
        logger.info("Waiting for valid GPS position...")
        while True:
            lat, lon, heading, speed = gps.get_position_and_heading()
            if lat is not None and lon is not None:
                logger.info(f"Valid GPS position obtained: {lat}, {lon}, heading: {heading}°")
                break
            time.sleep(0.5)
        
        # Set up for heading calculations
        current_waypoint = 0
        
        # Main test loop
        logger.info("Starting GPS monitoring. Press Ctrl+C to exit...")
        
        # For calculating distance moved
        initial_lat, initial_lon = lat, lon
        last_lat, last_lon = lat, lon
        total_distance = 0
        last_update_time = time.time()
        update_interval = 0.1  # 10Hz updates
        
        # For calculating heading from movement
        positions = []
        movement_bearing = None
        
        while True:
            current_time = time.time()
            
            # Update at fixed interval
            if current_time - last_update_time >= update_interval:
                last_update_time = current_time
                
                # Get GPS data
                lat, lon, heading, speed = gps.get_position_and_heading()
                
                if lat is None or lon is None:
                    logger.warning("GPS position unavailable")
                    continue
                
                # Store position for movement tracking (max 10 recent positions)
                positions.append((lat, lon, current_time))
                if len(positions) > 10:
                    positions.pop(0)
                
                # Calculate distance from start
                from_start = geodesic((initial_lat, initial_lon), (lat, lon)).meters
                
                # Calculate distance from last point
                from_last = geodesic((last_lat, last_lon), (lat, lon)).meters
                total_distance += from_last
                
                # Update last position
                last_lat, last_lon = lat, lon
                
                # Calculate heading to each waypoint
                waypoint_info = []
                for i, wp in enumerate(waypoints):
                    # Calculate bearing to this waypoint
                    bear = calculate_bearing(lat, lon, wp[0], wp[1])
                    
                    # Calculate distance to this waypoint
                    dist = geodesic((lat, lon), (wp[0], wp[1])).meters
                    
                    # Calculate what heading error would be for this waypoint
                    heading_diff = ((bear - heading + 180) % 360) - 180
                    
                    waypoint_info.append({
                        'index': i,
                        'bearing': bear,
                        'distance': dist,
                        'heading_error': heading_diff
                    })
                
                # Calculate heading from movement if we've moved enough
                if len(positions) >= 2:
                    # Use last 2 positions that are at least 10cm apart
                    pos_idx = -1
                    for i in range(len(positions) - 2, -1, -1):
                        dist = geodesic(
                            (positions[i][0], positions[i][1]), 
                            (positions[-1][0], positions[-1][1])
                        ).meters
                        if dist > 0.1:  # 10cm threshold
                            pos_idx = i
                            break
                    
                    if pos_idx >= 0:
                        old_lat, old_lon, _ = positions[pos_idx]
                        movement_bearing = calculate_bearing(old_lat, old_lon, lat, lon)
                
                # Print GPS information
                logger.info(f"\n===== GPS DIAGNOSTICS =====")
                logger.info(f"Position: {lat:.8f}, {lon:.8f}")
                logger.info(f"GPS Heading: {heading:.1f}° | Speed: {speed:.2f} m/s")
                if movement_bearing is not None:
                    logger.info(f"Movement Heading: {movement_bearing:.1f}° | "
                               f"Diff from GPS: {((movement_bearing - heading + 180) % 360 - 180):.1f}°")
                logger.info(f"Distance - From start: {from_start:.2f}m | Total: {total_distance:.2f}m")
                
                # Print waypoint information
                logger.info(f"\n===== WAYPOINT INFO =====")
                if current_waypoint < len(waypoints):
                    wp = waypoint_info[current_waypoint]
                    logger.info(f"Current target - WP {wp['index']}: "
                               f"Bearing: {wp['bearing']:.1f}°, "
                               f"Distance: {wp['distance']:.2f}m, "
                               f"Heading Error: {wp['heading_error']:.1f}°")
                    
                    logger.info(f"To steer correctly: "
                               f"{'RIGHT' if wp['heading_error'] > 0 else 'LEFT'} "
                               f"by {abs(wp['heading_error']):.1f}°")
                
                # Print nearest waypoint
                nearest_wp = min(waypoint_info, key=lambda w: w['distance'])
                logger.info(f"Nearest waypoint: WP {nearest_wp['index']} - {nearest_wp['distance']:.2f}m away")
                
                # Print all waypoints in compact form
                logger.info("\nAll Waypoints:")
                for wp in waypoint_info:
                    logger.info(f"WP {wp['index']}: {wp['distance']:.1f}m at {wp['bearing']:.1f}° "
                               f"(error: {wp['heading_error']:+.1f}°)")
                
                # Divider for readability
                logger.info("\n" + "=" * 30)
            
            # Small delay to prevent CPU hogging
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        logger.info("GPS test mode stopped by user")
    except Exception as e:
        logger.error(f"Error in GPS test mode: {e}")
        import traceback
        logger.error(traceback.format_exc())
    finally:
        # Clean up
        if 'gps' in locals():
            gps.stop()
        if 'motors' in locals():
            motors.set_speed(0)  # Ensure motors are stopped
        logger.info("GPS test mode terminated")


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