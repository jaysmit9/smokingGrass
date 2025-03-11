import numpy as np
import json
from geopy.distance import geodesic
from hardware.gps_monitor import GPSMonitor
from hardware.motor_controller import MotorController
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class StanleyController:
    def __init__(self, waypoints, max_steer=np.radians(50), waypoint_reach_threshold=1.0, steering_sensitivity=np.pi/3):
        """
        Initialize the Stanley Controller with simplified tanh-based steering.
        
        Args:
            waypoints: Array of waypoint coordinates (lat, lon)
            max_steer: Maximum steering angle in radians
            waypoint_reach_threshold: Distance in meters to consider a waypoint reached
            steering_sensitivity: Denominator for tanh function (lower = more aggressive)
        """
        self.waypoints = waypoints
        self.max_steer = max_steer
        self.waypoint_reach_threshold = waypoint_reach_threshold
        self.steering_sensitivity = steering_sensitivity  # New parameter for tanh sensitivity
        self.target_idx = 1  # Initial target waypoint index
        self.curr_nearest_point_idx = 0
        self.gps_monitor = GPSMonitor()
        self.motor_controller = MotorController()
        
        logger.info(f"Stanley Controller initialized with {len(waypoints)} waypoints")
        logger.info(f"Steering sensitivity: {np.degrees(steering_sensitivity):.1f}° (tanh denominator)")
        logger.info(f"Max steering angle: {np.degrees(max_steer):.1f}°")
        logger.info(f"Waypoint reach threshold: {waypoint_reach_threshold} meters")

    def add_extension_waypoint(self, extension_distance=1.0):
        """Add an extension waypoint beyond the final waypoint"""
        if len(self.waypoints) < 2:
            return self.waypoints
        
        final_waypoint = self.waypoints[-1]
        second_last_waypoint = self.waypoints[-2]
        
        lat_diff = final_waypoint[0] - second_last_waypoint[0]
        lon_diff = final_waypoint[1] - second_last_waypoint[1]
        
        distance = self.haversine_distance(
            (second_last_waypoint[0], second_last_waypoint[1]),
            (final_waypoint[0], final_waypoint[1])
        )
        
        if distance > 0:
            lat_diff /= distance
            lon_diff /= distance
        else:
            lat_diff = 1.0
            lon_diff = 0.0
        
        METERS_PER_LAT_DEGREE = 111000
        lat_change_per_meter = 1.0 / METERS_PER_LAT_DEGREE
        lon_change_per_meter = 1.0 / (METERS_PER_LAT_DEGREE * np.cos(np.radians(final_waypoint[0])))
        
        extension_lat = final_waypoint[0] + lat_diff * extension_distance * lat_change_per_meter
        extension_lon = final_waypoint[1] + lon_diff * extension_distance * lon_change_per_meter
        
        extended_waypoints = np.copy(self.waypoints)
        extended_waypoints = np.vstack([extended_waypoints, [extension_lat, extension_lon]])
        
        logger.info(f"Added extension waypoint at ({extension_lat:.6f}, {extension_lon:.6f})")
        return extended_waypoints

    def haversine_distance(self, coord1, coord2):
        """Calculate the great circle distance between two points in meters"""
        return geodesic(coord1, coord2).meters

    def control_loop(self):
        """Main control loop for autonomous navigation"""
        while True:
            current_position = self.gps_monitor.get_position()
            x, y, yaw, v = current_position
            
            delta, self.target_idx, distance, cte, yaw_error = self.stanley_control(x, y, yaw, v)
            
            # Log important metrics
            logger.debug(f"Target waypoint: {self.target_idx}, Distance: {distance:.2f}m")
            logger.debug(f"Heading error: {np.degrees(yaw_error):.1f}°, Steering: {np.degrees(delta):.1f}°")
            
            # Apply control
            self.motor_controller.set_steering(np.degrees(delta))  # Convert to degrees for motor controller
            self.motor_controller.set_speed(v)

    def stanley_control(self, x, y, yaw, v):
        """
        Simplified Stanley steering control using tanh function.
        
        IMPORTANT: The parameters are:
          x = longitude (not latitude)
          y = latitude (not longitude)
          yaw = heading in radians
          v = velocity
        
        This ordering is counterintuitive but must be preserved for compatibility.
        """
        # Current target waypoint - waypoints are stored as [lat, lon]
        tx = self.waypoints[self.target_idx, 0]  # lat
        ty = self.waypoints[self.target_idx, 1]  # lon
        
        # Calculate distance to current target
        dist_to_target = np.sqrt((tx - y) ** 2 + (ty - x) ** 2)  # Note y=lat, x=lon here!
        
        # Calculate bearing correctly with swapped parameters
        # _calculate_bearing expects (lat1, lon1, lat2, lon2)
        target_bearing = self._calculate_bearing(y, x, tx, ty)  # y=lat, x=lon
        
        # Current heading in degrees
        current_heading = np.degrees(yaw)
        
        # Calculate heading error
        heading_error_deg = target_bearing - current_heading
        
        # Normalize to -180 to +180
        heading_error_deg = ((heading_error_deg + 180) % 360) - 180
        
        # Debug info with bearing and heading
        logger.debug(f"Position: ({x:.6f}, {y:.6f}), Heading: {current_heading:.1f}°")
        logger.debug(f"Target: ({tx:.6f}, {ty:.6f}), Bearing: {target_bearing:.1f}°, Error: {heading_error_deg:.1f}°")
        
        # Convert back to radians for steering calculation
        heading_error = np.radians(heading_error_deg)
        
        # SIMPLIFIED STEERING WITH TANH
        steering_angle = np.tanh(heading_error / self.steering_sensitivity)
        
        # Scale to max_steer for steering control
        delta = steering_angle * self.max_steer
        
        logger.debug(f"Final steering angle: {np.degrees(delta):.1f}°")
        
        return delta, self.target_idx, dist_to_target, 0.0, heading_error

    def _normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def _calculate_bearing(self, lat1, lon1, lat2, lon2):
        """
        Calculate bearing between two points, matching the map visualizer exactly.
        Returns bearing in degrees (0-360).
        """
        import math
        # Convert to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Calculate bearing using the EXACT same formula as map_visualizer and GPS test
        dlon = lon2_rad - lon1_rad
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        bearing_rad = math.atan2(y, x)  # Critical: atan2(y,x) NOT atan2(x,y)
        
        # Convert to degrees and normalize to 0-360
        bearing_deg = math.degrees(bearing_rad)
        bearing_deg = (bearing_deg + 360) % 360
        
        # Add this line to debug bearing calculations
        import logging
        logger = logging.getLogger("rover")
        logger.debug(f"BEARING CHECK: ({lat1:.6f},{lon1:.6f}) → ({lat2:.6f},{lon2:.6f}) = {bearing_deg:.1f}°")
        
        return bearing_deg

def run_gps_test(waypoints_file, config=None):
    """Test GPS readings and bearing calculations"""
    try:
        # Load waypoints
        all_waypoints = load_waypoints(waypoints_file)
        
        # Initialize GPS
        gps = GPSMonitor()
        
        # Set up variables for tracking position
        initial_lat = None
        initial_lon = None
        last_lat = None
        last_lon = None
        positions = []
        total_distance = 0.0
        
        # Constants
        update_interval = 1.0  # Update every 1 second
        last_update_time = 0
        
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
                
                # Store position for movement tracking (max 10 recent positions)
                positions.append((lat, lon, current_time))
                if len(positions) > 10:
                    positions.pop(0)
                
                # Set initial position if not set
                if initial_lat is None:
                    initial_lat = lat
                    initial_lon = lon
                    logger.info(f"Initial position: {lat:.6f}, {lon:.6f}")
                
                # Calculate distance from start (if we have initial position)
                if initial_lat is not None and initial_lon is not None:
                    from_start = geodesic((initial_lat, initial_lon), (lat, lon)).meters
                    logger.info(f"Distance from start: {from_start:.1f}m")
                
                # Calculate distance from last point (if we have a previous position)
                if last_lat is not None and last_lon is not None:
                    from_last = geodesic((last_lat, last_lon), (lat, lon)).meters
                    total_distance += from_last
                    logger.info(f"Distance from last point: {from_last:.1f}m")
                    logger.info(f"Total distance traveled: {total_distance:.1f}m")
                
                # Update last position
                last_lat, last_lon = lat, lon
                
                # Calculate heading to each waypoint
                # Only do this if we have a valid heading
                if heading is not None:
                    logger.info(f"Current heading: {heading:.1f}°")
                    logger.info("All Waypoints:")
                    for i, wp in enumerate(all_waypoints):
                        wp_lat, wp_lon = wp
                        dist = geodesic((lat, lon), (wp_lat, wp_lon)).meters
                        bear = _calculate_bearing(lat, lon, wp_lat, wp_lon)
                        
                        # Calculate heading difference
                        heading_diff = ((bear - heading + 180) % 360) - 180
                        
                        logger.info(f"WP {i}: {dist:.1f}m at {bear:.1f}° (error: {'+' if heading_diff > 0 else ''}{heading_diff:.1f}°)")
                else:
                    logger.warning("GPS heading unavailable - waiting for valid heading")
            
            # Small delay to prevent CPU hogging
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        logger.info("GPS test stopped by user")
    except Exception as e:
        logger.error(f"Error in GPS test: {e}")
        logger.error(traceback.format_exc())

def main():
    """Test function for the controller"""
    with open('data/polygon_data.json') as f:
        polygon_data = json.load(f)
    
    waypoints = np.array([(point['lat'], point['lon']) for point in polygon_data])
    controller = StanleyController(waypoints)
    
    controller.control_loop()

if __name__ == "__main__":
    main()