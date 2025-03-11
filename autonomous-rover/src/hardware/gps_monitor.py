import time
import math
import threading
import logging
import random
import os
import numpy as np
from geopy.distance import geodesic
import json

logger = logging.getLogger("rover.hardware.gps")

class GPSMonitor:
    def __init__(self, main_port='/dev/ttyACM0', secondary_port='/dev/ttyACM1', 
                 baud_rate=115200, timeout=0.1, use_dual_gps=True, simulation_mode=False):
        """Initialize GPS monitor"""
        # Add thread lock for all instances
        self.lock = threading.RLock()
        
        self.simulation_mode = simulation_mode
        self.use_dual_gps = use_dual_gps
        self.timeout = timeout
        self.L = 1.0  # Wheelbase length in meters
        
        # For simulation calculations
        self.lat_scale = 1.0 / 111000  # Approximate conversion (meters to degrees)
        self.lon_scale = 1.0 / 111000  # Will be refined based on latitude
        
        # Initialize simulation variables
        if simulation_mode:
            self._init_simulation()
        else:
            # Your existing hardware initialization code
            # For debugging purposes, print object ID to identify different instances
            logger.info(f"Creating GPSMonitor instance {id(self)} with simulation_mode={simulation_mode}")
            
            # GPS data storage (matching serial_gps_monitor.py structure)
            self.gps_data = {
                "front": {
                    "lat": None, "lon": None, "heading": None, 
                    "speed": None, "fix_quality": 0, "last_update": 0,
                    "messages_per_sec": 0, "message_count": 0, "last_message_time": time.time()
                },
                "rear": {
                    "lat": None, "lon": None, "heading": None, 
                    "speed": None, "fix_quality": 0, "last_update": 0,
                    "messages_per_sec": 0, "message_count": 0, "last_message_time": time.time()
                }
            }
            
            logger.info(f"Initializing GPS Monitor in {'SIMULATION' if simulation_mode else 'HARDWARE'} mode")
            
            if not self.simulation_mode:
                try:
                    import serial
                    self.main_serial = serial.Serial(main_port, baud_rate, timeout=timeout)
                    logger.info(f"Connected to main GPS on {main_port}")
                    
                    if use_dual_gps:
                        self.secondary_serial = serial.Serial(secondary_port, baud_rate, timeout=timeout)
                        logger.info(f"Connected to secondary GPS on {secondary_port}")
                    
                    # Start reader threads
                    self._start_hardware_readers()
                except Exception as e:
                    logger.error(f"Failed to initialize hardware GPS: {e}")
                    logger.info("Falling back to simulation mode")
                    self.simulation_mode = True
            
            if self.simulation_mode:
                self._start_simulation()
    
    def _init_simulation(self):
        """Initialize simulation parameters"""
        logger.info("Initializing GPS simulation")
        
        # Load default waypoints for simulation
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(os.path.dirname(script_dir))
        waypoints_file = os.path.join(project_root, "data", "polygon_data.json")
        
        try:
            with open(waypoints_file) as f:
                data = json.load(f)
            
            # Extract waypoints
            self.sim_waypoints = np.array([(point['lat'], point['lon']) for point in data])
            logger.info(f"Loaded {len(self.sim_waypoints)} waypoints for GPS simulation")
            
            # Fix: Use actual GPS coordinates for initial position
            self.sim_x = float(self.sim_waypoints[0][0])  # Cast to float to ensure numeric type
            self.sim_y = float(self.sim_waypoints[0][1])
            logger.info(f"Initial position set to: ({self.sim_x:.6f}, {self.sim_y:.6f})")
            
            self.sim_yaw = 0.0  # Initial heading (North)
            self.sim_speed = 0.0  # Initial speed
            
            # FIXED: Use amplified scaling factors for better simulation
            self.lat_scale = 1.0 / 11100  # 10x larger for faster simulation
            self.lon_scale = 1.0 / (11100 * np.cos(np.radians(self.sim_x)))  # 10x larger
            logger.info(f"Using amplified simulation scales: lat_scale={self.lat_scale:.8f}, lon_scale={self.lon_scale:.8f}")
            
        except Exception as e:
            logger.error(f"Failed to load simulation waypoints: {e}", exc_info=True)
            # Use actual GPS coordinates for default values
            self.sim_waypoints = np.array([[34.151977, -77.866983], [34.152077, -77.866783]])
            self.sim_x = 34.151977
            self.sim_y = -77.866983
            logger.info(f"Using default position: ({self.sim_x}, {self.sim_y})")
            self.sim_yaw = 0.0
            self.sim_speed = 0.0
            # FIXED: Use amplified scaling factors here too
            self.lat_scale = 1.0 / 11100  # 10x larger
            self.lon_scale = 1.0 / (11100 * np.cos(np.radians(self.sim_x)))  # 10x larger
    
    def _start_hardware_readers(self):
        """Start threads to read from real GPS hardware"""
        def gps_reader_thread(serial_port, gps_position):
            import serial
            try:
                buffer = ""
                while True:
                    try:
                        # Read available data
                        if hasattr(serial_port, 'in_waiting') and serial_port.in_waiting:
                            data = serial_port.read(serial_port.in_waiting)
                            
                            # Convert bytes to string
                            text = data.decode('ascii', errors='replace')
                            buffer += text
                            
                            # Process any complete sentences in the buffer
                            while '\n' in buffer:
                                idx = buffer.find('\n')
                                sentence = buffer[:idx].strip()
                                buffer = buffer[idx+1:]
                                
                                # Parse the sentence
                                if sentence:
                                    self._parse_nmea(sentence, gps_position)
                    
                    except UnicodeDecodeError:
                        buffer = ""  # Reset on bad data
                    except Exception as e:
                        logger.error(f"Error in {gps_position} GPS reader: {e}")
                        time.sleep(1)  # Avoid rapid error loops
                    
                    # Small delay to prevent CPU hogging
                    time.sleep(0.01)
                    
            except Exception as e:
                logger.error(f"GPS reader thread error: {e}")
        
        # Front GPS thread
        front_thread = threading.Thread(
            target=gps_reader_thread, 
            args=(self.main_serial, "front"),
            daemon=True
        )
        
        # Rear GPS thread (if used)
        if self.use_dual_gps:
            rear_thread = threading.Thread(
                target=gps_reader_thread, 
                args=(self.secondary_serial, "rear"),
                daemon=True
            )
            rear_thread.start()
        
        front_thread.start()
    
    def _parse_nmea(self, sentence, gps_position):
        """Parse NMEA sentences, matching serial_gps_monitor.py logic"""
        try:
            if not sentence.startswith('$'):
                return
                
            parts = sentence.split(',')
            sentence_type = parts[0]
            
            # Update message counter for rate calculation
            self.gps_data[gps_position]["message_count"] += 1
            self.gps_data[gps_position]["last_message_time"] = time.time()
            
            # Parse GGA sentence (position and fix quality)
            if sentence_type == "$GPGGA" or sentence_type == "$GNGGA":
                try:
                    # Extract fix quality
                    fix_quality = int(parts[6]) if parts[6] else 0
                    self.gps_data[gps_position]["fix_quality"] = fix_quality
                    
                    # Only update position if we have a fix
                    if fix_quality > 0:
                        # Extract latitude
                        if parts[2] and parts[3]:
                            lat_deg = float(parts[2][:2])
                            lat_min = float(parts[2][2:])
                            lat_decimal = lat_deg + (lat_min / 60.0)
                            if parts[3] == 'S':
                                lat_decimal = -lat_decimal
                            self.gps_data[gps_position]["lat"] = lat_decimal
                        
                        # Extract longitude
                        if parts[4] and parts[5]:
                            lon_deg = float(parts[4][:3])
                            lon_min = float(parts[4][3:])
                            lon_decimal = lon_deg + (lon_min / 60.0)
                            if parts[5] == 'W':
                                lon_decimal = -lon_decimal
                            self.gps_data[gps_position]["lon"] = lon_decimal
                        
                        self.gps_data[gps_position]["last_update"] = time.time()
                except Exception:
                    pass
            
            # Parse RMC sentence (position, speed, heading)
            elif sentence_type == "$GPRMC" or sentence_type == "$GNRMC":
                try:
                    # Check if data is valid
                    if parts[2] == 'A':  # A = valid, V = invalid
                        # Extract speed in knots, convert to m/s
                        if parts[7]:
                            speed_knots = float(parts[7])
                            speed_ms = speed_knots * 0.514444
                            self.gps_data[gps_position]["speed"] = speed_ms
                        
                        # Extract heading
                        if parts[8]:
                            heading = float(parts[8])
                            self.gps_data[gps_position]["heading"] = heading
                except Exception:
                    pass
                    
            # Parse VTG sentence (heading and speed)
            elif sentence_type == "$GPVTG" or sentence_type == "$GNVTG":
                try:
                    # Extract true heading
                    if parts[1]:
                        heading = float(parts[1])
                        self.gps_data[gps_position]["heading"] = heading
                    
                    # Extract speed (usually in km/h at parts[7])
                    if parts[7]:
                        speed_kmh = float(parts[7])
                        speed_ms = speed_kmh / 3.6
                        self.gps_data[gps_position]["speed"] = speed_ms
                except Exception:
                    pass
            
        except Exception as e:
            logger.error(f"Error parsing NMEA: {e}")
    
    def _start_simulation(self):
        """Start GPS simulation with realistic movement"""
        logger.info("Starting GPS simulation")
        
        # Try to load waypoints for realistic simulation path
        try:
            import json
            script_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.dirname(os.path.dirname(script_dir))
            waypoints_path = os.path.join(project_root, "data", "polygon_data.json")
            
            with open(waypoints_path) as f:
                waypoints_data = json.load(f)
                self.sim_waypoints = [(point['lat'], point['lon']) for point in waypoints_data]
                logger.info(f"Loaded {len(self.sim_waypoints)} waypoints for GPS simulation")
                
                # Set initial position to first waypoint
                if self.sim_waypoints:
                    self.gps_data["front"]["lat"] = self.sim_waypoints[0][0]
                    self.gps_data["front"]["lon"] = self.sim_waypoints[0][1]
                    self.gps_data["front"]["fix_quality"] = 4  # RTK fixed
                    
                    # Set rear GPS 30cm behind front in initial direction
                    if len(self.sim_waypoints) > 1:
                        # Calculate initial direction from first to second waypoint
                        wp1 = self.sim_waypoints[0]
                        wp2 = self.sim_waypoints[1]
                        
                        # Calculate heading
                        heading = self._calculate_bearing(wp1[0], wp1[1], wp2[0], wp2[1])
                        
                        # Convert heading to radians and reverse direction (for rear position)
                        heading_rad = math.radians((heading + 180) % 360)
                        
                        # Calculate rear GPS position (30cm behind front)
                        dist = 0.3  # 30cm in meters
                        METERS_PER_LAT_DEGREE = 111000
                        meters_per_lon_degree = METERS_PER_LAT_DEGREE * math.cos(math.radians(wp1[0]))
                        
                        lat_change = (dist * math.cos(heading_rad)) / METERS_PER_LAT_DEGREE
                        lon_change = (dist * math.sin(heading_rad)) / meters_per_lon_degree
                        
                        self.gps_data["rear"]["lat"] = wp1[0] + lat_change
                        self.gps_data["rear"]["lon"] = wp1[1] + lon_change
                        self.gps_data["rear"]["fix_quality"] = 4  # RTK fixed
                        
                        # Set initial heading for both GPS units
                        self.gps_data["front"]["heading"] = heading
                        self.gps_data["rear"]["heading"] = heading
        except Exception as e:
            logger.warning(f"Failed to load waypoints for simulation: {e}")
            # Fallback: Set some default position
            self.gps_data["front"]["lat"] = 34.15197707670105
            self.gps_data["front"]["lon"] = -77.86698266348658
            self.gps_data["front"]["fix_quality"] = 4  # RTK fixed
            
            # Generate rear GPS position (30cm behind)
            self.gps_data["rear"]["lat"] = self.gps_data["front"]["lat"] - 0.0000027  # ~30cm south
            self.gps_data["rear"]["lon"] = self.gps_data["front"]["lon"]
            self.gps_data["rear"]["fix_quality"] = 4  # RTK fixed
            
            # Set default heading
            self.gps_data["front"]["heading"] = 0  # North
            self.gps_data["rear"]["heading"] = 0
        
        # Initialize simulation control variables
        self.sim_target_idx = 1 if hasattr(self, 'sim_waypoints') and len(self.sim_waypoints) > 1 else 0
        self.sim_speed = 0.5  # m/s
        self.sim_turning = 0  # -1=left, 0=straight, 1=right
        
        # Start simulation thread
        self.sim_thread = threading.Thread(target=self._simulate_gps_updates)
        self.sim_thread.daemon = True
        self.sim_thread.start()
    
    def _simulate_gps_updates(self):
        """Simulate GPS updates based on speed and steering angle"""
        while self.simulation_running:
            try:
                time.sleep(self.simulation_update_interval)
                
                # Get current state
                current_speed = self.simulated_speed  # Meters per second
                current_steering_angle = self.simulated_steering_angle  # Radians
                
                # Calculate change in heading
                delta_heading = (current_speed / self.wheelbase) * np.tan(current_steering_angle) * self.simulation_update_interval
                
                # Update heading
                self.simulated_heading += np.degrees(delta_heading)
                self.simulated_heading %= 360  # Keep within 0-360 range
                
                # Calculate change in position (simplified for simulation)
                delta_x = current_speed * np.cos(np.radians(self.simulated_heading)) * self.simulation_update_interval
                delta_y = current_speed * np.sin(np.radians(self.simulated_heading)) * self.simulation_update_interval
                
                # IMPORTANT FIX: Use a more accurate scaling factor
                # Standard is 111111.0 meters per degree at the equator
                # But let's amplify this for better simulation progress
                SCALING_FACTOR = 11111.0  # 10x smaller = 10x faster movement
                
                # Update position (using enhanced increments in lat/lon)
                self.simulated_latitude += delta_y / SCALING_FACTOR  # Enhanced meters to latitude conversion
                self.simulated_longitude += (delta_x / (SCALING_FACTOR * np.cos(np.radians(self.simulated_latitude))))  # Enhanced meters to longitude
                
                # Log simulation update
                logger.debug(f"Simulated GPS update - Lat: {self.simulated_latitude:.8f}, Lon: {self.simulated_longitude:.8f}, Heading: {self.simulated_heading:.2f}")
                
            except Exception as e:
                logger.error(f"Error in GPS simulation: {e}")
    
    def _calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing between two points in degrees (0-360)"""
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.atan2(x, y)
        
        return (math.degrees(bearing) + 360) % 360
    
    def get_position_and_heading(self):
        """Get current position and heading (lat, lon, heading_deg, speed)"""
        if self.simulation_mode:
            # In simulation mode, just return simulation state
            heading_deg = np.degrees(self.sim_yaw) % 360
            return self.sim_x, self.sim_y, heading_deg, self.sim_speed
        else:
            # Use front GPS data for position
            lat = self.gps_data["front"]["lat"]
            lon = self.gps_data["front"]["lon"]
            
            # DIRECT IMPLEMENTATION: Always calculate heading from dual GPS if possible
            heading_deg = None  # Default to None
            
            # First attempt: Calculate from dual GPS positions (most accurate)
            if self.use_dual_gps and "rear" in self.gps_data:
                rear_lat = self.gps_data["rear"]["lat"]
                rear_lon = self.gps_data["rear"]["lon"]
                
                if (lat is not None and lon is not None and 
                    rear_lat is not None and rear_lon is not None):
                    # Calculate bearing from REAR to FRONT (vehicle heading)
                    heading_deg = self._calculate_bearing(rear_lat, rear_lon, lat, lon)
                    logger.debug(f"Dual GPS heading calculation: {heading_deg:.1f}°")
            
            # Second attempt: Use GPS-provided heading if dual calculation failed
            if heading_deg is None:
                heading_deg = self.gps_data["front"]["heading"]
                if heading_deg is not None:
                    logger.debug(f"Using direct GPS heading: {heading_deg:.1f}°")
                else:
                    heading_deg = 0.0
                    logger.warning("No heading data available, using 0.0°")
            
            # Get speed in m/s
            speed = self.gps_data["front"]["speed"] or 0.0
            
            return lat, lon, heading_deg, speed
    
    def stop(self):
        """Stop GPS monitoring"""
        if not self.simulation_mode:
            if hasattr(self, 'main_serial'):
                self.main_serial.close()
            if hasattr(self, 'secondary_serial') and self.use_dual_gps:
                self.secondary_serial.close()
    
    def update_simulation(self, steering_angle, speed, dt=0.1):
        """
        Update simulated position based on steering and speed
        Using a consistent bicycle model for simulation
        """
        # Ensure we're working with a copy of the state to prevent race conditions
        with self.lock:
            # Store previous state for logging
            prev_x = self.sim_x
            prev_y = self.sim_y
            prev_yaw = self.sim_yaw
            prev_heading_deg = np.degrees(prev_yaw) % 360
            
            # 1. Make sure steering_angle is in radians and limited
            steering_angle_rad = steering_angle
            if abs(steering_angle_rad) > np.pi/2:
                logger.warning(f"Very large steering angle detected: {np.degrees(steering_angle_rad):.1f}°, limiting to ±45°")
                steering_angle_rad = np.clip(steering_angle_rad, -np.pi/4, np.pi/4)
            
            # 2. Limit the yaw rate change for smoother simulation
            max_yaw_rate = np.radians(20)  # Max 20 degrees per update
            yaw_rate = (speed / self.L) * np.tan(steering_angle_rad)
            yaw_rate = np.clip(yaw_rate, -max_yaw_rate, max_yaw_rate)
            
            # 3. Update yaw (heading)
            self.sim_yaw += yaw_rate * dt
            self.sim_yaw = self.sim_yaw % (2 * np.pi)  # Keep in [0, 2π]
            
            # 4. Calculate position change
            # Using standard navigation coordinate system:
            # 0° = North, 90° = East, 180° = South, 270° = West
            heading_rad = self.sim_yaw
            delta_x = speed * np.sin(heading_rad) * dt  # East-West
            delta_y = speed * np.cos(heading_rad) * dt  # North-South
            
            # 5. Apply a scaling factor to make simulation progress visible
            # Standard conversion: 111,111 meters per degree latitude at equator
            # Using enhanced factor (10x) for better simulation visibility
            lat_scale = 1.0 / 11111.0  # 10x larger than real world
            lon_scale = 1.0 / (11111.0 * np.cos(np.radians(self.sim_x)))
            
            # 6. Convert to latitude/longitude changes
            lat_change = delta_y * lat_scale
            lon_change = delta_x * lon_scale
            
            # 7. Update position
            self.sim_x += lat_change
            self.sim_y += lon_change
            
            # 8. Update speed
            self.sim_speed = speed
            
            # 9. Comprehensive logging
            heading_deg = np.degrees(self.sim_yaw) % 360
            heading_change = ((heading_deg - prev_heading_deg + 180) % 360) - 180  # -180 to +180
            
            logger.info(
                f"SIMULATION: Speed={speed:.2f}m/s, Steering={np.degrees(steering_angle_rad):.1f}°\n"
                f"Position: ({prev_x:.6f}, {prev_y:.6f}) → ({self.sim_x:.6f}, {self.sim_y:.6f})\n"
                f"Heading: {prev_heading_deg:.1f}° → {heading_deg:.1f}° (Δ={heading_change:.1f}°)\n"
                f"Movement: N/S={delta_y:.2f}m, E/W={delta_x:.2f}m"
            )
    
def run_gps_test(waypoints_file, config=None):
    # ... existing code ...
    
    while True:
        # ... existing code ...
        
        # Get GPS data
        lat, lon, heading_rad, speed = gps.get_position_and_heading()
        
        # CRITICAL FIX: Convert heading from radians back to degrees
        heading_deg = np.degrees(heading_rad) % 360
        
        # Print GPS information
        logger.info(f"\n===== GPS DIAGNOSTICS =====")
        logger.info(f"Position: {lat:.8f}, {lon:.8f}")
        logger.info(f"GPS Heading: {heading_deg:.1f}° | Speed: {speed:.2f} m/s")

def run_hardware(waypoints_file, config=None):
    # ... existing code ...
    
    # Create GPS monitor  
    gps = GPSMonitor(simulation_mode=False, use_dual_gps=True)
    
    # Apply the heading fix to ensure consistency

    
    # ... rest of the function ...

def _add_simulation_diagnostics():
    """Add debug output to main.py to display simulation diagnostics"""
    # In run_simulation before applying to hardware/simulation:
    
    # Display simulation diagnostics
    waypoint_lat, waypoint_lon = controller.waypoints[target_idx]
    bearing_to_wp = controller._calculate_bearing(x, y, waypoint_lat, waypoint_lon)
    
    logger.info(
        f"\n--- SIMULATION DIAGNOSTICS ---\n"
        f"Current position: ({x:.6f}, {y:.6f})\n"
        f"Current heading: {np.degrees(yaw):.1f}° (raw yaw: {yaw:.4f} rad)\n"
        f"Target waypoint {target_idx+1}/{len(waypoints)}: ({waypoint_lat:.6f}, {waypoint_lon:.6f})\n"
        f"Bearing to waypoint: {bearing_to_wp:.1f}°\n"
        f"Distance: {distance:.2f}m, CTE: {cte:.2f}m\n"
        f"Heading error: {np.degrees(yaw_error):.1f}°\n"
        f"Steering command: {np.degrees(delta):.1f}°\n"
        f"Speed: {v:.2f}m/s\n"
        f"---------------------------"
    )
    
    # After gps.update_simulation(), add:
    new_x, new_y, new_heading, _ = gps.get_position_and_heading()
    logger.info(
        f"Position updated: ({x:.6f}, {y:.6f}) → ({new_x:.6f}, {new_y:.6f})\n"
        f"Heading updated: {np.degrees(yaw):.1f}° → {new_heading:.1f}°"
    )

# For testing
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    
    gps = GPSMonitor(simulation_mode=True)
    
    try:
        print("Simulating GPS movement. Press Ctrl+C to stop.")
        while True:
            lat, lon, heading_rad, speed = gps.get_position_and_heading()
            
            # CRITICAL FIX: Convert heading from radians to degrees
            heading_deg = np.degrees(heading_rad) % 360
            
            print(f"Position: {lat:.7f}, {lon:.7f} | Heading: {heading_deg:.1f}° | Speed: {speed:.1f} m/s")
            
            # Debug: Show raw values to confirm
            print(f"DEBUG - Raw heading in radians: {heading_rad:.4f}, converted to deg: {heading_deg:.1f}°")
            
            # If using dual GPS, show values for verification
            if not gps.simulation_mode and gps.use_dual_gps:
                front_lat = gps.gps_data["front"]["lat"]
                front_lon = gps.gps_data["front"]["lon"]
                rear_lat = gps.gps_data["rear"]["lat"]
                rear_lon = gps.gps_data["rear"]["lon"]
                if all(p is not None for p in [front_lat, front_lon, rear_lat, rear_lon]):
                    direct_heading = gps._calculate_bearing(rear_lat, rear_lon, front_lat, front_lon)
                    print(f"DEBUG - DIRECT: Rear to Front heading: {direct_heading:.1f}°")
            
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping GPS simulation")
    finally:
        gps.stop()