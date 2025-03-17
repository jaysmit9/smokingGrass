#!/usr/bin/env python3

import socket
import time
import math
import os
import sys
import signal
import threading
import json
import curses
from datetime import datetime
import argparse
try:
    from adafruit_servokit import ServoKit
    SERVO_AVAILABLE = True
except ImportError:
    print("Warning: AdafruitServoKit not available. Motor control will be simulated.")
    SERVO_AVAILABLE = False

# Configuration
CONFIG = {
    "gpsd_host": "localhost",
    "gpsd_port": 2947,
    "update_interval": 0.2,
    "clear_screen": True,
    "waypoints_file": "data/polygon_data.json",
    "max_speed": 0.3,          # Maximum motor speed (0-1)
    "speed_increment": 0.05,   # Speed change per key press
    "turn_factor": 0.7,        # How much to slow inside wheel during turns
    "gps_timeout": 5.0,        # Seconds before considering GPS data stale
    "debug": False             # Enable debug output
}

# Global state
running = True
gps_data = {
    "rover": {
        "lat": None, "lon": None, "heading": None, 
        "speed": None, "fix_mode": None, "last_update": 0,
        "satellites_used": 0, "satellites_visible": 0,
        "precision": {"h": 0.0, "v": 0.0}, "pdop": 0.0,
        "device_path": None
    },
    "base": {
        "lat": None, "lon": None, "heading": None, 
        "speed": None, "fix_mode": None, "last_update": 0,
        "satellites_used": 0, "satellites_visible": 0, 
        "precision": {"h": 0.0, "v": 0.0}, "pdop": 0.0,
        "device_path": None
    }
}

# Motor control state
motor_state = {
    "left_speed": 0.0,
    "right_speed": 0.0,
    "current_speed": 0.0,
    "turning": 0,  # -1=left, 0=straight, 1=right
    "servo_kit": None
}

waypoints = []
first_waypoint = None
log_file = None

# Fix quality descriptions
FIX_MODES = {
    'NO DATA': {'value': 0, 'desc': 'No data', 'color': 'red'},
    'NO FIX': {'value': 1, 'desc': 'No fix', 'color': 'red'},
    '2D FIX': {'value': 2, 'desc': '2D fix', 'color': 'yellow'},
    '3D FIX': {'value': 3, 'desc': '3D fix', 'color': 'green'},
    'GPS': {'value': 1, 'desc': 'GPS SPS mode', 'color': 'yellow'},
    'DGPS': {'value': 2, 'desc': 'Differential GPS', 'color': 'green'},
    'PPS': {'value': 3, 'desc': 'PPS fix', 'color': 'green'},
    'RTK_FIX': {'value': 4, 'desc': 'RTK Fixed', 'color': 'green'},
    'RTK_FLOAT': {'value': 5, 'desc': 'RTK Float', 'color': 'yellow'},
    'DR': {'value': 6, 'desc': 'Dead Reckoning', 'color': 'red'},
    'GNSS+DR': {'value': 7, 'desc': 'GNSS+DR', 'color': 'yellow'},
    'TIME_ONLY': {'value': 8, 'desc': 'Time only', 'color': 'red'},
    'SIM': {'value': 9, 'desc': 'Simulated', 'color': 'red'},
    'WAAS': {'value': 10, 'desc': 'WAAS/SBAS', 'color': 'green'},
    'ESTIMATED': {'value': 11, 'desc': 'Estimated', 'color': 'red'},
    'MANUAL': {'value': 12, 'desc': 'Manual', 'color': 'red'}
}

class GpsdClient:
    """Client for GPSD service"""
    
    def __init__(self, host='localhost', port=2947, debug=False):
        self.host = host
        self.port = port
        self.socket = None
        self.debug = debug
        self.devices = {}  # Store info about each device
        self.last_data_time = datetime.now()
        
    def log(self, message):
        if self.debug:
            print(f"[DEBUG] {message}")
            
    def connect(self):
        """Connect to GPSD service"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            
            # First check devices
            self.socket.send(b'?DEVICES;\n')
            time.sleep(0.5)
            data = self.socket.recv(4096).decode('utf-8').strip()
            self.log(f"DEVICES response: {data}")
            
            # Enable watching with device field reporting
            watch_command = b'?WATCH={"enable":true,"json":true,"device":true,"nmea":true,"raw":1};\n'
            self.socket.send(watch_command)
            self.log(f"Sent WATCH command: {watch_command.decode()}")
            time.sleep(0.5)
            
            data = self.socket.recv(4096).decode('utf-8').strip()
            self.log(f"WATCH response: {data}")
            
            return True
        except Exception as e:
            print(f"Error connecting to GPSD: {e}")
            return False
            
    def process_reports(self, timeout=1.0):
        """Process incoming GPSD reports"""
        if not self.socket:
            return None
            
        try:
            # Use a shorter timeout to process data chunks more efficiently
            self.socket.settimeout(timeout)
            data = self.socket.recv(8192).decode('utf-8').strip()
            
            if not data:
                return None
                
            self.log(f"Received data length: {len(data)} bytes")
            
            # Process each line as a separate JSON object
            reports = []
            for line in data.split('\n'):
                if not line.strip():
                    continue
                try:
                    report = json.loads(line)
                    reports.append(report)
                    
                    # Track devices based on their path
                    if report.get('class') == 'TPV' and 'device' in report:
                        device_path = report.get('device')
                        
                        # Get basic position and fix information
                        status = report.get('status', 0)
                        mode = report.get('mode', 0)
                        
                        # Determine the complete fix mode
                        fix_mode = self.determine_fix_mode(report, mode, status)
                        
                        # Store device info
                        self.devices[device_path] = {
                            'last_update': datetime.now(),
                            'lat': report.get('lat', 0.0),
                            'lon': report.get('lon', 0.0),
                            'alt': report.get('alt', 0.0),
                            'track': report.get('track', 0.0),
                            'speed': report.get('speed', 0.0),
                            'mode': mode,
                            'status': status,
                            'fix_mode': fix_mode,
                            'eph': report.get('eph', 0.0),  # Horizontal position error
                            'epv': report.get('epv', 0.0),  # Vertical position error
                            'sep': report.get('sep', 0.0),  # Estimated position error
                            'raw_data': report.get('raw', ''),  # Store raw NMEA
                            'time': report.get('time', '')  # GPS time
                        }
                    
                    # Process SKY reports for satellite info
                    elif report.get('class') == 'SKY' and 'device' in report:
                        device_path = report.get('device')
                        
                        # Update satellite info if device exists
                        if device_path in self.devices:
                            self.devices[device_path].update({
                                'satellites_used': len([s for s in report.get('satellites', []) if s.get('used', False)]),
                                'satellites_visible': len(report.get('satellites', [])),
                                'hdop': report.get('hdop', 0.0),
                                'vdop': report.get('vdop', 0.0),
                                'pdop': report.get('pdop', 0.0)
                            })
                        else:
                            # Create new device entry if it doesn't exist
                            self.devices[device_path] = {
                                'last_update': datetime.now(),
                                'satellites_used': len([s for s in report.get('satellites', []) if s.get('used', False)]),
                                'satellites_visible': len(report.get('satellites', [])),
                                'hdop': report.get('hdop', 0.0),
                                'vdop': report.get('vdop', 0.0),
                                'pdop': report.get('pdop', 0.0)
                            }
                    
                    # Check NMEA sentences for RTK info
                    elif report.get('class') == 'NMEA' and 'device' in report:
                        device_path = report.get('device')
                        sentence = report.get('string', '')
                        
                        # Look for GGA sentence which contains RTK info
                        if sentence.startswith('$GNGGA') or sentence.startswith('$GPGGA'):
                            parts = sentence.split(',')
                            if len(parts) >= 7:
                                quality = parts[6]
                                if quality in ['4', '5']:  # RTK fix or float
                                    if device_path in self.devices:
                                        fix_mode = "RTK_FIX" if quality == '4' else "RTK_FLOAT"
                                        self.devices[device_path]['fix_mode'] = fix_mode
                                        self.devices[device_path]['rtk_capable'] = True
                                        self.log(f"RTK mode from NMEA for {device_path}: {quality} → {fix_mode}")
                        
                except json.JSONDecodeError as e:
                    self.log(f"JSON error: {e} for line: {line[:50]}...")
                    
            return reports
        except socket.timeout:
            # This is normal, just means no new data in the timeout period
            return None
        except Exception as e:
            print(f"Error receiving data: {e}")
            import traceback
            traceback.print_exc()
            return None
            
    def determine_fix_mode(self, report, mode, status):
        """Determine the detailed fix mode from various indicators"""
        
        # Start with basic mode mapping
        mode_map = {0: 'NO DATA', 1: 'NO FIX', 2: '2D FIX', 3: '3D FIX'}
        fix_mode = mode_map.get(mode, 'UNKNOWN')
        
        # Enhanced mode detection based on status
        status_map = {
            0: None,           # Unknown
            1: None,           # Normal, use basic mode
            2: "DGPS",         # Differential GPS
            3: "RTK_FIX",      # RTK Fixed solution
            4: "RTK_FLOAT",    # RTK Float solution
            5: "DR",           # Dead Reckoning
            6: "GNSS+DR",      # Combined GNSS and DR
            7: "TIME_ONLY",    # Time only
            8: "SIM",          # Simulation
            9: "SBAS"          # WAAS/EGNOS/MSAS
        }
        
        if status in status_map and status_map[status]:
            fix_mode = status_map[status]
        
        # Check for RTCM data, which indicates RTK corrections
        if 'rtcm' in report:
            self.log(f"RTCM data found in report")
            if fix_mode == '3D FIX':  # Only upgrade mode if already have basic 3D fix
                fix_mode = "RTK_DATA"  # Receiving RTK data but not yet applied
        
        # Check GGA sentence for RTK info (most reliable)
        if 'raw' in report:
            raw_data = report.get('raw', '')
            if '$GNGGA' in raw_data or '$GPGGA' in raw_data:
                gga_parts = raw_data.split(',')
                if len(gga_parts) >= 7:
                    quality = gga_parts[6]
                    quality_map = {
                        '0': 'NO FIX',        # Invalid
                        '1': 'GPS',           # GPS SPS
                        '2': 'DGPS',          # DGPS
                        '3': 'PPS',           # PPS
                        '4': 'RTK_FIX',       # RTK Fixed
                        '5': 'RTK_FLOAT',     # RTK Float
                        '6': 'ESTIMATED',     # Estimated/DR
                        '7': 'MANUAL',        # Manual
                        '8': 'SIMULATION',    # Simulation
                        '9': 'WAAS'           # WAAS/SBAS
                    }
                    if quality in quality_map:
                        fix_mode = quality_map[quality]
                        self.log(f"Mode from GGA quality: {quality} → {fix_mode}")
        
        return fix_mode
    
    def get_device_status(self):
        """Return status of all devices"""
        return self.devices
        
    def determine_device_roles(self):
        """Identify which device is rover and which is base"""
        rover = None
        base = None
        unknown_devices = []
        
        for device_path, data in self.devices.items():
            # A device is likely a rover if it:
            # 1. Shows RTK_FIX or RTK_FLOAT mode
            # 2. Has RTCM data
            # 3. Has much higher precision (lower eph)
            if (data.get('fix_mode') in ['RTK_FIX', 'RTK_FLOAT'] or 
                data.get('rtk_capable', False) or
                (data.get('eph', 99) < 0.5 and data.get('mode') == 3)):
                
                rover = device_path
                self.log(f"Identified rover: {device_path}")
                
            # Other devices with 3D fix are likely base stations
            elif data.get('mode') == 3:
                base = device_path
                self.log(f"Identified base: {device_path}")
            else:
                unknown_devices.append(device_path)
        
        return {
            'rover': rover,
            'base': base,
            'unknown': unknown_devices
        }

    def send_poll(self):
        """Send poll command to gpsd"""
        if self.socket:
            try:
                self.socket.send(b'?POLL;\n')
                return True
            except:
                return False
        return False

# Initialize servo kit
def init_motors():
    """Initialize servo motors"""
    if not SERVO_AVAILABLE:
        print("Motor control simulated (ServoKit not available)")
        return True
        
    try:
        motor_state["servo_kit"] = ServoKit(channels=16)
        motor_state["servo_kit"].continuous_servo[0].throttle = 0  # Right servo
        motor_state["servo_kit"].continuous_servo[1].throttle = 0  # Left servo
        print("Motors initialized")
        return True
    except Exception as e:
        print(f"Failed to initialize motors: {e}")
        return False

# Motor control functions
def set_motor_speeds(left, right):
    """Set motor speeds, ensuring they're within bounds"""
    try:
        # Ensure speeds are within -1 to 1
        left = max(-CONFIG["max_speed"], min(CONFIG["max_speed"], left))
        right = max(-CONFIG["max_speed"], min(CONFIG["max_speed"], right))
        
        # Update state
        motor_state["left_speed"] = left
        motor_state["right_speed"] = right
        
        # Set actual motor speeds if servo kit is available
        if SERVO_AVAILABLE and motor_state["servo_kit"]:
            motor_state["servo_kit"].continuous_servo[1].throttle = left
            motor_state["servo_kit"].continuous_servo[0].throttle = right
        
        return True
    except Exception as e:
        print(f"Error setting motor speeds: {e}")
        return False

def stop_motors():
    """Emergency stop for motors"""
    try:
        set_motor_speeds(0, 0)
        motor_state["current_speed"] = 0
        motor_state["turning"] = 0
        return True
    except Exception as e:
        print(f"Error stopping motors: {e}")
        return False

def save_current_waypoint():
    """Save current GPS position as a waypoint"""
    try:
        # Use rover GPS for position
        lat = gps_data["rover"]["lat"]
        lon = gps_data["rover"]["lon"]
        
        if lat is None or lon is None or gps_data["rover"]["fix_mode"] == 'NO FIX':
            print("\nCannot save waypoint: No valid GPS position")
            return False
        
        # Create the waypoint data
        new_waypoint = {"lat": lat, "lon": lon}
        
        # Define the output file path
        output_file = "polygon_data_proposed.json"
        
        # Check if file exists and load existing data
        waypoints_list = []
        if os.path.exists(output_file):
            try:
                with open(output_file, 'r') as f:
                    existing_data = json.load(f)
                    if isinstance(existing_data, list):
                        waypoints_list = existing_data
            except Exception as e:
                print(f"\nError reading existing waypoints: {e}")
        
        # Add new waypoint
        waypoints_list.append(new_waypoint)
        
        # Save to file
        with open(output_file, 'w') as f:
            json.dump(waypoints_list, f, indent=2)
        
        # Get current time for notification
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        # Show success notification
        print(f"\n✅ SAVED WAYPOINT #{len(waypoints_list)} @ {timestamp}")
        print(f"✅ ({lat:.7f}, {lon:.7f}) → {os.path.abspath(output_file)}")
        
        return True
        
    except Exception as e:
        print(f"\nError saving waypoint: {e}")
        return False

def handle_keyboard(key):
    """Handle keyboard input for motor control"""
    current = motor_state["current_speed"]
    turning = motor_state["turning"]
    speed_change = CONFIG["speed_increment"]
    
    # Arrow Up/Down: Change speed
    if key == curses.KEY_UP:
        current += speed_change
        current = min(current, CONFIG["max_speed"])
    elif key == curses.KEY_DOWN:
        current -= speed_change
        current = max(current, -CONFIG["max_speed"])
    # Arrow Left/Right: Turn
    elif key == curses.KEY_LEFT:
        turning = -1
    elif key == curses.KEY_RIGHT:
        turning = 1
    # Space: Stop
    elif key == ord(' '):
        current = 0
        turning = 0
    # 'c' key to center steering
    elif key == ord('c'):
        turning = 0
    # 'p' key to save current position as a waypoint
    elif key == ord('p'):
        save_current_waypoint()
    
    motor_state["current_speed"] = current
    motor_state["turning"] = turning
    
    # Calculate differential speeds for turning
    left_speed = current
    right_speed = current
    
    # Apply minimum turn speed when stationary
    min_turn_speed = CONFIG["max_speed"] * 0.5  # 50% of max speed for turning in place
    
    if turning == -1:  # Turn left
        if current > 0:  # Moving forward
            left_speed = current * (1 - CONFIG["turn_factor"])
        elif current < 0:  # Moving backward
            left_speed = current * (1 + CONFIG["turn_factor"])
        else:  # Stationary - spin in place
            left_speed = -min_turn_speed
            right_speed = min_turn_speed
    elif turning == 1:  # Turn right
        if current > 0:  # Moving forward
            right_speed = current * (1 - CONFIG["turn_factor"])
        elif current < 0:  # Moving backward
            right_speed = current * (1 + CONFIG["turn_factor"])
        else:  # Stationary - spin in place
            left_speed = min_turn_speed
            right_speed = -min_turn_speed
    
    set_motor_speeds(left_speed, right_speed)

def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate the great circle distance between two points in meters"""
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371000  # Radius of earth in meters
    return c * r

def calculate_bearing(lat1, lon1, lat2, lon2):
    """Calculate the bearing between two points in degrees"""
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Calculate bearing
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.atan2(x, y)
    
    # Convert to degrees and normalize to 0-360
    bearing_deg = (math.degrees(bearing) + 360) % 360
    
    return bearing_deg

def calculate_dual_gps_heading():
    """Calculate heading based on positions of rover and base GPS"""
    if (gps_data["rover"]["lat"] is None or 
        gps_data["base"]["lat"] is None or 
        gps_data["rover"]["fix_mode"] == 'NO FIX' or 
        gps_data["base"]["fix_mode"] == 'NO FIX'):
        return None
    
    # Calculate heading as bearing from base to rover GPS
    heading = calculate_bearing(
        gps_data["base"]["lat"], gps_data["base"]["lon"],
        gps_data["rover"]["lat"], gps_data["rover"]["lon"]
    )
    
    return heading

def format_position(lat, lon):
    """Format a lat/lon position for display"""
    if lat is None or lon is None:
        return "No position"
    return f"{lat:.7f}, {lon:.7f}"

def signal_handler(sig, frame):
    """Handle Ctrl+C"""
    global running
    print("\nStopping GPS monitor...")
    running = False

def load_waypoints():
    """Load waypoints from JSON file"""
    global waypoints, first_waypoint
    
    try:
        # Find the JSON file path
        script_dir = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(script_dir, CONFIG["waypoints_file"])
        
        # For absolute path specified in the config
        if not os.path.exists(file_path):
            # Try the absolute path directly
            if os.path.exists(CONFIG["waypoints_file"]):
                file_path = CONFIG["waypoints_file"]
            else:
                # Try project root folder
                project_root = os.path.dirname(script_dir)
                file_path = os.path.join(project_root, CONFIG["waypoints_file"])
                
        # Check if file exists
        if not os.path.exists(file_path):
            print(f"Waypoints file not found: {file_path}")
            return False
            
        # Read JSON waypoints
        with open(file_path, 'r') as file:
            data = json.load(file)
            waypoints = []
            
            # Process each waypoint in the JSON array
            for point in data:
                if "lat" in point and "lon" in point:
                    lat = float(point["lat"])
                    lon = float(point["lon"])
                    waypoints.append((lat, lon))
        
        if waypoints:
            first_waypoint = waypoints[0]
            print(f"Loaded {len(waypoints)} waypoints from JSON. First: {first_waypoint}")
            return True
        else:
            print("No valid waypoints found in JSON file")
            return False
            
    except Exception as e:
        print(f"Error loading waypoints: {e}")
        return False

def calculate_waypoint_metrics(current_lat, current_lon):
    """Calculate distance and heading to first waypoint"""
    if first_waypoint is None or current_lat is None or current_lon is None:
        return None, None
    
    # Calculate distance to first waypoint
    distance = haversine_distance(
        current_lat, current_lon,
        first_waypoint[0], first_waypoint[1]
    )
    
    # Calculate bearing to first waypoint
    heading = calculate_bearing(
        current_lat, current_lon,
        first_waypoint[0], first_waypoint[1]
    )
    
    return distance, heading

def get_fix_status_indicator(fix_mode):
    """Return colored status indicator based on fix mode"""
    if fix_mode in ['RTK_FIX']:
        return "[RTK FIXED] 🟢"
    elif fix_mode in ['RTK_FLOAT']:
        return "[RTK FLOAT] 🟡"
    elif fix_mode in ['DGPS', 'WAAS']:
        return "[DGPS] 🔵"
    elif fix_mode == '3D FIX':
        return "[3D FIX] 🔵"
    elif fix_mode == '2D FIX':
        return "[2D FIX] 🟠"
    elif fix_mode in ['NO FIX', 'NO DATA']:
        return "[NO FIX] 🔴"
    else:
        return f"[{fix_mode}]"

def print_gps_status(stdscr=None):
    """Print current GPS status to terminal or curses window"""
    global gps_data
    
    if stdscr:
        stdscr.clear()
        height, width = stdscr.getmaxyx()
        row = 0
    else:
        if CONFIG["clear_screen"]:
            os.system('cls' if os.name == 'nt' else 'clear')
    
    # Calculate distance between GPSs
    distance_between_gps = None
    if (gps_data["rover"]["lat"] is not None and 
        gps_data["base"]["lat"] is not None):
        distance_between_gps = haversine_distance(
            gps_data["rover"]["lat"], gps_data["rover"]["lon"],
            gps_data["base"]["lat"], gps_data["base"]["lon"]
        )
    
    # Calculate heading from GPSs
    heading = calculate_dual_gps_heading()
    
    # Calculate metrics to first waypoint - use rover GPS position
    waypoint_distance = None
    waypoint_heading = None
    if gps_data["rover"]["lat"] is not None and first_waypoint is not None:
        waypoint_distance, waypoint_heading = calculate_waypoint_metrics(
            gps_data["rover"]["lat"], gps_data["rover"]["lon"]
        )
    
    # Get individual GPS headings
    rover_heading = gps_data["rover"]["heading"]
    base_heading = gps_data["base"]["heading"]
    
    # Prepare lines to print
    lines = []
    lines.append(f"=== GPSD ROVER MONITOR ===  [Arrow keys to drive, Space to stop]  {time.strftime('%H:%M:%S')}")
    lines.append("=" * 60)
    
    # Add motor control status
    turning_indicator = "◄ LEFT" if motor_state["turning"] == -1 else "RIGHT ►" if motor_state["turning"] == 1 else "STRAIGHT"
    direction = "FORWARD" if motor_state["current_speed"] > 0 else "REVERSE" if motor_state["current_speed"] < 0 else "STOPPED"
    
    lines.append(f"MOTORS: {direction} at {abs(motor_state['current_speed']):.2f} - Turning: {turning_indicator}")
    lines.append(f"  Left: {motor_state['left_speed']:.2f}  Right: {motor_state['right_speed']:.2f}")
    lines.append(f"  [Space=STOP] [c=Center] [p=Save Waypoint] [Arrow keys=Drive]")
    lines.append("-" * 60)
    
    # Print GPS device info
    lines.append(f"ROVER GPS: {gps_data['rover']['device_path'] or 'Not detected'}")
    lines.append(f"BASE GPS: {gps_data['base']['device_path'] or 'Not detected'}")
    
    # Print GPS quality information with RTK status
    rover_status = get_fix_status_indicator(gps_data['rover']['fix_mode'])
    base_status = get_fix_status_indicator(gps_data['base']['fix_mode'])
    
    lines.append(f"Rover GPS: {rover_status}")
    lines.append(f"Base GPS: {base_status}")
    lines.append("-" * 60)
    
    # Print satellite information
    lines.append(f"Rover Satellites: {gps_data['rover']['satellites_used']}/{gps_data['rover']['satellites_visible']} " +
          f"HDOP: {gps_data['rover'].get('hdop', 0.0):.2f}")
    lines.append(f"Base Satellites: {gps_data['base']['satellites_used']}/{gps_data['base']['satellites_visible']} " +
          f"HDOP: {gps_data['base'].get('hdop', 0.0):.2f}")
    lines.append("-" * 60)
    
    # Print position data
    lines.append(f"Rover Position: {format_position(gps_data['rover']['lat'], gps_data['rover']['lon'])}")
    lines.append(f"Base Position: {format_position(gps_data['base']['lat'], gps_data['base']['lon'])}")
    
    # Print precision data
    rover_precision = f"H±{gps_data['rover']['precision']['h']:.2f}m V±{gps_data['rover']['precision']['v']:.2f}m"
    base_precision = f"H±{gps_data['base']['precision']['h']:.2f}m V±{gps_data['base']['precision']['v']:.2f}m"
    lines.append(f"Rover Precision: {rover_precision}")
    lines.append(f"Base Precision: {base_precision}")
    lines.append("-" * 60)
    
    # Print heading information
    if heading is not None:
        lines.append(f"GPS Heading: {heading:.1f}°")
    else:
        lines.append(f"GPS Heading: Unknown")
        
    if rover_heading is not None:
        lines.append(f"Rover Course: {rover_heading:.1f}°")
        
    if distance_between_gps is not None:
        lines.append(f"GPS Separation: {distance_between_gps:.2f}m")
    
    # Print waypoint information
    if waypoint_distance is not None and waypoint_heading is not None:
        lines.append(f"Waypoint: {waypoint_distance:.2f}m at {waypoint_heading:.1f}°")
    
    # Print each line
    if stdscr:
        for i, line in enumerate(lines):
            if i < height-1:  # Avoid writing to the bottom line
                stdscr.addstr(i, 0, line[:width-1])  # Avoid writing to the last column
        stdscr.refresh()
    else:
        for line in lines:
            print(line)

def gpsd_reader_thread(gpsd_client):
    """Thread that reads from GPSD and updates the global data structure"""
    global running, gps_data
    
    poll_count = 0
    
    while running:
        try:
            # Every 10 loops, send a POLL command
            if poll_count % 10 == 0:
                gpsd_client.send_poll()
            poll_count += 1
            
            # Process reports from GPSD
            gpsd_client.process_reports()
            
            # Determine device roles (rover vs base)
            roles = gpsd_client.determine_device_roles()
            
            # Update the global gps_data structure with device information
            if roles['rover'] and roles['rover'] in gpsd_client.devices:
                device = gpsd_client.devices[roles['rover']]
                gps_data["rover"]["device_path"] = roles['rover']
                gps_data["rover"]["lat"] = device.get('lat')
                gps_data["rover"]["lon"] = device.get('lon')
                gps_data["rover"]["heading"] = device.get('track')
                gps_data["rover"]["speed"] = device.get('speed')
                gps_data["rover"]["fix_mode"] = device.get('fix_mode', 'UNKNOWN')
                gps_data["rover"]["last_update"] = time.time()
                gps_data["rover"]["satellites_used"] = device.get('satellites_used', 0)
                gps_data["rover"]["satellites_visible"] = device.get('satellites_visible', 0)
                gps_data["rover"]["precision"]["h"] = device.get('eph', 0.0)
                gps_data["rover"]["precision"]["v"] = device.get('epv', 0.0)
                gps_data["rover"]["pdop"] = device.get('pdop', 0.0)
                gps_data["rover"]["hdop"] = device.get('hdop', 0.0)
                gps_data["rover"]["vdop"] = device.get('vdop', 0.0)
                
            if roles['base'] and roles['base'] in gpsd_client.devices:
                device = gpsd_client.devices[roles['base']]
                gps_data["base"]["device_path"] = roles['base']
                gps_data["base"]["lat"] = device.get('lat')
                gps_data["base"]["lon"] = device.get('lon')
                gps_data["base"]["heading"] = device.get('track')
                gps_data["base"]["speed"] = device.get('speed')
                gps_data["base"]["fix_mode"] = device.get('fix_mode', 'UNKNOWN')
                gps_data["base"]["last_update"] = time.time()
                gps_data["base"]["satellites_used"] = device.get('satellites_used', 0)
                gps_data["base"]["satellites_visible"] = device.get('satellites_visible', 0)
                gps_data["base"]["precision"]["h"] = device.get('eph', 0.0)
                gps_data["base"]["precision"]["v"] = device.get('epv', 0.0)
                gps_data["base"]["pdop"] = device.get('pdop', 0.0)
                gps_data["base"]["hdop"] = device.get('hdop', 0.0)
                gps_data["base"]["vdop"] = device.get('vdop', 0.0)
                
            # If no clear rover/base identification, use the first available device
            if not roles['rover'] and not roles['base'] and gpsd_client.devices:
                device_path = list(gpsd_client.devices.keys())[0]
                device = gpsd_client.devices[device_path]
                
                # Assign to rover
                gps_data["rover"]["device_path"] = device_path
                gps_data["rover"]["lat"] = device.get('lat')
                gps_data["rover"]["lon"] = device.get('lon')
                gps_data["rover"]["heading"] = device.get('track')
                gps_data["rover"]["speed"] = device.get('speed')
                gps_data["rover"]["fix_mode"] = device.get('fix_mode', 'UNKNOWN')
                gps_data["rover"]["last_update"] = time.time()
                
            # Small sleep to avoid excessive CPU usage
            time.sleep(CONFIG["update_interval"])
                
        except Exception as e:
            print(f"Error in GPSD reader thread: {e}")
            time.sleep(1)  # Sleep longer on error

def console_ui_thread():
    """Thread that handles console UI and keyboard input"""
    global running
    
    try:
        # Initialize curses
        stdscr = curses.initscr()
        curses.start_color()
        curses.noecho()
        curses.cbreak()
        stdscr.keypad(True)
        stdscr.nodelay(True)  # Non-blocking input
        
        # Main UI loop
        while running:
            # Handle keyboard input
            try:
                key = stdscr.getch()
                if key != -1:  # -1 means no key pressed
                    if key == ord('q'):  # Quit
                        running = False
                    else:
                        handle_keyboard(key)
            except Exception as e:
                print(f"Error handling keyboard input: {e}")
            
            # Update display
            print_gps_status(stdscr)
            
            # Sleep
            time.sleep(CONFIG["update_interval"])
            
    except Exception as e:
        print(f"Error in UI thread: {e}")
    finally:
        # Clean up curses
        try:
            curses.nocbreak()
            stdscr.keypad(False)
            curses.echo()
            curses.endwin()
        except:
            pass

def console_mode():
    """Run GPS monitor in simple console mode without curses"""
    global running
    
    try:
        while running:
            print_gps_status()
            time.sleep(CONFIG["update_interval"])
            
    except KeyboardInterrupt:
        running = False
    except Exception as e:
        print(f"Error in console mode: {e}")

def init_logging():
    """Initialize logging"""
    global log_file
    
    try:
        # Create logs directory if it doesn't exist
        if not os.path.exists("logs"):
            os.makedirs("logs")
        
        # Create a log file with timestamp
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        log_filename = f"logs/gps_log_{timestamp}.csv"
        
        log_file = open(log_filename, "w")
        log_file.write("timestamp,device,lat,lon,alt,heading,speed,fix_mode,satellites_used,satellites_visible,hdop,vdop,pdop,eph,epv\n")
        
        print(f"Logging to {log_filename}")
        return True
    except Exception as e:
        print(f"Error initializing logging: {e}")
        return False

def log_gps_data():
    """Log GPS data to CSV file"""
    global log_file, gps_data
    
    if not log_file:
        return False
    
    try:
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        
        # Log rover data
        if gps_data["rover"]["lat"] is not None:
            log_file.write(f"{timestamp},rover,{gps_data['rover']['lat']},{gps_data['rover']['lon']}," +
                          f"{gps_data['rover'].get('alt', 0)},{gps_data['rover']['heading']},{gps_data['rover']['speed']}," +
                          f"{gps_data['rover']['fix_mode']},{gps_data['rover']['satellites_used']}," +
                          f"{gps_data['rover']['satellites_visible']},{gps_data['rover'].get('hdop', 0)}," +
                          f"{gps_data['rover'].get('vdop', 0)},{gps_data['rover'].get('pdop', 0)}," +
                          f"{gps_data['rover']['precision']['h']},{gps_data['rover']['precision']['v']}\n")
        
        # Log base data
        if gps_data["base"]["lat"] is not None:
            log_file.write(f"{timestamp},base,{gps_data['base']['lat']},{gps_data['base']['lon']}," +
                          f"{gps_data['base'].get('alt', 0)},{gps_data['base']['heading']},{gps_data['base']['speed']}," +
                          f"{gps_data['base']['fix_mode']},{gps_data['base']['satellites_used']}," +
                          f"{gps_data['base']['satellites_visible']},{gps_data['base'].get('hdop', 0)}," +
                          f"{gps_data['base'].get('vdop', 0)},{gps_data['base'].get('pdop', 0)}," +
                          f"{gps_data['base']['precision']['h']},{gps_data['base']['precision']['v']}\n")
        
        log_file.flush()
        return True
    except Exception as e:
        print(f"Error logging GPS data: {e}")
        return False

def logging_thread():
    """Thread that handles periodic logging"""
    global running
    
    log_interval = 1.0  # Log every second
    
    while running:
        try:
            log_gps_data()
            time.sleep(log_interval)
        except Exception as e:
            print(f"Error in logging thread: {e}")
            time.sleep(5)  # Sleep longer on error

def main():
    global running
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="GPSD GPS Monitor for Dual-GPS Rover")
    parser.add_argument("-H", "--host", default=CONFIG["gpsd_host"], help="GPSD host")
    parser.add_argument("-p", "--port", type=int, default=CONFIG["gpsd_port"], help="GPSD port")
    parser.add_argument("-i", "--interval", type=float, default=CONFIG["update_interval"], help="Update interval")
    parser.add_argument("-n", "--no-clear", action="store_true", help="Don't clear screen between updates")
    parser.add_argument("-c", "--console", action="store_true", help="Use simple console mode (no curses)")
    parser.add_argument("-d", "--debug", action="store_true", help="Enable debug output")
    parser.add_argument("-w", "--waypoints", help="Path to waypoints JSON file")
    parser.add_argument("--no-motors", action="store_true", help="Disable motor control")
    args = parser.parse_args()
    
    # Update config with command line args
    CONFIG["gpsd_host"] = args.host
    CONFIG["gpsd_port"] = args.port
    CONFIG["update_interval"] = args.interval
    CONFIG["clear_screen"] = not args.no_clear
    CONFIG["debug"] = args.debug
    if args.waypoints:
        CONFIG["waypoints_file"] = args.waypoints
    
    # Set up signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    print(f"Connecting to GPSD at {CONFIG['gpsd_host']}:{CONFIG['gpsd_port']}...")
    
    # Initialize GPSD client
    gpsd_client = GpsdClient(host=CONFIG["gpsd_host"], port=CONFIG["gpsd_port"], debug=CONFIG["debug"])
    if not gpsd_client.connect():
        print("Failed to connect to GPSD. Is the service running?")
        return 1
    
    # Initialize motors if not disabled
    if not args.no_motors:
        if not init_motors():
            print("Failed to initialize motors. Continuing without motor control.")
    
    # Load waypoints
    load_waypoints()
    
    # Initialize logging
    init_logging()
    
    # Start GPSD reader thread
    gpsd_thread = threading.Thread(target=gpsd_reader_thread, args=(gpsd_client,))
    gpsd_thread.daemon = True
    gpsd_thread.start()
    
    # Start logging thread
    log_thread = threading.Thread(target=logging_thread)
    log_thread.daemon = True
    log_thread.start()
    
    try:
        # Run UI thread
        if args.console:
            console_mode()
        else:
            console_ui_thread()
    except Exception as e:
        print(f"Error in main thread: {e}")
    finally:
        # Clean up
        running = False
        if log_file:
            log_file.close()
        stop_motors()
        print("GPSD GPS Monitor stopped.")

if __name__ == "__main__":
    main()