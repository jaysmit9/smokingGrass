#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/autonomous-rover/src/utils/map_visualizer.py

import folium
import webbrowser
import time
import threading
import json
import sys
import os
import numpy as np
import http.server
import socketserver
import argparse
import logging
from datetime import datetime
from pathlib import Path

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class MapVisualizer:
    def __init__(self, waypoints_file, port=8000):
        """Initialize the map visualizer with waypoints."""
        self.port = port
        self.waypoints = self._load_waypoints(waypoints_file)
        self.map_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rover_map.html')
        self.position_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'current_position.json')
        self.stop_event = threading.Event()
        
        # Create initial position file
        with open(self.position_file, 'w') as f:
            json.dump({
                "lat": self.waypoints[0][0],
                "lon": self.waypoints[0][1],
                "heading": 0,
                "target_idx": 0,
                "timestamp": datetime.now().isoformat()
            }, f)
        
        # Create the map
        self._create_map()
        
    def _load_waypoints(self, waypoints_file):
        """Load waypoints from file."""
        try:
            with open(waypoints_file, 'r') as f:
                if waypoints_file.endswith('.json'):
                    # JSON format
                    data = json.load(f)
                    if isinstance(data, list) and all('lat' in p and 'lon' in p for p in data):
                        waypoints = [(p['lat'], p['lon']) for p in data]
                    else:
                        # Try other JSON formats
                        if 'waypoints' in data:
                            waypoints = [(p['lat'], p['lon']) for p in data['waypoints']]
                        else:
                            # Try to extract as array of coordinates
                            waypoints = [(p[0], p[1]) for p in data]
                elif waypoints_file.endswith('.csv'):
                    # CSV format
                    import csv
                    waypoints = []
                    with open(waypoints_file, 'r') as f:
                        reader = csv.reader(f)
                        for row in reader:
                            try:
                                lat = float(row[0])
                                lon = float(row[1])
                                waypoints.append((lat, lon))
                            except (ValueError, IndexError):
                                continue
                else:
                    # Try to determine format from content
                    content = f.read().strip()
                    if content.startswith('[') and content.endswith(']'):
                        # Likely JSON
                        data = json.loads(content)
                        waypoints = [(p[0], p[1]) for p in data]
                    else:
                        # Assume simple lat,lon format
                        waypoints = []
                        for line in content.split('\n'):
                            try:
                                lat, lon = line.split(',')
                                waypoints.append((float(lat), float(lon)))
                            except ValueError:
                                continue
            
            logger.info(f"Loaded {len(waypoints)} waypoints")
            return waypoints
            
        except Exception as e:
            logger.error(f"Error loading waypoints: {e}")
            return []
            
    def _create_map(self):
        """Create the OpenStreetMap with waypoints and information panel."""
        if not self.waypoints:
            logger.error("No waypoints to visualize")
            return
            
        # Calculate map center
        center_lat = np.mean([wp[0] for wp in self.waypoints])
        center_lon = np.mean([wp[1] for wp in self.waypoints])
        
        # Create map
        m = folium.Map(location=[center_lat, center_lon], zoom_start=20)
        
        # Add waypoints
        for i, (lat, lon) in enumerate(self.waypoints):
            folium.Marker(
                [lat, lon],
                popup=f"Waypoint {i}",
                icon=folium.Icon(color="blue", icon="flag"),
            ).add_to(m)
            
        # Connect waypoints with a line
        folium.PolyLine(
            self.waypoints,
            color="blue",
            weight=2.5,
            opacity=0.8,
        ).add_to(m)
        
        # Add rover marker (will be updated by JavaScript)
        folium.Marker(
            [self.waypoints[0][0], self.waypoints[0][1]],
            popup="Rover",
            icon=folium.Icon(color="red", icon="car", prefix='fa'),
        ).add_to(m)
        
        # Add info panel for waypoint distances and headings (will be updated by JavaScript)
        info_panel = """
        <div id="info-panel" style="
            position: absolute;
            top: 10px;
            right: 10px;
            z-index: 1000;
            background-color: white;
            padding: 10px;
            border-radius: 5px;
            box-shadow: 0 0 10px rgba(0,0,0,0.5);
            max-width: 300px;
            max-height: 80%;
            overflow-y: auto;
        ">
            <h4 style="margin-top: 0;">Waypoint Information</h4>
            <div id="waypoint-info">Loading...</div>
        </div>
        """
        
        # Add the info panel to the map
        m.get_root().html.add_child(folium.Element(info_panel))
        
        # Add JavaScript to periodically update the rover position and info panel
        import json
        js = """
        <script>
        var roverMarker = null;
        var headingLine = null;
        
        // Haversine formula to calculate distance between points
        function haversineDistance(lat1, lon1, lat2, lon2) {
            function toRad(x) {
                return x * Math.PI / 180;
            }
            
            var R = 6371; // Earth's radius in km
            var dLat = toRad(lat2 - lat1);
            var dLon = toRad(lon2 - lon1);
            var a = Math.sin(dLat/2) * Math.sin(dLat/2) +
                    Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * 
                    Math.sin(dLon/2) * Math.sin(dLon/2);
            var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
            var d = R * c;
            return d * 1000; // Convert to meters
        }
        
        // Calculate bearing between two points
        function calculateBearing(lat1, lon1, lat2, lon2) {
            function toRad(x) {
                return x * Math.PI / 180;
            }
            
            var startLat = toRad(lat1);
            var startLng = toRad(lon1);
            var destLat = toRad(lat2);
            var destLng = toRad(lon2);
            
            var y = Math.sin(destLng - startLng) * Math.cos(destLat);
            var x = Math.cos(startLat) * Math.sin(destLat) -
                    Math.sin(startLat) * Math.cos(destLat) * Math.cos(destLng - startLng);
            var brng = Math.atan2(y, x);
            return (brng * 180 / Math.PI + 360) %% 360; // Convert to degrees
        }
        
        // Calculate relative bearing (difference from current heading)
        function calculateRelativeBearing(currentHeading, targetBearing) {
            var diff = targetBearing - currentHeading;
            // Normalize to -180 to 180
            return ((diff + 180) %% 360) - 180;
        }
        
        // Waypoint coordinates
        var waypoints = %s;
        
        function updateInfoPanel(roverLat, roverLon, heading, targetIdx) {
            var infoHtml = '<table style="width:100%%%%"><tr><th>WP</th><th>Dist (m)</th><th>Heading</th><th>Turn</th></tr>';
            
            waypoints.forEach(function(waypoint, idx) {
                var wpLat = waypoint[0];
                var wpLon = waypoint[1];
                
                // Calculate distance
                var distance = haversineDistance(roverLat, roverLon, wpLat, wpLon);
                
                // Calculate bearing to waypoint
                var bearing = calculateBearing(roverLat, roverLon, wpLat, wpLon);
                
                // Calculate relative bearing (how much to turn)
                var relativeBearing = calculateRelativeBearing(heading, bearing);
                
                // Determine turn direction
                var turnDirection = relativeBearing > 0 ? "→" : relativeBearing < 0 ? "←" : "↑";
                
                // Highlight current target waypoint
                var style = (idx === targetIdx) ? 'background-color: #ffff99;' : '';
                
                infoHtml += '<tr style="' + style + '">' +
                           '<td>' + idx + '</td>' +
                           '<td>' + distance.toFixed(1) + '</td>' +
                           '<td>' + bearing.toFixed(1) + '°</td>' +
                           '<td>' + turnDirection + ' ' + Math.abs(relativeBearing).toFixed(1) + '°</td>' +
                           '</tr>';
            });
            
            infoHtml += '</table>';
            document.getElementById('waypoint-info').innerHTML = infoHtml;
        }
        
        function updatePosition() {
            fetch('current_position.json')
                .then(response => response.json())
                .then(data => {
                    var latlng = [data.lat, data.lon];
                    
                    // Update or create rover marker
                    if (!roverMarker) {
                        roverMarker = L.marker(latlng, {
                            icon: L.divIcon({
                                html: '<div style="color: white; background-color: red; width: 20px; height: 20px; border-radius: 50%%; display: flex; justify-content: center; align-items: center; transform: rotate(' + data.heading + 'deg)">⬆</div>',
                                className: 'rover-marker',
                                iconSize: [20, 20]
                            })
                        }).addTo(map);
                    } else {
                        roverMarker.setLatLng(latlng);
                        roverMarker.setIcon(L.divIcon({
                            html: '<div style="color: white; background-color: red; width: 20px; height: 20px; border-radius: 50%%; display: flex; justify-content: center; align-items: center; transform: rotate(' + data.heading + 'deg)">⬆</div>',
                            className: 'rover-marker',
                            iconSize: [20, 20]
                        }));
                    }
                    
                    roverMarker.bindPopup("Current Heading: " + data.heading.toFixed(1) + "°<br>Target: WP " + data.target_idx);
                    
                    // Update info panel with distances and headings
                    updateInfoPanel(data.lat, data.lon, data.heading, data.target_idx);
                    
                    // Draw heading line
                    if (headingLine) {
                        map.removeLayer(headingLine);
                    }
                    
                    // Convert heading to radians and calculate end point
                    var headingRad = data.heading * Math.PI / 180;
                    var length = 0.00005; // adjust for visible line length
                    var endLat = data.lat + Math.sin(headingRad) * length;
                    var endLng = data.lon + Math.cos(headingRad) * length;
                    
                    headingLine = L.polyline([latlng, [endLat, endLng]], {
                        color: 'red',
                        weight: 2
                    }).addTo(map);
                    
                    // Update every 0.5 seconds
                    setTimeout(updatePosition, 500);
                })
                .catch(error => {
                    console.error('Error fetching position:', error);
                    setTimeout(updatePosition, 2000); // Retry after 2 seconds on error
                });
        }
        
        // Start position updates when the map is ready
        map.whenReady(function() {
            setTimeout(updatePosition, 100);
        });
        </script>
        """ % json.dumps(self.waypoints)
        
        # Add custom JavaScript to the map
        m.get_root().html.add_child(folium.Element(js))
        
        # Save the map
        m.save(self.map_file)
        logger.info(f"Map created at {self.map_file}")
        
    def update_position(self, lat, lon, heading, target_idx):
        """Update the current position of the rover."""
        with open(self.position_file, 'w') as f:
            json.dump({
                "lat": lat,
                "lon": lon,
                "heading": heading,
                "target_idx": target_idx,
                "timestamp": datetime.now().isoformat()
            }, f)
            
    def start_server(self):
        """Start the HTTP server to serve the map."""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        class CustomHandler(http.server.SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=current_dir, **kwargs)
                
            def log_message(self, format, *args):
                # Suppress server logs
                pass
        
        # Create and start the server in a separate thread
        self.server = socketserver.TCPServer(("", self.port), CustomHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()
        logger.info(f"Server started at http://localhost:{self.port}")
        
        # Open the map in browser
        webbrowser.open(f"http://localhost:{self.port}/rover_map.html")
        
    def stop_server(self):
        """Stop the HTTP server."""
        if hasattr(self, 'server'):
            self.server.shutdown()
            self.server.server_close()
            logger.info("Server stopped")
            
    def __del__(self):
        """Clean up resources when the object is destroyed."""
        self.stop_server()
        
    def print_waypoint_info(self, current_lat=None, current_lon=None, current_heading=None):
        """Print static waypoint information to console"""
        import math
        from geopy.distance import geodesic
        
        # Use first waypoint as current position if not provided
        if current_lat is None or current_lon is None:
            current_lat, current_lon = self.waypoints[0]
        
        print("\n=== WAYPOINT INFORMATION ===")
        print(f"{'WP':<4}{'Distance (m)':<14}{'Bearing':<10}{'Turn'}")
        print("-" * 40)
        
        for idx, (wp_lat, wp_lon) in enumerate(self.waypoints):
            # Calculate distance using geodesic
            distance = geodesic((current_lat, current_lon), (wp_lat, wp_lon)).meters
            
            # Calculate bearing
            lat1 = math.radians(current_lat)
            lon1 = math.radians(current_lon)
            lat2 = math.radians(wp_lat)
            lon2 = math.radians(wp_lon)
            
            dlon = lon2 - lon1
            y = math.sin(dlon) * math.cos(lat2)
            x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
            bearing = math.degrees(math.atan2(y, x))
            bearing = (bearing + 360) % 360  # Normalize to 0-360
            
            # Calculate turn direction if heading is provided
            turn = ""
            if current_heading is not None:
                relative_bearing = bearing - current_heading
                relative_bearing = ((relative_bearing + 180) % 360) - 180  # Normalize to -180 to 180
                
                if abs(relative_bearing) < 1:
                    turn_direction = "STRAIGHT"
                elif relative_bearing > 0:
                    turn_direction = "RIGHT"
                else:
                    turn_direction = "LEFT"
                    
                turn = f"{turn_direction} {abs(relative_bearing):.1f}°"
            
            print(f"{idx:<4}{distance:<14.1f}{bearing:<10.1f}°{turn}")
        
        print("=" * 40)

def main():
    """Main function to run the visualizer as a standalone tool."""
    parser = argparse.ArgumentParser(description='Rover Map Visualizer')
    parser.add_argument('--waypoints', '-w', help='Path to waypoints file (default: data/polygon_data.json)',
                       default=None)
    parser.add_argument('--port', '-p', type=int, default=8000, help='HTTP server port')
    args = parser.parse_args()
    
    # If waypoints file not specified, try to use the default polygon_data.json
    if args.waypoints is None:
        # Try different possible locations for the default file
        possible_paths = [
            'data/polygon_data.json',
            '../data/polygon_data.json',
            '../../data/polygon_data.json',
            os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../data/polygon_data.json'),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), '../data/polygon_data.json')
        ]
        
        for path in possible_paths:
            if (os.path.exists(path)):
                args.waypoints = path
                logger.info(f"Using default waypoints file: {path}")
                break
        
        if args.waypoints is None:
            logger.error("Default waypoints file not found. Please specify a waypoints file.")
            sys.exit(1)
    
    visualizer = MapVisualizer(args.waypoints, args.port)
    visualizer.print_waypoint_info()
    visualizer.start_server()
    
    try:
        logger.info("Map visualizer running. Press Ctrl+C to exit.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Stopping visualizer")
    finally:
        visualizer.stop_server()
            
if __name__ == "__main__":
    main()
