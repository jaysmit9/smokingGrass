import numpy as np
import math
from geopy.distance import geodesic
import json  # For main function

class StanleyController:
    def __init__(self, waypoints, k=6.0, k_e=0.2, L=1.0, max_steer=0.8726645886, waypoint_reach_threshold=3.0):
        # Initialize controller parameters
        self.waypoints = waypoints
        self.k = k           # Heading gain (keep as is)
        self.k_e = k_e       # DRASTICALLY REDUCE CTE GAIN from 1.0 to 0.2
        self.L = L
        self.max_steer = max_steer
        self.waypoint_reach_threshold = waypoint_reach_threshold
        self.target_idx = 1
        self.prev_delta = 0.0
        
        # Debug info
        print(f"Stanley controller initialized with {len(waypoints)} waypoints")
        print(f"Waypoint reach threshold: {waypoint_reach_threshold} meters")
        print(f"Targeting initial waypoint {self.target_idx}")
        print(f"Controller gains: k={k} (heading), k_e={k_e} (CTE)")

    def add_extension_waypoint(self, extension_distance=1.0):
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
        
        return extended_waypoints

    def haversine_distance(self, coord1, coord2):
        """Calculate the great circle distance between two points in meters"""
        lat1, lon1 = coord1
        lat2, lon2 = coord2
        return geodesic((lat1, lon1), (lat2, lon2)).meters

    def stanley_control(self, x, y, yaw, v):
        """Stanley controller with adaptive gains"""
        # Normalize yaw and convert to degrees (keep existing code)
        yaw = yaw % (2 * np.pi)
        current_heading_deg = np.degrees(yaw) % 360
        
        # Print position (keep existing code)
        print(f"\nCurrent position: ({x:.6f}, {y:.6f}), heading: {np.degrees(yaw):.1f}°, speed: {v:.1f}m/s")
        
        # Find target waypoint (keep existing code)
        old_target = self.target_idx
        self.target_idx = self._find_target_waypoint(x, y, self.target_idx)
        
        # Log waypoint changes (keep existing code)
        if old_target != self.target_idx:
            print(f"TARGET CHANGED from {old_target} to {self.target_idx}")
            # Print all waypoints
        
        # Get waypoints and calculate distance (keep existing code)
        current_point = self.waypoints[self.target_idx]
        prev_idx = max(0, self.target_idx - 1)
        prev_point = self.waypoints[prev_idx]
        
        # CRITICAL DEBUG: Print target waypoint and current position
        print(f"Target waypoint: {current_point[0]:.7f}, {current_point[1]:.7f}")
        print(f"Current position: {x:.7f}, {y:.7f}")
        
        distance = self.haversine_distance((x, y), current_point)
        
        # CRITICAL DEBUG: Print calculated distance
        print(f"Calculated distance: {distance:.2f} meters")
        
        # Calculate CTE (keep existing code)
        # ===== CALCULATE CTE (MISSING CODE) =====
        # 1. Calculate path segment vector (from prev to current waypoint)
        path_vector = [current_point[0] - prev_point[0], current_point[1] - prev_point[1]]
        
        # 2. Calculate vector from prev point to current position
        pos_vector = [x - prev_point[0], y - prev_point[1]]
        
        # 3. Calculate path length - this was missing!
        path_length = np.sqrt(path_vector[0]**2 + path_vector[1]**2)
        
        # 4. Calculate projection of position onto path - this was missing!
        projection = 0  # Default value
        cte = 0  # Default value
        
        if path_length > 0:
            # Normalize path vector
            path_unit = [path_vector[0]/path_length, path_vector[1]/path_length]
            
            # Project position vector onto path
            projection = pos_vector[0]*path_unit[0] + pos_vector[1]*path_unit[1]
            projection = min(max(0, projection), path_length)  # Clamp to segment
            
            # Calculate projection point coordinates
            proj_point = [
                prev_point[0] + projection*path_unit[0],
                prev_point[1] + projection*path_unit[1]
            ]
            
            # Calculate CTE distance in meters
            cte_distance = self.haversine_distance((x, y), proj_point)
            
            # Determine sign of CTE (positive = right of path, negative = left of path)
            cross_z = path_vector[0]*pos_vector[1] - path_vector[1]*pos_vector[0]
            cte = cte_distance * (1 if cross_z >= 0 else -1)
        else:
            # If path is a point, CTE is just distance to the point
            cte = self.haversine_distance((x, y), prev_point)
            projection = 0
        # =================================================
        
        # Get bearing to target and calculate heading error (keep existing code)
        bearing_deg = self._calculate_bearing(x, y, current_point[0], current_point[1])
        heading_error_deg = bearing_deg - current_heading_deg
        while heading_error_deg > 180:
            heading_error_deg -= 360
        while heading_error_deg < -180:
            heading_error_deg += 360
        heading_error = np.radians(heading_error_deg)
        
        # Calculate distance to target waypoint
        distance = self.haversine_distance((x, y), self.waypoints[self.target_idx])
        
        # Adaptive gains based on distance
        k_scale = min(1.0, distance / 10.0)  # Scale from 0-1 up to 10m
        k_e_scale = min(1.0, distance / 5.0)  # Scale from 0-1 up to 5m
        
        adaptive_k = self.k * k_scale
        adaptive_k_e = self.k_e * k_e_scale
        
        # Calculate heading term
        heading_term = adaptive_k * heading_error
        
        # Calculate CTE term
        cte = self._calculate_cross_track_error(x, y, yaw)
        cte_term = np.arctan2(adaptive_k_e * cte, v)
        
        # Combine terms
        delta = heading_term + cte_term
        
        # Apply steering limits
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        
        # Apply smoothing filter
        if hasattr(self, 'prev_delta'):
            delta = 0.7 * delta + 0.3 * self.prev_delta
        
        # Store for next iteration
        self.prev_delta = delta
        
        return delta, self.target_idx, distance, cte, heading_error

    def _haversine_distance(self, coord1, coord2):
        """Calculate the great circle distance between two points in meters"""
        from geopy.distance import geodesic
        return geodesic(coord1, coord2).meters

    def _normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def _calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing between two points in degrees (0-360)"""
        import math
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.atan2(x, y)
        
        return (math.degrees(bearing) + 360) % 360

    def _find_target_waypoint(self, x, y, current_idx):
        """Find the target waypoint to follow"""
        # If we're at the start, find the closest waypoint
        if current_idx == 0:
            min_distance = float('inf')
            closest_idx = -1
            
            for i in range(len(self.waypoints)):
                waypoint = self.waypoints[i]
                distance = self.haversine_distance((x, y), waypoint)
                
                if distance < min_distance:
                    min_distance = distance
                    closest_idx = i
            
            return closest_idx
        
        # Otherwise, continue to the next waypoint
        else:
            # Get the coordinates of the current waypoint
            current_waypoint = self.waypoints[current_idx]
            
            # Calculate the distance to the current waypoint
            distance = self.haversine_distance((x, y), current_waypoint)
            
            # If we've reached the current target within threshold
            if distance < self.waypoint_reach_threshold:
                # Move to next waypoint if we have more
                if current_idx < len(self.waypoints) - 1:
                    next_idx = current_idx + 1
                    return next_idx
                else:
                    return current_idx
            
            return current_idx

    def _calculate_cross_track_error(self, x, y, yaw):
        """Calculate cross track error"""
        # Find the nearest point on the path
        closest_point, target_idx, distance = self._find_closest_waypoint(x, y)
        
        # Calculate the angle between the robot and the path
        angle = np.arctan2(y - closest_point[1], x - closest_point[0])
        
        # Calculate the cross track error
        cross_track_error = distance * np.sin(angle - yaw)
        
        return cross_track_error

    def _find_closest_waypoint(self, x, y):
        """Find the closest waypoint to the current position"""
        min_distance = float('inf')
        closest_idx = -1
        
        for i in range(len(self.waypoints)):
            waypoint = self.waypoints[i]
            distance = self.haversine_distance((x, y), waypoint)
            
            if distance < min_distance:
                min_distance = distance
                closest_idx = i
        
        return self.waypoints[closest_idx], closest_idx, min_distance

def main():
    with open('data/polygon_data.json') as f:
        polygon_data = json.load(f)
    
    waypoints = np.array([(point['lat'], point['lon']) for point in polygon_data])
    controller = StanleyController(waypoints)
    
    controller.control_loop()

if __name__ == "__main__":
    main()
    #jay