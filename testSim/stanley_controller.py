import math
import numpy as np

class StanleyController:
    def __init__(self, k_crosstrack=1.0, k_heading=1.0):
        """
        Initialize Stanley controller
        
        Args:
            k_crosstrack: Cross track error gain
            k_heading: Heading error gain
        """
        self.k_crosstrack = k_crosstrack
        self.k_heading = k_heading
    
    def control(self, vehicle, path, nearest_idx, target_speed):
        """
        Calculate steering control for skid-steer vehicle using Stanley method
        
        Args:
            vehicle: Vehicle object with state information
            path: Array of [x, y] path points
            nearest_idx: Index of nearest path point
            target_speed: Desired speed of the vehicle
            
        Returns:
            left_speed: Speed command for left wheels
            right_speed: Speed command for right wheels
        """
        # Extract current path segment
        current_x, current_y = vehicle.x, vehicle.y
        
        # Get path segment (use next point as target)
        idx = min(nearest_idx, len(path) - 2)  # Ensure we don't exceed path bounds
        path_x, path_y = path[idx]
        next_x, next_y = path[idx + 1]
        
        # Calculate path heading
        path_heading = math.atan2(next_y - path_y, next_x - path_x)
        
        # Calculate heading error
        heading_error = path_heading - vehicle.yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))  # Normalize
        
        # Calculate crosstrack error (perpendicular distance to path)
        # Vector from path point to vehicle
        dx = current_x - path_x
        dy = current_y - path_y
        
        # Path direction vector
        path_dx = next_x - path_x
        path_dy = next_y - path_y
        path_length = math.sqrt(path_dx**2 + path_dy**2)
        
        # Normalize path vector
        if path_length > 0:
            path_dx /= path_length
            path_dy /= path_length
        
        # Cross track error (signed)
        crosstrack_error = dy * path_dx - dx * path_dy
        
        # Speed for tracking
        speed = 0.5  # Base speed
        
        # Stanley control law
        crosstrack_term = math.atan2(self.k_crosstrack * crosstrack_error, speed)
        
        # Calculate final steering angle
        steering = self.k_heading * heading_error + crosstrack_term
        
        # Clamp steering to limits for skid-steer
        steering = max(-math.pi/2, min(math.pi/2, steering))
        
        # Convert steering to wheel speeds for skid-steer
        base_speed = target_speed * (1 + min(1.0, 0.5 * (abs(crosstrack_error) + abs(heading_error))))
        
        # For skid-steer, translate steering to differential wheel speeds
        if steering > 0:  # Turn right
            left_speed = base_speed
            right_speed = base_speed * (1 - 2.0 * abs(steering) / math.pi)
        else:  # Turn left or straight
            right_speed = base_speed
            left_speed = base_speed * (1 - 2.0 * abs(steering) / math.pi)
        
        # After controller.control() call
        # Adjust speeds based on error magnitude
        error_magnitude = abs(crosstrack_error) + abs(heading_error)
        adjustment = 1.0 + min(1.0, 0.6 * error_magnitude)  # Scale factor
        left_speed *= adjustment
        right_speed *= adjustment
        
        return left_speed, right_speed, crosstrack_error, heading_error