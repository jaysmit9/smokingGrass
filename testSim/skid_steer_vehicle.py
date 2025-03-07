import math
import numpy as np

class SkidSteerVehicle:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, width=1.0, length=1.0):
        # Position and orientation
        self.x = x
        self.y = y
        self.yaw = yaw  # heading in radians
        
        # Vehicle dimensions
        self.width = width    # distance between wheels (track width)
        self.length = length  # vehicle length
        
        # Motion parameters
        self.max_speed = 1.0  # m/s
        self.max_angular_velocity = 1.0  # rad/s
        
    def set_speeds(self, left_speed, right_speed):
        """Set wheel speeds and update vehicle state"""
        # Limit speeds
        left_speed = max(-self.max_speed, min(self.max_speed, left_speed))
        right_speed = max(-self.max_speed, min(self.max_speed, right_speed))
        
        # Calculate linear and angular velocities
        linear_velocity = (left_speed + right_speed) / 2.0
        angular_velocity = (right_speed - left_speed) / self.width
        
        return linear_velocity, angular_velocity
        
    def move(self, left_speed, right_speed, dt):
        """Move the vehicle based on wheel speeds for time step dt"""
        # Get linear and angular velocities
        v, omega = self.set_speeds(left_speed, right_speed)
        
        # Update position and orientation using simple kinematics
        if abs(omega) < 0.0001:  # Moving in straight line
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
        else:  # Following a curved path
            # Radius of curvature
            r = v / omega
            # Update position and orientation
            self.x += r * (math.sin(self.yaw + omega * dt) - math.sin(self.yaw))
            self.y -= r * (math.cos(self.yaw + omega * dt) - math.cos(self.yaw))
            self.yaw += omega * dt
            
            # Normalize yaw to [-pi, pi]
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
            
        return self.x, self.y, self.yaw