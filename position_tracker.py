import numpy as np
from scipy.spatial.transform import Rotation
import logging
from collections import deque
import time

logger = logging.getLogger(__name__)

class PositionTracker:
    """Tracks the position and orientation of a moving object in 3D space."""
    
    def __init__(self, position_smoothing=5, quaternion_smoothing=5):
        # Initial position and orientation
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0])
        
        # Filter settings
        self.position_smoothing = position_smoothing
        self.quaternion_smoothing = quaternion_smoothing
        
        # Filtering buffers
        self.position_buffer = deque(maxlen=position_smoothing)
        self.orientation_buffer = deque(maxlen=quaternion_smoothing)
        
        # Time tracking
        self.last_update_time = None
        self.last_imu_timestamp = None
        
        # Gravity compensation
        self.gravity = np.array([0.0, 0.0, -9.81])
        
        # Calibration
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.calibrated = False
        self.calibration_samples = []
        self.max_calibration_samples = 100
        
        logger.info("PositionTracker initialized.")
        
    def calibrate(self, imu_data):
        """Collect initial samples to determine sensor biases when stationary."""
        if self.calibrated:
            return
        
        # Extract acceleration and angular velocity
        accel = np.array([
            imu_data['acceleration_x'],
            imu_data['acceleration_y'],
            imu_data['acceleration_z']
        ])
        
        gyro = np.array([
            imu_data['angular_velocity_x'],
            imu_data['angular_velocity_y'],
            imu_data['angular_velocity_z']
        ])
        
        self.calibration_samples.append((accel, gyro))
        
        if len(self.calibration_samples) >= self.max_calibration_samples:
            # Calculate average acceleration and angular velocity
            accel_samples = np.array([sample[0] for sample in self.calibration_samples])
            gyro_samples = np.array([sample[1] for sample in self.calibration_samples])
            
            # Calculate biases (assuming stationary during calibration)
            self.accel_bias = np.mean(accel_samples, axis=0)
            # Remove gravity from Z component
            self.accel_bias[2] -= 9.81
            
            self.gyro_bias = np.mean(gyro_samples, axis=0)
            
            self.calibrated = True
            logger.info(f"Calibration complete. Accel bias: {self.accel_bias}, Gyro bias: {self.gyro_bias}")
            
    def update_from_imu(self, imu_data):
        """Update position and orientation based on IMU data"""
        if not self.calibrated:
            self.calibrate(imu_data)
            return
        
        # Extract IMU data
        accel = np.array([
            imu_data['acceleration_x'],
            imu_data['acceleration_y'],
            imu_data['acceleration_z']
        ])
        
        # Extract quaternion from IMU data 
        quat = np.array([
            imu_data['orientation_w'],
            imu_data['orientation_x'],
            imu_data['orientation_y'],
            imu_data['orientation_z']
        ])
        
        # Get timestamp in seconds
        imu_timestamp = imu_data['timestamp'] / 1e6
        
        # Initialize timestamps if needed
        if self.last_update_time is None:
            self.last_update_time = time.time()
        if self.last_imu_timestamp is None:
            self.last_imu_timestamp = imu_timestamp
        
        # Calculate time delta
        dt = imu_timestamp - self.last_imu_timestamp
        if dt <= 0:
            dt = 0.01  # Prevent division by zero
        
        # Update orientation using quaternion
        self.orientation_buffer.append(quat)
        avg_quat = np.mean(self.orientation_buffer, axis=0)
        self.orientation = avg_quat / np.linalg.norm(avg_quat)  # Normalize quaternion
        
        # Apply bias correction
        accel_corrected = accel - self.accel_bias
        
        # Transform acceleration to world space
        rot = Rotation.from_quat([self.orientation[1], self.orientation[2], self.orientation[3], self.orientation[0]])
        accel_world = rot.apply(accel_corrected)
        
        accel_world -= self.gravity  # Compensate for gravity
        
        self.velocity += accel_world * dt
        
        # Apply motion threshold to reduce drift
        vel_norm = np.linalg.norm(self.velocity)
        if vel_norm < 0.02:
            self.velocity = np.zeros(3)
            
        # Update position using integration
        new_position = self.position + self.velocity * dt
        
        # Smooth position
        self.position_buffer.append(new_position)
        self.position = np.mean(self.position_buffer, axis=0)
        
        # Update timestamps
        self.last_update_time = time.time()
        self.last_imu_timestamp = imu_timestamp
        
        logger.debug(f"Updated position: {self.position}, velocity: {self.velocity}, orientation: {self.orientation}")
        
    def transform_point_cloud(self, points_x, points_y, points_z):
        """Transfrom point cloud coordinates to world space."""
        
        points = np.vstack((points_x, points_y, points_z)).T
        
        # Create rotation matrix from quaternion
        rot = Rotation.from_quat([self.orientation[1], self.orientation[2], self.orientation[3], self.orientation[0]])
        
        # APply rotation and translation
        rotated_points = rot.apply(points)
        transformed_points = rotated_points + self.position
        
        # Return as separate x, y, z arrays
        return transformed_points[:, 0], transformed_points[:, 1], transformed_points[:, 2]
    
    def get_pose(self):
        """Return current pose as position and quaternion."""
        return {
            'position': self.position,
            'orientation': self.orientation
        }
    