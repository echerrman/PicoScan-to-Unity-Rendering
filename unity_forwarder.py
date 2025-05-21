import socket
import struct
import time
import logging
import numpy as np
from listener import PicoScanListener
from parser import PicoScanParser
from position_tracker import PositionTracker

# COnfigure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Network configuration
UNITY_IP = '127.0.0.1'
UNITY_PORT = 5005
SCANNER_PORT = 2115
MAX_POINTS = 1000000  # Increase as needed for large scans
CHUNK_SIZE = 250
BUFFER_SIZE = 65536
SEND_INTERVAL = 0.2  # Send less frequently for large sets

# Data processing configuration
VOXEL_SIZE = 10.0  # Size of voxels for downsampling
ROUND_DECIMALS = 0  # Controls spatial deduplication (in mm)
SCALE_FACTOR = 0.005  # Scale factor for sending to Unity

# IMU COnfiguration
USE_IMU = False
IMU_WARMUP_SECONDS = 2.0

def send_points_to_unity(points, poses=None):
    """
    Send point cloud data to Unity via UDP.
    
    Args:
        points (set): A set of tuples representing the point cloud data.
        poses: Optional dict with scanner pose information
    """
    unity_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    unity_addr = (UNITY_IP, UNITY_PORT)
    
    points = list(points)
    
    if poses:
        pose_data = struct.pack(
            '4sffffff',
            b'POSE',
            *poses['position'],
            *poses['orientation']
        )
        unity_sock.sendto(pose_data, unity_addr)
    
    # Send points in chunks to avoid large packets
    for i in range(0, len(points), CHUNK_SIZE):
        chunk = points[i:i+CHUNK_SIZE]
        # FOrmat 3 floats: x, y, z for each point, scaled for Unity
        data = b''.join([
            struct.pack('fff', x * SCALE_FACTOR, y * SCALE_FACTOR, z * SCALE_FACTOR)
            for x, y, z in chunk
        ])
        unity_sock.sendto(data, unity_addr)
        logging.debug(f"Sent chunk of {len(chunk)} points to Unity.")
        
    unity_sock.close()
    logger.info(f"Sent {len(points)} points to Unity.")

def main():
    """Main function to process scanner data and forward to Unity."""
    listener = PicoScanListener(host='0.0.0.0', port=SCANNER_PORT, buffer_size=BUFFER_SIZE)
    parser = PicoScanParser(voxel_size=VOXEL_SIZE)
    position_tracker = PositionTracker()
    
    if not listener.setup_socket():
        print("Failed to set up scanner socket.")
        return

    accumulated_points = set()
    last_send = time.time()
    imu_warmup_start = time.time()
    imu_ready = False

    logger.info("Listening for scanner data and building persistent point cloud...")

    try:
        while True:
            data = listener.receive_data()
            if not data:
                logger.debug("No data received, continuing...")
                continue
            logger.debug(f"Received {len(data)} bytes of data.")
            
            # Process received packet
            result = parser.process_packet(data)
            if not result:
                logger.debug("No valid data in packet, continuing...")
                continue
            logger.debug(f"Parser result type: {result['type']}")
            
            if result['type'] == 'imu':
                # Process IMU data to update position and orientation
                imu_data = result['data']
                logger.debug(f"Processing IMU data: {imu_data}")
                
                position_tracker.update_from_imu(imu_data)
                
                # Chekc if IMU warmup is complete
                if not imu_ready and (time.time() - imu_warmup_start) > IMU_WARMUP_SECONDS:
                    imu_ready = True
                    logger.info("IMU warmup complete.")
                    
                # Send current pose to Unity
                if imu_ready:
                    pose = position_tracker.get_pose()
                    logger.debug(f"Sending pose to Unity: {pose}")
                    send_points_to_unity([], pose)
            
            elif result['type'] == 'scan' and (not USE_IMU or imu_ready):
                # Extract point cloud data
                xs = result['x']
                ys = result['y']
                zs = result['z']
                logger.debug(f"Processing scan data: {len(xs)} points")
                
                if USE_IMU:
                    xs, ys, zs = position_tracker.transform_point_cloud(xs, ys, zs)
                
                for x, y, z in zip(xs, ys, zs):
                    # Round to avoid storing near-duplicates (adjust decimals as needed)
                    key = (round(x, ROUND_DECIMALS), round(y, ROUND_DECIMALS), round(z, ROUND_DECIMALS))
                    accumulated_points.add(key)
                    if len(accumulated_points) >= MAX_POINTS:
                        logger.warning("Reached maximum point limit ({MAX_POINTS}), discarding new points")
                        break
                logger.debug(f"Accumulated {len(accumulated_points)} points.")

                # Periodically send the full accumulated set to Unity
                if time.time() - last_send > SEND_INTERVAL and len(accumulated_points) > 0:
                    logger.info(f"Sending {len(accumulated_points)} points to Unity...")
                    pose = position_tracker.get_pose() if USE_IMU else None
                    logger.debug("Calling send_points_to_unity()")
                    send_points_to_unity(accumulated_points, pose)
                    last_send = time.time()
                    
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        listener.close()
        logger.info("Closed sockets.")


if __name__ == "__main__":
    main()
