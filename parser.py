import struct
import binascii
import numpy as np
import logging
from datetime import datetime

logger = logging.getLogger(__name__)

class PicoScanParser:
    """Parses picoScan 150 sensor data in compact format."""

    STX = b'\x02\x02\x02\x02'
    HEADER_SIZE = 32
    
    # Command IDs
    CMD_SCAN_DATA = 1
    CMD_IMU_DATA = 2

    def __init__(self, voxel_size=10.0, point_limit=10000):
        self.frame_counter = 0
        self.segments_received = set()
        self.voxel_size = voxel_size
        self.point_limit = point_limit
        self.total_packets = 0
        self.total_points = 0

    def validate_frame(self, data):
        if len(data) < self.HEADER_SIZE + 4:
            logger.warning(f"Packet too small: {len(data)} bytes")
            return False
        if data[:4] != self.STX:
            logger.warning("Invalid start of frame marker")
            return False
        received_crc = struct.unpack('<I', data[-4:])[0]
        calculated_crc = binascii.crc32(data[:-4]) & 0xFFFFFFFF
        if received_crc != calculated_crc:
            logger.warning(f"CRC check failed: expected {received_crc}, got {calculated_crc}")
            return False
        return True

    def parse_header(self, data):
        header = {}
        header['startOfFrame'] = data[:4]
        header['commandId'] = struct.unpack('<I', data[4:8])[0]
        header['telegramCounter'] = struct.unpack('<Q', data[8:16])[0]
        header['timeStampTransmit'] = struct.unpack('<Q', data[16:24])[0]
        header['telegramVersion'] = struct.unpack('<I', data[24:28])[0]
        header['sizeModule0'] = struct.unpack('<I', data[28:32])[0]
        return header
    
    def parse_imu_data(self, data):
        """Parses IMU data from the packet."""
        if len(data) < 64:
            logger.warning(f"IMU data packet too small: {len(data)} bytes")
            return None
        
        imu_data = {}
        imu_data['startOfFrame'] = data[:4]
        imu_data['commandId'] = struct.unpack('<I', data[4:8])[0]
        imu_data['telegramVersion'] = struct.unpack('<I', data[8:12])[0]
        
        # Parse acceleration data
        imu_data['acceleration_x'] = struct.unpack('<f', data[12:16])[0]
        imu_data['acceleration_y'] = struct.unpack('<f', data[16:20])[0]
        imu_data['acceleration_z'] = struct.unpack('<f', data[20:24])[0]
        
        # Parse angular velocity data
        imu_data['angular_velocity_x'] = struct.unpack('<f', data[24:28])[0]
        imu_data['angular_velocity_y'] = struct.unpack('<f', data[28:32])[0]
        imu_data['angular_velocity_z'] = struct.unpack('<f', data[32:36])[0]
        
        # Parse orientation quaternion data
        imu_data['orientation_w'] = struct.unpack('<f', data[36:40])[0]
        imu_data['orientation_x'] = struct.unpack('<f', data[40:44])[0]
        imu_data['orientation_y'] = struct.unpack('<f', data[44:48])[0]
        imu_data['orientation_z'] = struct.unpack('<f', data[48:52])[0]
        
        # Parse timestamp
        imu_data['timestamp'] = struct.unpack('<Q', data[52:60])[0]
        
        # Verify CRC
        received_crc = struct.unpack('<I', data[60:64])[0]
        calculated_crc = binascii.crc32(data[:60]) & 0xFFFFFFFF
        if received_crc != calculated_crc:
            logger.warning(f"IMU data CRC check failed: expected {received_crc}, got {calculated_crc}")
            return None
        return imu_data

    def parse_module_metadata(self, data):
        metadata = {}
        offset = 0
        metadata['segmentCounter'] = struct.unpack('<Q', data[offset:offset+8])[0]
        offset += 8
        metadata['frameNumber'] = struct.unpack('<Q', data[offset:offset+8])[0]
        offset += 8
        metadata['senderId'] = struct.unpack('<I', data[offset:offset+4])[0]
        offset += 4
        metadata['numberOfLinesInModule'] = struct.unpack('<I', data[offset:offset+4])[0]
        offset += 4
        metadata['numberOfBeamsPerScan'] = struct.unpack('<I', data[offset:offset+4])[0]
        offset += 4
        metadata['numberOfEchosPerBeam'] = struct.unpack('<I', data[offset:offset+4])[0]
        offset += 4
        num_lines = metadata['numberOfLinesInModule']
        metadata['timeStampStart'] = [struct.unpack('<Q', data[offset+i*8:offset+(i+1)*8])[0] for i in range(num_lines)]
        offset += 8 * num_lines
        metadata['timeStampStop'] = [struct.unpack('<Q', data[offset+i*8:offset+(i+1)*8])[0] for i in range(num_lines)]
        offset += 8 * num_lines
        metadata['phi'] = [struct.unpack('<f', data[offset+i*4:offset+(i+1)*4])[0] for i in range(num_lines)]
        offset += 4 * num_lines
        metadata['thetaStart'] = [struct.unpack('<f', data[offset+i*4:offset+(i+1)*4])[0] for i in range(num_lines)]
        offset += 4 * num_lines
        metadata['thetaStop'] = [struct.unpack('<f', data[offset+i*4:offset+(i+1)*4])[0] for i in range(num_lines)]
        offset += 4 * num_lines
        metadata['distanceScalingFactor'] = struct.unpack('<f', data[offset:offset+4])[0]
        offset += 4
        metadata['nextModuleSize'] = struct.unpack('<I', data[offset:offset+4])[0]
        offset += 4
        offset += 1  # Reserved
        metadata['dataContentEchos'] = data[offset]
        offset += 1
        metadata['dataContentBeams'] = data[offset]
        offset += 1
        offset += 1  # Reserved
        return metadata, offset

    def get_echo_data_size(self, data_content_echos):
        size = 0
        if data_content_echos & 0x01:
            size += 2
        if data_content_echos & 0x02:
            size += 2
        return size

    def get_beam_data_size(self, data_content_beams):
        size = 0
        if data_content_beams & 0x01:
            size += 1
        if data_content_beams & 0x02:
            size += 2
        return size

    def parse_measurement_data(self, data, metadata, metadata_size):
        num_lines = metadata['numberOfLinesInModule']
        num_beams = metadata['numberOfBeamsPerScan']
        num_echos = metadata['numberOfEchosPerBeam']
        distance_scaling = metadata['distanceScalingFactor']
        echo_data_size = self.get_echo_data_size(metadata['dataContentEchos'])
        beam_data_size = self.get_beam_data_size(metadata['dataContentBeams'])
        measurements = []
        offset = metadata_size
        for beam_idx in range(num_beams):
            for line_idx in range(num_lines):
                beam_data = {
                    'line_idx': line_idx,
                    'beam_idx': beam_idx,
                    'phi': metadata['phi'][line_idx],
                    'echoes': []
                }
                for echo_idx in range(num_echos):
                    echo = {}
                    if metadata['dataContentEchos'] & 0x01:
                        raw_distance = struct.unpack('<H', data[offset:offset+2])[0]
                        echo['distance'] = raw_distance * distance_scaling
                        offset += 2
                    if metadata['dataContentEchos'] & 0x02:
                        echo['rssi'] = struct.unpack('<H', data[offset:offset+2])[0]
                        offset += 2
                    beam_data['echoes'].append(echo)
                if metadata['dataContentBeams'] & 0x01:
                    properties = data[offset]
                    beam_data['reflector_detected'] = bool(properties & 0x01)
                    offset += 1
                if metadata['dataContentBeams'] & 0x02:
                    raw_theta = struct.unpack('<H', data[offset:offset+2])[0]
                    beam_data['theta'] = (raw_theta - 16384) / 5215
                    offset += 2
                else:
                    theta_start = metadata['thetaStart'][line_idx]
                    theta_stop = metadata['thetaStop'][line_idx]
                    beam_data['theta'] = theta_start + (theta_stop - theta_start) * beam_idx / (num_beams - 1)
                measurements.append(beam_data)
        return measurements

    def parse_module(self, data):
        metadata, metadata_size = self.parse_module_metadata(data)
        measurements = self.parse_measurement_data(data, metadata, metadata_size)
        module_data = {'metadata': metadata, 'measurements': measurements}
        next_module_size = metadata['nextModuleSize']
        return module_data, len(data), next_module_size

    def parse_frame(self, data):
        if not self.validate_frame(data):
            return None
        
        header = self.parse_header(data)
        
        if header['commandId'] == self.CMD_IMU_DATA:
            logger.info(f"Received IMU data packet")
            return self.parse_imu_data(data)
        elif header['commandId'] != self.CMD_SCAN_DATA:
            logger.info(f"Received unknown data packet (commandId: {header['commandId']})")
            return header
        
        modules = []
        current_offset = self.HEADER_SIZE
        current_module_size = header['sizeModule0']
        while current_module_size > 0:
            module_data, _, next_module_size = self.parse_module(
                data[current_offset:current_offset + current_module_size]
            )
            modules.append(module_data)
            current_offset += current_module_size
            current_module_size = next_module_size
            if current_offset + current_module_size > len(data) - 4:
                break
        frame_data = {'header': header, 'modules': modules}
        if modules:
            segment_counter = modules[0]['metadata']['segmentCounter']
            frame_number = modules[0]['metadata']['frameNumber']
            if frame_number != self.frame_counter:
                if self.frame_counter > 0:
                    logger.info(f"Completed frame {self.frame_counter} with {len(self.segments_received)} segments")
                self.frame_counter = frame_number
                self.segments_received = set()
            self.segments_received.add(segment_counter)
            #logger.info(f"Received segment {segment_counter} of frame {frame_number}")
        return frame_data

    def extract_point_cloud(self, frame_data):
        points_x, points_y, points_z, intensities = [], [], [], []
        for module in frame_data['modules']:
            for measurement in module['measurements']:
                phi = measurement['phi']
                theta = measurement['theta']
                if measurement['echoes'] and 'distance' in measurement['echoes'][0]:
                    distance = measurement['echoes'][0]['distance']
                    x = distance * np.cos(phi) * np.cos(theta)
                    y = distance * np.cos(phi) * np.sin(theta)
                    z = distance * np.sin(phi)
                    points_x.append(x)
                    points_y.append(y)
                    points_z.append(z)
                    if 'rssi' in measurement['echoes'][0]:
                        intensities.append(measurement['echoes'][0]['rssi'])
                    else:
                        intensities.append(0)
        return np.array(points_x), np.array(points_y), np.array(points_z), np.array(intensities)

    def downsample_point_cloud(self, x, y, z, intensities, voxel_size=None):
        if voxel_size is None:
            voxel_size = self.voxel_size
        if len(x) == 0:
            return x, y, z, intensities
        voxels = {}
        for i in range(len(x)):
            vx = int(x[i] // voxel_size)
            vy = int(y[i] // voxel_size)
            vz = int(z[i] // voxel_size)
            key = (vx, vy, vz)
            if key not in voxels:
                voxels[key] = (x[i], y[i], z[i], intensities[i])
        if voxels:
            x_down, y_down, z_down, int_down = zip(*voxels.values())
            return np.array(x_down), np.array(y_down), np.array(z_down), np.array(int_down)
        else:
            return np.array([]), np.array([]), np.array([]), np.array([])

    def process_packet(self, data):
        """Process any type of packet from the scanner"""
        if not self.validate_frame(data):
            return None
        
        if len(data) >= 8:
            command_id = struct.unpack('<I', data[4:8])[0]
            
            if command_id == self.CMD_IMU_DATA:
                imu_data = self.parse_imu_data(data)
                if imu_data:
                    return {
                        'type': 'imu',
                        'data': imu_data,
                        'timestamp': datetime.now()
                    }
                return None
                

        frame_data = self.parse_frame(data)
        if frame_data and 'modules' in frame_data:
            x, y, z, intensities = self.extract_point_cloud(frame_data)
            if len(x) > 0:
                x, y, z, intensities = self.downsample_point_cloud(
                    x, y, z, intensities, voxel_size=self.voxel_size
                )
            point_cloud = {
                'type': 'scan',
                'x': x,
                'y': y,
                'z': z,
                'intensities': intensities,
                'timestamp': datetime.now(),
                'telegram_counter': frame_data['header']['telegramCounter']
            }
            self.total_packets += 1
            self.total_points += len(x)
            return point_cloud
        return None