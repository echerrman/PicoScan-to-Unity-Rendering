import socket
import struct
import time
import threading
import json
from listener import PicoScanListener
from parser import PicoScanParser

UNITY_IP = '127.0.0.1'
UNITY_PORT = 5005
CONTROL_PORT = 5006  # New port for receiving control commands
SCANNER_PORT = 2115
MAX_POINTS = 1000000  # Increase as needed for large scans
CHUNK_SIZE = 250
BUFFER_SIZE = 65536
SEND_INTERVAL = 0.05  # Faster updates for better real-time response

ROUND_DECIMALS = 0  # Controls spatial deduplication (in mm)

class PointCloudForwarder:
    def __init__(self):
        self.persistent_mode = True  # Start in persistent mode
        self.accumulated_points = set()
        self.current_frame_points = set()
        self.last_send = time.time()
        self.control_socket = None
        self.running = True
        
    def setup_control_socket(self):
        """Set up UDP socket to receive control commands from Unity"""
        try:
            self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.control_socket.bind(('127.0.0.1', CONTROL_PORT))
            self.control_socket.settimeout(0.1)  # Non-blocking with short timeout
            print(f"Control socket listening on port {CONTROL_PORT}")
            return True
        except Exception as e:
            print(f"Failed to setup control socket: {e}")
            return False
    
    def check_control_commands(self):
        """Check for control commands from Unity"""
        if not self.control_socket:
            return
            
        try:
            data, addr = self.control_socket.recvfrom(1024)
            command = data.decode('utf-8').strip()
            
            if command == "PERSISTENT":
                self.persistent_mode = True
                print("Switched to PERSISTENT mode")
            elif command == "LIVE_ONLY":
                self.persistent_mode = False
                self.accumulated_points.clear()  # Clear accumulated points
                print("Switched to LIVE_ONLY mode")
            elif command == "CLEAR":
                self.accumulated_points.clear()
                self.current_frame_points.clear()
                print("Cleared all points")
            elif command == "STATUS":
                status = {
                    "mode": "PERSISTENT" if self.persistent_mode else "LIVE_ONLY",
                    "accumulated_points": len(self.accumulated_points),
                    "current_frame_points": len(self.current_frame_points)
                }
                response = json.dumps(status).encode('utf-8')
                self.control_socket.sendto(response, addr)
                
        except socket.timeout:
            pass  # No command received, continue
        except Exception as e:
            print(f"Error checking control commands: {e}")

    def send_points_to_unity(self, points):
        """Send point cloud data to Unity"""
        unity_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        unity_addr = (UNITY_IP, UNITY_PORT)
        points = list(points)
        
        if not points:
            unity_sock.close()
            return
        
        # For live mode, send faster with less protocol overhead
        if not self.persistent_mode:
            # Fast path for live-only mode - skip mode header for speed
            for i in range(0, len(points), CHUNK_SIZE):
                chunk = points[i:i+CHUNK_SIZE]
                data = b''.join([struct.pack('fff', x * 0.005, y * 0.005, z * 0.005) for x, y, z in chunk])
                unity_sock.sendto(b"LIVE:" + data, unity_addr)
        else:
            # Full protocol for persistent mode
            mode_data = "PERSISTENT".encode('utf-8')
            unity_sock.sendto(b"MODE:" + mode_data, unity_addr)
            
            # Send point data in chunks
            for i in range(0, len(points), CHUNK_SIZE):
                chunk = points[i:i+CHUNK_SIZE]
                data = b''.join([struct.pack('fff', x * 0.005, y * 0.005, z * 0.005) for x, y, z in chunk])
                unity_sock.sendto(b"POINTS:" + data, unity_addr)
            
            # Send end marker
            unity_sock.sendto(b"END", unity_addr)
        
        unity_sock.close()

    def process_point_cloud(self, point_cloud):
        """Process incoming point cloud based on current mode"""
        xs = point_cloud['x']
        ys = point_cloud['y']
        zs = point_cloud['z']
        
        if self.persistent_mode:
            # Add to accumulated points
            for x, y, z in zip(xs, ys, zs):
                key = (round(x, ROUND_DECIMALS), round(y, ROUND_DECIMALS), round(z, ROUND_DECIMALS))
                self.accumulated_points.add(key)
                if len(self.accumulated_points) >= MAX_POINTS:
                    break
            
            # Send accumulated points periodically
            if time.time() - self.last_send > SEND_INTERVAL and len(self.accumulated_points) > 0:
                print(f"Sending {len(self.accumulated_points)} accumulated points to Unity...")
                self.send_points_to_unity(self.accumulated_points)
                self.last_send = time.time()
        else:
            # Live-only mode: send immediately for low latency
            self.current_frame_points.clear()
            for x, y, z in zip(xs, ys, zs):
                key = (round(x, ROUND_DECIMALS), round(y, ROUND_DECIMALS), round(z, ROUND_DECIMALS))
                self.current_frame_points.add(key)
            
            if len(self.current_frame_points) > 0:
                print(f"Sending {len(self.current_frame_points)} live points to Unity...")
                self.send_points_to_unity(self.current_frame_points)

    def run(self):
        """Main execution loop"""
        listener = PicoScanListener(host='0.0.0.0', port=SCANNER_PORT, buffer_size=BUFFER_SIZE)
        parser = PicoScanParser()
        
        if not listener.setup_socket():
            print("Failed to set up scanner socket.")
            return
            
        if not self.setup_control_socket():
            print("Failed to set up control socket.")
            return

        print("Listening for scanner data...")
        print("Commands: Send 'PERSISTENT', 'LIVE_ONLY', 'CLEAR', or 'STATUS' to control port")
        print(f"Current mode: {'PERSISTENT' if self.persistent_mode else 'LIVE_ONLY'}")
        
        try:
            while self.running:
                # Check for control commands
                self.check_control_commands()
                
                # Process scanner data
                data = listener.receive_data()
                if not data:
                    continue
                    
                point_cloud = parser.process_packet(data)
                if not point_cloud:
                    continue
                    
                self.process_point_cloud(point_cloud)
                
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            listener.close()
            if self.control_socket:
                self.control_socket.close()
            print("Closed sockets.")

def main():
    forwarder = PointCloudForwarder()
    forwarder.run()

if __name__ == "__main__":
    main()
