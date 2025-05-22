import socket
import struct
import time
import base64
from listener import PicoScanListener
from parser import PicoScanParser

UNITY_IP = '127.0.0.1'
UNITY_PORT = 5005
SCANNER_PORT = 2115
MAX_POINTS = 1000000  # Increase as needed for large scans
CHUNK_SIZE = 250
BUFFER_SIZE = 65536
SEND_INTERVAL = 0.2  # Send less frequently for large sets

ROUND_DECIMALS = 0  # Controls spatial deduplication (in mm)

def send_points_to_unity(points):
    unity_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    unity_addr = (UNITY_IP, UNITY_PORT)
    points = list(points)
    for i in range(0, len(points), CHUNK_SIZE):
        chunk = points[i:i+CHUNK_SIZE]
        data = b''.join([struct.pack('fff', x * 0.005, y * 0.005, z * 0.005) for x, y, z in chunk])
        unity_sock.sendto(data, unity_addr)
    unity_sock.close()

def main():
    listener = PicoScanListener(host='0.0.0.0', port=SCANNER_PORT, buffer_size=BUFFER_SIZE)
    parser = PicoScanParser()
    if not listener.setup_socket():
        print("Failed to set up scanner socket.")
        return

    accumulated_points = set()
    last_send = time.time()

    print("Listening for scanner data and building persistent point cloud...")
    
    try:
        while True:
            data = listener.receive_data()
            if not data:
                continue
            point_cloud = parser.process_packet(data)
            if not point_cloud:
                continue
            xs = point_cloud['x']
            ys = point_cloud['y']
            zs = point_cloud['z']
            for x, y, z in zip(xs, ys, zs):
                # Round to avoid storing near-duplicates (adjust decimals as needed)
                key = (round(x, ROUND_DECIMALS), round(y, ROUND_DECIMALS), round(z, ROUND_DECIMALS))
                accumulated_points.add(key)
                if len(accumulated_points) >= MAX_POINTS:
                    break

            # Periodically send the full accumulated set to Unity
            if time.time() - last_send > SEND_INTERVAL and len(accumulated_points) > 0:
                print(f"Sending {len(accumulated_points)} points to Unity...")
                send_points_to_unity(accumulated_points)
                last_send = time.time()
    finally:
        listener.close()
        print("Closed sockets.")

if __name__ == "__main__":
    main()
