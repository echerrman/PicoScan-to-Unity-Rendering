import socket
import logging

logger = logging.getLogger(__name__)

int imu_port = 7503

class PicoScanListener:
    """Handles UDP socket setup and receiving data."""

    def __init__(self, host='0.0.0.0', port=2115, buffer_size=65536):
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        self.sock = None

    def setup_socket(self):
        """Set up the UDP socket for listening."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(1.0)
            logger.info(f"Socket bound to {self.host}:{self.port}")
            return True
        except Exception as e:
            logger.error(f"Socket setup failed: {e}")
            return False

    def receive_data(self):
        """Receive data from the UDP socket."""
        try:
            data, addr = self.sock.recvfrom(self.buffer_size)
            logger.debug(f"Received {len(data)} bytes from {addr}")
            return data
        except socket.timeout:
            return None
        except Exception as e:
            logger.error(f"Error receiving data: {e}")
            return None

    def close(self):
        if self.sock:
            self.sock.close()