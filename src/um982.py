import serial
import serial.tools.list_ports
import threading
import time
import math
import logging

# Set up logger
logger = logging.getLogger(__name__)

def _generate_crc32_table():
    """Generate CRC-32 lookup table according to PEP 8 guidelines."""
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
        table.append(crc)
    return table


# Pre-compute CRC table for better performance
NMEA_EXPEND_CRC_TABLE = _generate_crc32_table()


def nmea_expend_crc(nmea_expend_sentence):
    """Validate CRC32 checksum for extended NMEA sentences."""
    try:
        sentence, crc = nmea_expend_sentence[1:].split("*")
        crc = crc[:8].lower()
        
        # Calculate CRC32 in one pass
        calculated_crc = 0
        for byte in sentence.encode():
            calculated_crc = NMEA_EXPEND_CRC_TABLE[(calculated_crc ^ byte) & 0xFF] ^ (calculated_crc >> 8)
        
        return crc == format(calculated_crc & 0xFFFFFFFF, '08x')
    except Exception:
        return False


def nmea_crc(nmea_sentence):
    """Validate simple XOR checksum for standard NMEA sentences."""
    try:
        sentence, crc = nmea_sentence[1:].split("*")
        crc = crc[:2].upper()
        
        # Calculate XOR checksum
        calculated_checksum = 0
        for char in sentence:
            calculated_checksum ^= ord(char)
            
        return format(calculated_checksum, 'X').zfill(2) == crc
    except Exception:
        return False


def msg_separate(msg):
    """Parse None NMEA message into header and body."""
    parts = msg[:msg.find('*')].split(';')
    return {
        'header': parts[0],
        'body': parts[1].split(',') if len(parts) > 1 else []
    }


def pvtsln_solver(msg):
    """Parse PVTSLN message into position and heading data."""
    parts = msg_separate(msg)
    body = parts['body']
    
    # Unpack position data in one operation
    bestpos = [
        body[0],                # type
        float(body[1]),         # height
        float(body[2]),         # latitude
        float(body[3]),         # longitude
        float(body[4]),         # height std
        float(body[5]),         # latitude std
        float(body[6])          # longitude std
    ]
    
    # Unpack heading data in one operation
    heading = [
        body[20],               # type
        float(body[21]),        # length
        float(body[22]),        # degrees
        float(body[23])         # pitch
    ]
    
    return bestpos, heading


def gnhpr_solver(msg):
    """Parse GNHPR message for orientation data."""
    parts = msg_separate(msg)
    body = parts['body']
    
    # Return heading, pitch, roll tuple
    return [float(body[2]), float(body[3]), float(body[4])]


def bestnav_solver(msg):
    """Parse BESTNAV message for velocity data."""
    parts = msg_separate(msg)
    body = parts['body']
    
    # Extract velocity components
    vel_hor = float(body[-5])
    vel_heading_rad = math.radians(float(body[-4]))
    
    return (
        vel_hor * math.sin(vel_heading_rad),  # vel_east
        vel_hor * math.cos(vel_heading_rad),  # vel_north
        float(body[-3]),                      # vel_ver
        float(body[-1]),                      # vel_hor_std
        float(body[-1]),                      # vel_hor_std (duplicated)
        float(body[-2])                       # vel_ver_std
    )


class UM982:
    """Interface class for UM982 GNSS receiver."""
    
    

    def __init__(
        self, 
        data_port=None, 
        data_port_baudrate=115200,
        rtcm_port=None, 
        rtcm_port_baudrate=115200,
        heading_system='ned'):
        """
        Initialize the UM982 interface.
        
        Args:
            data_port: Serial port for data communication
            data_port_baudrate: Baud rate for data port
            rtcm_port: Serial port for RTCM corrections
            rtcm_port_baudrate: Baud rate for RTCM port
            world: Heading/Orientation reference frame ('std', 'enu', 'ned')
        """
        self.data_port = data_port
        self.data_port_baudrate = data_port_baudrate
        self.rtcm_port = rtcm_port
        self.rtcm_port_baudrate = rtcm_port_baudrate
        if heading_system not in ['enu', 'ned']:
            raise ValueError("heading_system must be 'enu' or 'ned'")
        self.heading_system = heading_system
        
        self.running = False
        self._lock = threading.Lock()  # For thread-safe access
        self._message_count = 0
        self._last_message_time = 0
        
        try:
            # Configure serial for data
            self.data_serial = serial.Serial(
                port=self.data_port,
                baudrate=self.data_port_baudrate,
                timeout=1.0
            )
            
            # Configure serial for RTCM
            self.rtcm_serial = serial.Serial(
                port=self.rtcm_port,
                baudrate=self.rtcm_port_baudrate,
                timeout=1.0
            ) if rtcm_port else None
            
            # Initialize thread for data reading
            self.data_port_thread = threading.Thread(target=self._data_rx_thread, daemon=True)
            logger.info(f"UM982 initialized on {self.data_port} and {self.rtcm_port}")
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to initialize serial ports: {e}")
    
    def __enter__(self):
        """Support for 'with' statement."""
        self.open()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Clean up resources when exiting context."""
        self.close()
    
    def open(self):
        """Open serial ports and start listening thread."""
        if not self.data_serial.is_open:
            try:
                self.data_serial.open()
            except serial.SerialException as e:
                raise RuntimeError(f"Failed to open data port: {e}")

        # Optional RTCM port
        if self.rtcm_serial and not self.rtcm_serial.is_open:
            try:
                self.rtcm_serial.open()
            except serial.SerialException as e:
                logger.warning(f"Failed to open RTCM port: {e}")

        # Start the reading thread
        self.running = True
        self.data_port_thread.start()
        
        time.sleep(1)
        
        # Send configuration commands
        self.write_config("gpgga 0.1\r\n")    # Global Positioning System Fix Data + rate Hz
        self.write_config("pvtslna 0.1\r\n")  # Position, Velocity, Time, Satellite Information + rate Hz
        # self.write_config("bestnava 1\r\n") # Best Position and Velocity
        # self.write_config("gphpr 1\r\n")    # Attitude Parameters
        
        # Initialize state variables with underscore prefix
        self._last_bestpos = None
        self._last_heading = None
        self._last_orientation = None
        self._last_vel = None
        self._last_nmea = None
        
    def close(self):
        """Safely stop the thread and close serial ports."""
        self.running = False  # Signal the thread to stop
        if hasattr(self, 'data_port_thread'):
            self.data_port_thread.join(timeout=2)  # Wait for the thread to finish
            
        if hasattr(self, 'data_serial') and self.data_serial.is_open:
            self.data_serial.close()
            
        if hasattr(self, 'rtcm_serial') and self.rtcm_serial and self.rtcm_serial.is_open:
            self.rtcm_serial.close()
        
    def write_config(self, data):
        """
        Write a command to the GNSS module.
        
        Args:
            data: Command string to send
        """
        if self.data_serial.is_open:
            self.data_serial.write(data.encode('utf-8'))  # Ensure data is encoded before sending

    def write_rtcm(self, data):
        """
        Send RTCM data to the receiver.
        
        Args:
            data: RTCM data to send
        """
        if self.rtcm_serial and self.rtcm_serial.is_open:
            self.rtcm_serial.write(data)

    def _data_rx_thread(self):
        """Thread for reading data from the serial port."""
        while self.running:
            try:
                frame = self.data_serial.readline().decode('utf-8').strip()
                if frame:
                    self._message_count += 1
                    self._last_message_time = time.time()
                    
                    # print(f"Received: {frame}")  # Debug output
                    
                    if frame.startswith("$command"):
                        pass
                    elif frame.startswith("$GNGGA") and nmea_crc(frame):
                        with self._lock:
                            self._last_nmea = frame
                    elif frame.startswith("#PVTSLNA") and nmea_expend_crc(frame):
                        bestpos, heading = pvtsln_solver(frame)
                        if self.heading_system == 'enu':
                            # NED to ENU conversion
                            heading[2] = self.ned_to_enu_deg(heading[2])
                        elif self.heading_system == 'ned':
                            heading[2] = heading[2]
                        with self._lock:
                            self._last_bestpos = bestpos
                            self._last_heading = heading
                    # elif frame.startswith("$GNHPR") and nmea_crc(frame):
                    #     with self._lock:
                    #         self._last_orientation = gnhpr_solver(frame)
                    # elif frame.startswith("#BESTNAVA") and nmea_expend_crc(frame):
                    #     with self._lock:
                    #         self._last_vel = bestnav_solver(frame)

            except Exception as e:
                logger.error(f"Error reading serial: {e}")
                self.running = False
                break  # Exit the loop if there's an error
    
    @staticmethod
    def ned_to_enu_deg(yaw_ned_deg: float) -> float:
        """NED → ENU yaw, both in degrees, wrapped to [0, 360)."""
        return (90.0 - yaw_ned_deg) % 360.0
    
    @property
    def bestpos(self):
        """Get and clear the latest position data."""
        with self._lock:
            value = self._last_bestpos
            self._last_bestpos = None
        return value
    
    @property
    def heading(self):
        """Get and clear the latest heading data."""
        with self._lock:
            value = self._last_heading
            self._last_heading = None
        return value
    
    @property
    def nmea(self):
        """Get and clear the latest NMEA data."""
        with self._lock:
            value = self._last_nmea
            self._last_nmea = None
        return value
        
    @property
    def status(self):
        """Get receiver status information."""
        return {
            "connected": self.running,
            "data_port": self.data_port,
            "rtcm_port": self.rtcm_port,
            "last_message_time": self._last_message_time,
            "messages_received": self._message_count
        }
    
    # For backwards compatibility
    def get_bestpos(self):
        return self.bestpos
    
    def get_heading(self):
        return self.heading
    
    def get_nmea(self):
        return self.nmea


def main():
    """Main function to demonstrate UM982 usage."""

    data_port = "/dev/ttyUSB0"  # Default fallback
    rtcm_port = "/dev/ttyUSB1"  # Default fallback
    
    # Use context manager for clean resource handling
    with UM982(data_port=data_port, rtcm_port=rtcm_port) as um982:
        print(f"UM982 initialized - Status: {um982.status}")
        try:
            while True:
                # Process data as it becomes available
                if pos := um982.bestpos:
                    print(f"Position: Lat {pos[2]}, Lon {pos[3]}, Height {pos[1]}")
                if hdg := um982.heading:
                    print(f"Heading: {hdg[2]} degrees")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Shutting down...")


if __name__ == '__main__':
    main()