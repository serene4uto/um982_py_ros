import serial
import serial.tools.list_ports
import threading
import time
import math


# Pre-compute CRC table for better performance
NMEA_EXPEND_CRC_TABLE = [
    (i >> 1) ^ (0xEDB88320 if i & 1 else 0) for i in range(256)
    for _ in range(7)
]


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
    except:
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
    except:
        return False


def msg_seperate(msg):
    """Parse NMEA message into header and body components."""
    parts = msg[:msg.find('*')].split(';')
    return {
        'header': parts[0],
        'body': parts[1].split(',') if len(parts) > 1 else []
    }


def PVTSLN_solver(msg):
    """Parse PVTSLN message into position and heading data."""
    parts = msg_seperate(msg)
    body = parts['body']
    
    # Unpack position data in one operation
    bestpos = (
        body[0],                # type
        float(body[1]),         # height
        float(body[2]),         # latitude
        float(body[3]),         # longitude
        float(body[4]),         # height std
        float(body[5]),         # latitude std
        float(body[6])          # longitude std
    )
    
    # Unpack heading data in one operation
    heading = (
        body[20],               # type
        float(body[21]),        # length
        float(body[21]),        # degrees
        float(body[22])         # pitch
    )
    
    return bestpos, heading


def GNHPR_solver(msg):
    """Parse GNHPR message for orientation data."""
    parts = msg_seperate(msg)
    body = parts['body']
    
    # Return heading, pitch, roll tuple
    return (float(body[2]), float(body[3]), float(body[4]))


def BESTNAV_solver(msg):
    """Parse BESTNAV message for velocity data."""
    parts = msg_seperate(msg)
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
    def __init__(
        self, 
        data_port=None, 
        data_port_baudrate=115200,
        rtcm_port=None, 
        rtcm_port_baudrate=115200):
        
        self.data_port = data_port
        self.data_port_baudrate = data_port_baudrate
        self.rtcm_port = rtcm_port
        self.rtcm_port_baudrate = rtcm_port_baudrate
        self.running = False
        
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
    
            self.data_port_thread = threading.Thread(target=self._data_rx_thread, daemon=True)
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to initialize serial ports: {e}")
    
    def open(self):
        """ Open serial ports and start listening thread """
        if not self.data_serial.is_open:
            self.data_serial.open()

        # Start the reading thread
        self.running = True
        self.data_port_thread.start()
        
        time.sleep(1)
        
        # Send GPGGA configuration command
        self.write_config("gpgga 1\r\n") # Global Positioning System Fix Data
        self.write_config("pvtslna 1\r\n") # Position, Velocity, Time, Satellite Information
        # self.write_config("bestnava 1\r\n") # Best Position and Velocity
        # self.write_config("gphpr 1\r\n") # Attitude Parameters
        
        self.last_fix_ = None
        self.last_orientation_ = None
        self.last_vel_ = None
        self.last_nmea_ = None
        
    def close(self):
        """ Safely stop the thread and close serial ports """
        self.running = False  # Signal the thread to stop
        self.data_port_thread.join(timeout=2)  # Wait for the thread to finish
        if self.data_serial.is_open:
            self.data_serial.close()
        if self.rtcm_serial.is_open:
            self.rtcm_serial.close()
        
    def write_config(self, data):
        """ Write a command to the GNSS module """
        if self.data_serial.is_open:
            self.data_serial.write(data.encode('utf-8'))  # Ensure data is encoded before sending

    def write_rtcm(self, data):
        """ Send RTCM data """
        if self.rtcm_serial.is_open:
            self.rtcm_serial.write(data)

    def _data_rx_thread(self):
        """ Thread for reading data from the serial port """
        while self.running:
            try:
                frame = self.data_serial.readline().decode('utf-8').strip()
                if frame:
                    if frame.startswith("$command"):
                        pass
                    if frame.startswith("$GNGGA") and nmea_crc(frame):
                        self.last_nmea_ = frame
                    if frame.startswith("#PVTSLNA") and nmea_expend_crc(frame):
                        self.last_bestpos_, self.last_heading_ = PVTSLN_solver(frame)
                    # if frame.startswith("$GNHPR") and nmea_crc(frame):
                    #     self.last_orientation_ = GNHPR_solver(frame)
                    # if frame.startswith("#BESTNAVA") and nmea_expend_crc(frame):
                    #     self.last_vel_ = BESTNAV_solver(frame)

            except Exception as e:
                print(f"Error reading serial: {e}")
                self.running = False
                break  # Exit the loop if there's an error
    
    def get_bestpos(self):
        bestpos = self.last_bestpos_
        self.last_bestpos_ = None
        return bestpos
    
    def get_heading(self):
        heading = self.last_heading_
        self.last_heading_ = None
        return heading
    
    def get_nmea(self):
        nmea = self.last_nmea_
        self.last_nmea_ = None
        return nmea

def main():
    um982 = UM982(data_port="/dev/ttyUSB0", rtcm_port="/dev/ttyUSB1")
    um982.open()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down...")
        um982.close()

if __name__ == '__main__':
    main()
