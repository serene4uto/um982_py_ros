import serial
import serial.tools.list_ports
import threading
import time
import math


def crc_table():
    table = []
    for i in range(256):
        crc = i
        for j in range(8, 0, -1):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
        table.append(crc)
    return table


NMEA_EXPEND_CRC_TABLE = crc_table()


def nmea_expend_crc(nmea_expend_sentence):
    def calculate_crc32(data):
        crc = 0
        for byte in data:
            crc = NMEA_EXPEND_CRC_TABLE[(crc ^ byte) & 0xFF] ^ (crc >> 8)
        return crc & 0xFFFFFFFF

    try:
        sentence, crc = nmea_expend_sentence[1:].split("*")
        crc = crc[:8]
    except:
        return False
    calculated_crc = calculate_crc32(sentence.encode())
    return crc.lower() == format(calculated_crc, '08x')


def nmea_crc(nmea_sentence):
    try:
        sentence, crc = nmea_sentence[1:].split("*")
        crc = crc[:2]
    except:
        return False
    calculated_checksum = 0
    for char in sentence:
        calculated_checksum ^= ord(char)
    calculated_checksum_hex = format(calculated_checksum, 'X')
    return calculated_checksum_hex.zfill(2) == crc.upper()


def msg_seperate(msg:str):
    header, body = msg[:msg.find('*')].split(';')
    
    return {
        'header': header,
        'body': body.split(',')
    }


def PVTSLN_solver(msg:str):
    parts = msg_seperate(msg)
    bestpos_type   = str(parts['body'][0])
    bestpos_hgt    = float(parts['body'][1])  
    bestpos_lat    = float(parts['body'][2])  
    bestpos_lon    = float(parts['body'][3])  
    bestpos_hgtstd = float(parts['body'][4])  
    bestpos_latstd = float(parts['body'][5])  
    bestpos_lonstd = float(parts['body'][6])  
    
    heading_type = str(parts['body'][20])
    heading_len  = float(parts['body'][21])
    heading_deg  = float(parts['body'][21])
    heading_pitch = float(parts['body'][22])
    
    bestpos = (bestpos_type, bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd)
    heading = (heading_type, heading_len, heading_deg, heading_pitch)
    return bestpos, heading


def GNHPR_solver(msg:str):
    parts = msg_seperate(msg)
    heading = float(parts[3-1])
    pitch   = float(parts[4-1])
    roll    = float(parts[5-1])
    orientation = (heading, pitch, roll)
    return orientation


def BESTNAV_solver(msg:str):
    parts = msg_seperate(msg)
    vel_hor_std = float(parts[-1])  
    vel_ver_std = float(parts[-2]) 
    vel_ver     = float(parts[-3])
    vel_heading = float(parts[-4]) 
    vel_hor     = float(parts[-5])
    vel_north   = vel_hor * math.cos(math.radians(vel_heading))
    vel_east    = vel_hor * math.sin(math.radians(vel_heading))
    return (vel_east, vel_north, vel_ver, vel_hor_std, vel_hor_std, vel_ver_std)


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
        self.running = False  # Flag to manage the thread safely

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
        )

        self.data_port_thread = threading.Thread(target=self._data_rx_thread, daemon=True)
    
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
