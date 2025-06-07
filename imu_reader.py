import serial
import time
import struct
import numpy as np

class IMUReader:
    def __init__(self, port='COM4', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.buffer = bytearray()
        
        # IMU data storage
        self.acc = [0.0, 0.0, 0.0]      # Accelerometer [g]
        self.gyro = [0.0, 0.0, 0.0]     # Gyroscope [°/s]
        self.angle = [0.0, 0.0, 0.0]    # Angle [°]
        self.mag = [0, 0, 0]            # Magnetometer [raw]
        
        # Data update flags
        self.acc_updated = False
        self.gyro_updated = False
        self.angle_updated = False
        self.mag_updated = False
        
    def connect(self):
        """Connect to IMU sensor"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"Connected to IMU on {self.port} at {self.baudrate} baud")
            time.sleep(1)  # Wait for connection to stabilize
            return True
        except Exception as e:
            print(f"Failed to connect to IMU: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from IMU sensor"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("IMU disconnected")
    
    def read_data(self):
        """Read and parse IMU data"""
        if not self.ser or not self.ser.is_open:
            return False
        
        try:
            # Read available data
            if self.ser.in_waiting > 0:
                new_data = self.ser.read(self.ser.in_waiting)
                self.buffer.extend(new_data)
                
                # Process complete packets
                self._process_buffer()
                return True
        except Exception as e:
            print(f"Error reading IMU data: {e}")
            return False
        
        return False
    
    def _process_buffer(self):
        """Process buffer data and extract IMU readings"""
        while len(self.buffer) >= 11:  # Minimum packet size
            # Look for packet header (0x55)
            header_pos = self.buffer.find(0x55)
            if header_pos == -1:
                self.buffer.clear()
                break
            elif header_pos > 0:
                # Remove data before header
                self.buffer = self.buffer[header_pos:]
            
            if len(self.buffer) < 11:
                break
            
            # Parse packet type
            packet_type = self.buffer[1]
            
            if packet_type == 0x51:  # Accelerometer data
                self._parse_accelerometer()
            elif packet_type == 0x52:  # Gyroscope data
                self._parse_gyroscope()
            elif packet_type == 0x53:  # Angle data
                self._parse_angle()
            elif packet_type == 0x54:  # Magnetometer data
                self._parse_magnetometer()
            
            # Remove processed packet
            self.buffer = self.buffer[11:]
    
    def _parse_accelerometer(self):
        """Parse accelerometer data packet"""
        if len(self.buffer) >= 11:
            # Extract raw values (little endian)
            ax_raw = struct.unpack('<h', self.buffer[2:4])[0]
            ay_raw = struct.unpack('<h', self.buffer[4:6])[0]
            az_raw = struct.unpack('<h', self.buffer[6:8])[0]
            
            # Convert to g (gravity units)
            self.acc[0] = ax_raw / 32768.0 * 16.0  # ±16g range
            self.acc[1] = ay_raw / 32768.0 * 16.0
            self.acc[2] = az_raw / 32768.0 * 16.0
            
            self.acc_updated = True
    
    def _parse_gyroscope(self):
        """Parse gyroscope data packet"""
        if len(self.buffer) >= 11:
            # Extract raw values (little endian)
            gx_raw = struct.unpack('<h', self.buffer[2:4])[0]
            gy_raw = struct.unpack('<h', self.buffer[4:6])[0]
            gz_raw = struct.unpack('<h', self.buffer[6:8])[0]
            
            # Convert to degrees/second
            self.gyro[0] = gx_raw / 32768.0 * 2000.0  # ±2000°/s range
            self.gyro[1] = gy_raw / 32768.0 * 2000.0
            self.gyro[2] = gz_raw / 32768.0 * 2000.0
            
            self.gyro_updated = True
    
    def _parse_angle(self):
        """Parse angle data packet"""
        if len(self.buffer) >= 11:
            # Extract raw values (little endian)
            roll_raw = struct.unpack('<h', self.buffer[2:4])[0]
            pitch_raw = struct.unpack('<h', self.buffer[4:6])[0]
            yaw_raw = struct.unpack('<h', self.buffer[6:8])[0]
            
            # Convert to degrees
            self.angle[0] = roll_raw / 32768.0 * 180.0   # Roll
            self.angle[1] = pitch_raw / 32768.0 * 180.0  # Pitch
            self.angle[2] = yaw_raw / 32768.0 * 180.0    # Yaw
            
            self.angle_updated = True
    
    def _parse_magnetometer(self):
        """Parse magnetometer data packet"""
        if len(self.buffer) >= 11:
            # Extract raw values (little endian)
            hx_raw = struct.unpack('<h', self.buffer[2:4])[0]
            hy_raw = struct.unpack('<h', self.buffer[4:6])[0]
            hz_raw = struct.unpack('<h', self.buffer[6:8])[0]
            
            # Store raw magnetometer values
            self.mag[0] = hx_raw
            self.mag[1] = hy_raw
            self.mag[2] = hz_raw
            
            self.mag_updated = True
    
    def get_accelerometer(self):
        """Get accelerometer data in g"""
        self.acc_updated = False
        return self.acc.copy()
    
    def get_gyroscope(self):
        """Get gyroscope data in degrees/second"""
        self.gyro_updated = False
        return self.gyro.copy()
    
    def get_angle(self):
        """Get angle data in degrees [roll, pitch, yaw]"""
        self.angle_updated = False
        return self.angle.copy()
    
    def get_magnetometer(self):
        """Get magnetometer raw data"""
        self.mag_updated = False
        return self.mag.copy()
    
    def has_new_data(self):
        """Check if any new data is available"""
        return (self.acc_updated or self.gyro_updated or 
                self.angle_updated or self.mag_updated)

def main():
    """Test the IMU reader"""
    # Try different ports - change as needed
    possible_ports = ['COM4', 'COM1']
    
    imu = None
    for port in possible_ports:
        print(f"Trying to connect to {port}...")
        imu = IMUReader(port=port, baudrate=9600)
        if imu.connect():
            break
        else:
            imu = None
    
    if not imu:
        print("Failed to connect to IMU on any port!")
        return
    
    print("IMU connected successfully!")
    print("Reading IMU data... (Press Ctrl+C to stop)")
    print("-" * 60)
    print("ACC(g)         GYRO(°/s)      ANGLE(°)       MAG(raw)")
    print("-" * 60)
    
    try:
        while True:
            # Read data from IMU
            if imu.read_data():
                # Print data when available
                if imu.has_new_data():
                    acc = imu.get_accelerometer()
                    gyro = imu.get_gyroscope()
                    angle = imu.get_angle()
                    mag = imu.get_magnetometer()
                    
                    print(f"X:{acc[0]:6.3f}     X:{gyro[0]:7.1f}     R:{angle[0]:6.1f}     X:{mag[0]:5d}")
                    print(f"Y:{acc[1]:6.3f}     Y:{gyro[1]:7.1f}     P:{angle[1]:6.1f}     Y:{mag[1]:5d}")
                    print(f"Z:{acc[2]:6.3f}     Z:{gyro[2]:7.1f}     Y:{angle[2]:6.1f}     Z:{mag[2]:5d}")
                    print("-" * 60)
            
            time.sleep(0.05)  # 20Hz update rate
            
    except KeyboardInterrupt:
        print("\nStopping IMU reader...")
    finally:
        imu.disconnect()

if __name__ == "__main__":
    main()
