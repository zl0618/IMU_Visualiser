import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib.animation import FuncAnimation
import time
from imu_reader import IMUReader

class IMUVisualizer:
    def __init__(self, imu_reader):
        self.imu = imu_reader
        
        # Setup 3D plot
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(15, 10), facecolor='black')
        
        # 3D orientation plot
        self.ax3d = self.fig.add_subplot(221, projection='3d')
        self.ax3d.set_title('IMU 3D Orientation', color='white', fontsize=14)
        self.ax3d.set_xlim([-2, 2])
        self.ax3d.set_ylim([-2, 2])
        self.ax3d.set_zlim([-2, 2])
        self.ax3d.set_xlabel('X', color='white')
        self.ax3d.set_ylabel('Y', color='white')
        self.ax3d.set_zlabel('Z', color='white')
        self.ax3d.set_facecolor('black')
        
        # Accelerometer plot
        self.ax_acc = self.fig.add_subplot(222)
        self.ax_acc.set_title('Accelerometer (g)', color='white', fontsize=12)
        self.ax_acc.set_ylim([-2, 2])
        self.ax_acc.set_xlim([0, 100])
        self.ax_acc.set_facecolor('black')
        self.ax_acc.grid(True, alpha=0.3)
        
        # Gyroscope plot
        self.ax_gyro = self.fig.add_subplot(223)
        self.ax_gyro.set_title('Gyroscope (°/s)', color='white', fontsize=12)
        self.ax_gyro.set_ylim([-200, 200])
        self.ax_gyro.set_xlim([0, 100])
        self.ax_gyro.set_facecolor('black')
        self.ax_gyro.grid(True, alpha=0.3)
        
        # Angle plot
        self.ax_angle = self.fig.add_subplot(224)
        self.ax_angle.set_title('Orientation Angles (°)', color='white', fontsize=12)
        self.ax_angle.set_ylim([-180, 180])
        self.ax_angle.set_xlim([0, 100])
        self.ax_angle.set_facecolor('black')
        self.ax_angle.grid(True, alpha=0.3)
        
        # Initialize 3D coordinate system
        self.setup_3d_axes()
        
        # Data history for plots
        self.history_length = 100
        self.time_data = list(range(self.history_length))
        self.acc_history = {'x': [0]*self.history_length, 'y': [0]*self.history_length, 'z': [0]*self.history_length}
        self.gyro_history = {'x': [0]*self.history_length, 'y': [0]*self.history_length, 'z': [0]*self.history_length}
        self.angle_history = {'roll': [0]*self.history_length, 'pitch': [0]*self.history_length, 'yaw': [0]*self.history_length}
        
        # Initialize line plots
        self.acc_lines = {
            'x': self.ax_acc.plot(self.time_data, self.acc_history['x'], 'r-', label='X', alpha=0.8)[0],
            'y': self.ax_acc.plot(self.time_data, self.acc_history['y'], 'g-', label='Y', alpha=0.8)[0],
            'z': self.ax_acc.plot(self.time_data, self.acc_history['z'], 'b-', label='Z', alpha=0.8)[0]
        }
        self.ax_acc.legend()
        
        self.gyro_lines = {
            'x': self.ax_gyro.plot(self.time_data, self.gyro_history['x'], 'r-', label='X', alpha=0.8)[0],
            'y': self.ax_gyro.plot(self.time_data, self.gyro_history['y'], 'g-', label='Y', alpha=0.8)[0],
            'z': self.ax_gyro.plot(self.time_data, self.gyro_history['z'], 'b-', label='Z', alpha=0.8)[0]
        }
        self.ax_gyro.legend()
        
        self.angle_lines = {
            'roll': self.ax_angle.plot(self.time_data, self.angle_history['roll'], 'r-', label='Roll', alpha=0.8)[0],
            'pitch': self.ax_angle.plot(self.time_data, self.angle_history['pitch'], 'g-', label='Pitch', alpha=0.8)[0],
            'yaw': self.ax_angle.plot(self.time_data, self.angle_history['yaw'], 'b-', label='Yaw', alpha=0.8)[0]
        }
        self.ax_angle.legend()
        
        # Status text
        self.status_text = self.fig.text(0.02, 0.95, '', fontsize=10, color='white')
        
    def setup_3d_axes(self):
        """Setup 3D coordinate system representation"""
        # Clear previous axes
        self.ax3d.clear()
        self.ax3d.set_xlim([-2, 2])
        self.ax3d.set_ylim([-2, 2])
        self.ax3d.set_zlim([-2, 2])
        self.ax3d.set_xlabel('X', color='white')
        self.ax3d.set_ylabel('Y', color='white')
        self.ax3d.set_zlabel('Z', color='white')
        self.ax3d.set_title('IMU 3D Orientation', color='white', fontsize=14)
        
        # Draw origin point
        self.ax3d.scatter([0], [0], [0], color='white', s=50)
        
    def rotation_matrix(self, roll, pitch, yaw):
        """Create rotation matrix from Euler angles"""
        # Convert to radians
        roll = np.radians(roll)
        pitch = np.radians(pitch)
        yaw = np.radians(yaw)
        
        # Rotation matrices
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
        
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])
        
        return Rz @ Ry @ Rx
    
    def update_3d_orientation(self, roll, pitch, yaw):
        """Update 3D orientation visualization"""
        self.setup_3d_axes()
        
        # Get rotation matrix
        R = self.rotation_matrix(roll, pitch, yaw)
        
        # Define local coordinate system vectors
        x_axis = np.array([1.5, 0, 0])
        y_axis = np.array([0, 1.5, 0])
        z_axis = np.array([0, 0, 1.5])
        
        # Rotate axes
        x_rotated = R @ x_axis
        y_rotated = R @ y_axis
        z_rotated = R @ z_axis
        
        # Draw rotated coordinate system
        self.ax3d.quiver(0, 0, 0, x_rotated[0], x_rotated[1], x_rotated[2], 
                        color='red', arrow_length_ratio=0.1, linewidth=3, label='X')
        self.ax3d.quiver(0, 0, 0, y_rotated[0], y_rotated[1], y_rotated[2], 
                        color='green', arrow_length_ratio=0.1, linewidth=3, label='Y')
        self.ax3d.quiver(0, 0, 0, z_rotated[0], z_rotated[1], z_rotated[2], 
                        color='blue', arrow_length_ratio=0.1, linewidth=3, label='Z')
        
        # Add IMU body representation (simple box)
        # Box corners
        box_size = 0.3
        corners = np.array([[-box_size, -box_size, -box_size],
                           [box_size, -box_size, -box_size],
                           [box_size, box_size, -box_size],
                           [-box_size, box_size, -box_size],
                           [-box_size, -box_size, box_size],
                           [box_size, -box_size, box_size],
                           [box_size, box_size, box_size],
                           [-box_size, box_size, box_size]])
        
        # Rotate box
        rotated_corners = np.array([R @ corner for corner in corners])
        
        # Draw box edges
        edges = [(0,1), (1,2), (2,3), (3,0),  # bottom face
                 (4,5), (5,6), (6,7), (7,4),  # top face
                 (0,4), (1,5), (2,6), (3,7)]  # vertical edges
        
        for edge in edges:
            points = rotated_corners[list(edge)]
            self.ax3d.plot3D(*points.T, 'cyan', alpha=0.6, linewidth=1)
    
    def update_plots(self, acc, gyro, angle):
        """Update time series plots"""
        # Shift data left
        for key in self.acc_history:
            self.acc_history[key] = self.acc_history[key][1:] + [0]
            self.gyro_history[key] = self.gyro_history[key][1:] + [0]
        
        for key in self.angle_history:
            self.angle_history[key] = self.angle_history[key][1:] + [0]
        
        # Add new data
        self.acc_history['x'][-1] = acc[0]
        self.acc_history['y'][-1] = acc[1]
        self.acc_history['z'][-1] = acc[2]
        
        self.gyro_history['x'][-1] = gyro[0]
        self.gyro_history['y'][-1] = gyro[1]
        self.gyro_history['z'][-1] = gyro[2]
        
        self.angle_history['roll'][-1] = angle[0]
        self.angle_history['pitch'][-1] = angle[1]
        self.angle_history['yaw'][-1] = angle[2]
        
        # Update line plots
        for key, line in self.acc_lines.items():
            line.set_ydata(self.acc_history[key])
        
        for key, line in self.gyro_lines.items():
            line.set_ydata(self.gyro_history[key])
        
        for key, line in self.angle_lines.items():
            line.set_ydata(self.angle_history[key])
    
    def update(self, frame):
        """Animation update function"""
        if self.imu.read_data() and self.imu.has_new_data():
            # Get latest data
            acc = self.imu.get_accelerometer()
            gyro = self.imu.get_gyroscope()
            angle = self.imu.get_angle()
            
            # Update visualizations
            self.update_3d_orientation(angle[0], angle[1], angle[2])
            self.update_plots(acc, gyro, angle)
            
            # Update status
            status = f"Roll: {angle[0]:6.1f}°  Pitch: {angle[1]:6.1f}°  Yaw: {angle[2]:6.1f}°\n"
            status += f"Acc: [{acc[0]:5.2f}, {acc[1]:5.2f}, {acc[2]:5.2f}] g\n"
            status += f"Gyro: [{gyro[0]:6.1f}, {gyro[1]:6.1f}, {gyro[2]:6.1f}] °/s"
            self.status_text.set_text(status)
    
    def start(self):
        """Start the visualization"""
        ani = FuncAnimation(self.fig, self.update, interval=50, cache_frame_data=False)
        plt.tight_layout()
        plt.show()

def main():
    # Try to connect to IMU
    possible_ports = ['COM4', 'COM1', 'COM3', 'COM5']
    
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
    print("Starting IMU visualization... (Close window to stop)")
    
    try:
        # Create and start visualizer
        visualizer = IMUVisualizer(imu)
        visualizer.start()
    except KeyboardInterrupt:
        print("\nVisualization stopped by user")
    finally:
        imu.disconnect()

if __name__ == "__main__":
    main()
