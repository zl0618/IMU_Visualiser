# Real-time IMU 3D Visualization

A comprehensive real-time visualization system for IMU (Inertial Measurement Unit) sensors with 3D orientation display and live data plots.

![IMU Visualization Demo](https://img.shields.io/badge/Python-3.x-blue) ![Matplotlib](https://img.shields.io/badge/Matplotlib-3D-green) ![Serial](https://img.shields.io/badge/PySerial-Real--time-orange)

## Features

🎯 **Real-time 3D Orientation** - Visual IMU body with rotating coordinate axes
📊 **Live Data Plots** - Accelerometer, gyroscope, and orientation angles
🎨 **Dark Theme Interface** - Professional 4-panel matplotlib layout
🔌 **Auto Port Detection** - Automatic COM port scanning and connection
🛠️ **Raw Data Tools** - Debug tools for packet analysis
⚡ **High Performance** - 50ms update intervals for smooth visualization

## Hardware Compatibility

- WT901 IMU sensors
- Similar UART IMU sensors with 0x55 packet headers
- 9600 baud rate (configurable)
- Standard USB-to-Serial connections

## Installation

```bash
pip install matplotlib numpy pyserial
```

## Usage

### Quick Start
```bash
python imu_visualizer.py
```

### Raw Data Monitoring
```bash
python raw_imu_data.py
```

### Basic IMU Reading
```bash
python imu_reader.py
```

## File Structure

- `imu_visualizer.py` - Main 3D visualization application
- `imu_reader.py` - Core IMU data reader class
- `raw_imu_data.py` - Raw data monitoring and debugging
- `testport.py` - Port detection utility

## Visualization Panels

1. **3D Orientation** - Real-time IMU body with coordinate axes
2. **Accelerometer** - X, Y, Z acceleration in g-forces
3. **Gyroscope** - X, Y, Z rotation rates in °/s
4. **Orientation Angles** - Roll, Pitch, Yaw in degrees

## Data Format

The system expects IMU packets with:
- Header: `0x55`
- Packet types: `0x51` (Acc), `0x52` (Gyro), `0x53` (Angle), `0x54` (Mag)
- Little-endian 16-bit signed integers
- Standard WT901 protocol

## Perfect For

- 🤖 Robotics projects and navigation systems
- 🎮 Motion tracking and gesture recognition
- 📱 Sensor testing and calibration
- 🎓 Educational IMU demonstrations
- 🔬 Research and development

## Contributing

Feel free to submit issues, fork the repository, and create pull requests for any improvements.

## License

Open source - feel free to use in your projects!

---

*Real-time IMU visualization made simple* ✨
