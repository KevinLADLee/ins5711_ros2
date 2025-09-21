# pbox_node

ROS2 package for interfacing with INS5711 inertial navigation system.

## Overview

This package provides a ROS2 node that communicates with the INS5711 inertial navigation system through serial communication. It decodes various protocol formats and publishes sensor data to ROS2 topics.

## Features

- Supports multiple communication protocols (BDDB0A, BDDB04, BDDB8B, BDDB0B, BDDB10, BDDB1B)
- Publishes sensor data including IMU, GNSS, and odometry information
- Supports both ROS2 and ROS1 platforms
- Configurable message types and protocol formats

## Packages

- `pbox_node`: Main node package for INS5711 communication
- `imu_msgs`: Custom message definitions for sensor data

## Supported Messages

- `Imu0A`: IMU data with gyroscope, accelerometer and temperature
- `Imu04`: IMU data (variant)
- `Imu8B`: IMU data (variant)
- `Gnss`: GNSS position and navigation data
- `Odom`: Odometry data
- `ASENSING`: Asensing protocol data
- `Imu`: Standard IMU message
- `ImuInitial`: Initial IMU data

## Usage

1. Connect your INS5711 device via USB/serial
2. Launch the node using the provided launch file:
   ```bash
   ros2 launch pbox_node pbox_node.launch.py
   ```
3. Subscribe to the available topics for sensor data

## Build

```bash
colcon build --packages-select pbox_node imu_msgs
```

## Parameters

- `serial_port`: Serial port device path (default: `/dev/ttyUSB0`)
- `baud_rate`: Serial communication baud rate (default: `115200`)
- `msg_switch`: Message type selection (default: `1`)
- `protocol_type`: Protocol type selection (default: `0`)

## Topics

### Published Topics
- `/Imu0A`: IMU data from BDDB0A protocol
- `/Imu04`: IMU data from BDDB04 protocol
- `/Imu8B`: IMU data from BDDB8B protocol
- `/Gnss`: GNSS data
- `/Odom`: Odometry data
- `/Imu`: Standard IMU message

### Subscribed Topics
- None

## Launch Files

- `pbox_node.launch.py`: Main launch file for the node
- `pbox_node.launch`: Legacy launch file for ROS1

## Build Dependencies

- `rclcpp`
- `sensor_msgs`
- `imu_msgs`
- `std_msgs`
- `nav_msgs`
- `serial`

## License

TODO: License declaration