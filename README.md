# Active Neck Control System

This repository contains a setup for controlling a 2 DoF (yaw, pitch) robotic neck using a Meta Quest 2 or 3. The successfull mapping of the operator's head's movements to the robot's neck and the real-time visual feedback is achieved by using an integration of of ROS, two PID controllers, camera streaming, with Unity via UDP and TCP/IP communication.

---
### Dependencies
- [ROS PID package](https://wiki.ros.org/pid) --> Used to implement the PID controllers for pitch and yaw motors.
- [RealSense ROS package](https://github.com/IntelRealSense/realsense-ros) --> To launch the camera and stream the video to a ROS topic.
- [UDP ROS package](https://github.com/AlfaKeNTAvR/udp_ros) --> To stream the camera video to Unity using UDP.
- [UNITY ROS package](https://github.com/AlfaKeNTAvR/unity_ros) --> To communication with Unity over TCP/IP.
- [OCULUS ROS package](https://github.com/AlfaKeNTAvR/oculus_ros) --> To receive the headset pose and converting it into the correct input for PIDs. (From left-hand coordinate system used in Unity to right-hand used in ROS)
---

## PID Control

### Implementation
The PID controllers for pitch and yaw motors are implemented using the [ROS PID package](https://wiki.ros.org/pid), as specified in the active_neck.launch file.

---

## Camera Integration

### RealSense Camera
The camera integration is managed using the [RealSense ROS package](https://github.com/IntelRealSense/realsense-ros).

### Workflow
1. The RealSense camera node communicates with the camera and publishes the image stream to a ROS topic.
2. The image stream is received by the [`udp_ros` node](https://github.com/AlfaKeNTAvR/udp_ros).
3. The `udp_ros` node transmits the image stream to Unity using UDP.

---

## Unity Integration

### Headset Pose
The integration with Unity includes communication for reading and converting the headset pose:
- **Script and Launch File**: Receives the headset pose from Unity.
- **Coordinate System Conversion**: Converts the pose from a left-hand coordinate system to a right-hand coordinate system.
- **Published ROS Topics**:
  - Headset pose
  - Setpoints (absolute pitch and yaw in degrees) for the PIDs to compute control efforts.

**Note:** The code for the headset pose reading is not yet committed.

---

## Launch Instructions
To operate the system, launch the following nodes:

1. **`active_neck.launch`**:
   - Starts the active neck script.
   - Launches the two PIDs for motor control.

2. **RealSense camera node**:
   - Handles image streaming from the camera.

3. **`udp_ros` node**:
   - Transmits the camera stream to Unity.

4. **Unity ROS node**:
   - Facilitates communication with Unity over TCP/IP.
  
5. **Oculus ROS node**:
- Receive the headset pose and converts it into the correct input for PIDs.

---

## Notes
- Ensure all dependencies are installed and configured properly.
- Verify network connections for seamless UDP and TCP/IP communication.

For more information, refer to the respective package documentation linked above.

