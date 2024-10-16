# ROS teleop_twist_joy Example

Make sure you have the following packages installed:

- `joy`: This package is used to interface with the joystick/gamepad.

- `teleop_twist_joy`: This package is used to map joystick inputs to `cmd_vel` commands.

Check if already installed

```bash
rospack find ros-noetic-joy 
rospack find ros-noetic-teleop-twist-joy
```

You can install these packages using:

```bash
sudo apt-get install ros-noetic-joy 
sudo apt-get install ros-noetic-teleop-twist-joy
```

![img](https://articulatedrobotics.xyz/assets/images/joy_node-717e780db7f44957b5fd3c33a3589058.png)

## Installation guide

http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

```bash
roslaunch teleop_twist_joy_example teleop_joy.launch
```

```bash
rostopic echo /joy
```

```bash
rostopic echo /cmd_vel
```

> NOTE: to enable the controller for `/cmd_vel` messages commands
>
> >  INFO] [1728982511.685182765]: Teleop enable button 3.
>
> which is the `Y`.

## `joy`

The `joy` package in ROS contains a generic driver that allows ROS to read information from most standard gamepads and joysticks. 

#### How the `joy` Package Works

- **Generic Joystick Driver**: The `joy` package acts as a bridge between the Linux joystick interface and ROS. It doesn't include specific drivers for individual gamepad models; instead, it uses the Linux kernel's built-in joystick support, which handles a wide variety of gamepads and joysticks.
- **Accessing Linux Joystick Interface**: The Linux kernel provides a uniform interface for gamepads and joysticks via the `/dev/input/jsX` devices (where `X` is a number starting from 0). The `joy` package reads data from the Linux joystick interface and converts the raw input into ROS messages and publish into the `/joy` topic as `sensor_msgs/Joy` messages. These messages include:
  - **Axes Data**: Values for analog controls (e.g., thumbsticks, triggers). The values range from -1.0 to 1.0.
  - **Buttons Data**: States for digital controls (e.g., push buttons), represented as 0 (not pressed) or 1 (pressed).

#### Why You Don't Need Separate Drivers for Most Gamepads

- Since the Linux kernel already has support for most gamepads, no additional drivers are usually needed.
- The `joy` package leverages this existing support to work with a wide range of gamepads, including Xbox, PlayStation, Logitech, and generic USB gamepads.

#### Documentation

> http://wiki.ros.org/joy

## `teleop_twist_joy`

- **Purpose**: It listens for joystick inputs (buttons and axes) published on the `/joy` topic and converts these inputs into velocity commands (specifically, `geometry_msgs/Twist` messages published to the `/cmd_vel` topic).
- **Why `cmd_vel`?**: In ROS, the `cmd_vel` topic is the standard topic name used to send velocity commands to **mobile robots**.

#### Alternative Packages

There are other ROS packages for translating joystick inputs into various message types or robot-specific controls. Here are a few examples:

- **`teleop_joy`**: A generic package that provides configurable mappings from joystick inputs to different ROS message types (not just `Twist`).
- **`ros_control` and `controller_manager`**: Use joystick inputs to send commands to actuators (If you're controlling an arm, you might need to convert joystick inputs into `sensor_msgs/JointState`).
- **Custom Nodes**: If you need a specific translation not covered by existing packages, you can create a custom node that subscribes to the `/joy` topic and publishes the desired message type.

#### Documentation

> http://wiki.ros.org/teleop_twist_joy


## NOTES: Sensors

1. **Sensor Connection**:
   - When a sensor (like a serial device, such as an IMU or a GPS) is connected to the PC, it is typically recognized as a device under `/dev`, such as `/dev/ttyACM0`, `/dev/ttyUSB0`, or similar.
   - The naming convention often depends on the type of interface the sensor uses (e.g., USB, serial).
2. **Linux Device Driver**:
   - Linux has built-in support for many types of devices. When you connect a sensor, the appropriate device driver in the Linux kernel handles the low-level communication with the hardware.
   - This includes opening the serial port, reading data, and managing data flow.
3. **ROS Sensor Driver Node**:
   - A ROS package (often referred to as a **sensor driver**) acts as a bridge between the Linux sensor interface and the ROS framework.
   - This node communicates with the sensor through the appropriate device file (e.g., `/dev/ttyACM0`), reads the sensor data, and then publishes that data to a specific ROS topic, like `/imu_data` (for an IMU sensor) or `/gps_data` (for GPS).


## NOTES: Motors

1. **ROS package**:
   - A ROS package (often referred to as a **motor driver node**) serves as a bridge between the Linux motor interface and the ROS framework.
2. **ROS package node**:
   - This node subscribes to specific ROS topics (e.g., `/cmd_vel` for velocity commands), translates ROS messages into control signals that the motor driver understands, and sends these commands to the motor controller, allowing for effective control of the robot's movement.
3. **Motor Connection**:
   - Depending on the motor driver, the Linux kernel may manage the communication through serial ports, GPIO pins, or other protocols, allowing the ROS system to send commands to control the motors.
