# STM32UAVGazebo

## Introduction

**STM32UAVGazebo** is a UAV (Unmanned Aerial Vehicle) simulation project that integrates STM32 microcontroller hardware, the FlySky FS-i6 transmitter, and the FS-iA6B receiver in the **Gazebo Classic** simulation environment. PX4 provides simulated sensor data (IMU, GPS, barometer), while the STM32 board handles motor control and receiver signals. This project supports hardware-in-the-loop (HIL) testing, enabling validation of flight control algorithms before deploying them to real hardware.

> **Note:** This project does **not** use ROS.

### Key Features

- **Simulated sensor data:** IMU, GPS, and barometer data provided by PX4.
- **Motor control via STM32:** Receives control signals and outputs to ESCs.
- **Integration with FlySky FS-i6/FS-iA6B:** Remote control support in simulation.
- **HIL support:** Real hardware in the loop for controller validation.
- **Extensible:** Can be adapted for different UAV configurations.
- **Object tracking flight:** Supports flying the UAV to follow detected objects using the `vx+vy+vz.py` script.
- **For researchers and developers:** Ideal for learning, prototyping, and testing UAV systems.

---

## System Requirements

- Ubuntu 20.04 or later (recommended)
- **Gazebo Classic** (gazebo11)
- PX4 Autopilot (Firmware)
- FlySky FS-i6 transmitter & FS-iA6B receiver
- STM32 microcontroller (F4/F7/Ux series)
- STM32 programming tools (STM32CubeIDE, OpenOCD, etc.)
- USB/Serial cable for connecting STM32 to PC
- Python 3 (for running data forwarding and object tracking scripts)

---

## Setup & Installation

### 1. Clone the repository

```bash
git clone https://github.com/Inzaghi04/STM32UAVGazebo.git
cd STM32UAVGazebo
```

### 2. Install dependencies

**Gazebo Classic:**
```bash
sudo apt update
sudo apt install gazebo11
```

**PX4 Firmware:**
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
make px4_sitl_default gazebo
```

**Python requirements (for data forwarding and object tracking):**
```bash
pip3 install -r requirements.txt
```
> Run this in the same directory as the Python scripts if a `requirements.txt` is provided.

### 3. Connect STM32 to your PC

- Flash the STM32 firmware from the `/firmware` directory (see instructions provided in code or docs).
- Ensure the USB/Serial cable is properly connected.
- Verify device recognition (`lsusb` or `dmesg | grep tty`).

### 4. Setup FlySky FS-i6 & FS-iA6B receiver

- Connect the FS-iA6B receiver to STM32 according to the circuit diagram.
- Bind the transmitter to the receiver and confirm signals are received.

### 5. Run the simulation (with Iris UAV model)

**Start PX4 SITL with Gazebo Classic and Iris UAV:**
```bash
cd PX4-Autopilot
make px4_sitl gazebo_classic_iris
```

### 6. Forward data between simulation and STM32

**Run the data forwarding script:**

```bash
python3 forward_data.py
```
- This script handles communication between PX4/Gazebo and your STM32 board.
- Make sure any required configuration (e.g., serial port settings) inside `forward_data.py` matches your hardware setup.

### 7. Fly UAV to follow detected object

**Run the object tracking script:**

```bash
python3 vx+vy+vz.py
```
- This script enables the UAV to follow a detected object by sending appropriate velocity commands (vx, vy, vz).
- Ensure any required object detection modules or inputs are properly set up.

---

## Directory Structure

- `/firmware`: STM32 embedded source code.
- `/gazebo_models`: UAV models and simulation environment.
- `/docs`: Documentation and circuit diagrams.
- `forward_data.py`: Python script to forward data between PX4/Gazebo and STM32.
- `vx+vy+vz.py`: Python script to enable object tracking flight.

---

## References

- [PX4 Autopilot Documentation](https://docs.px4.io/)
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [Gazebo Classic Tutorials](https://classic.gazebosim.org/tutorials)
- [FlySky FS-i6 Manual](https://www.flysky-cn.com/)

---

## Contribution

Contributions in code, documentation, and ideas are welcome! Please open a pull request or contact via [Issues](https://github.com/Inzaghi04/STM32UAVGazebo/issues).

---

## License

MIT License. See the LICENSE file for details.

---

**Contact:**  
Author: [Inzaghi04](https://github.com/Inzaghi04)  
Email: (see profile for updates)