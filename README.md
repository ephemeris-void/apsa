# APSA - Autonomous Perception and Sensor Agent

**Board**       : NUCLEO-H755ZI-Q (STM32H755ZI, Cortex-M7 480MHz + Cortex-M4 240MHz)  
**Sensor Node** : ESP32-S3 (240MHz, 8MB flash, 8MB PSRAM)  
**RTOS**        : Zephyr v4.1.0  
**Status**      : Production-ready  

---

## The Problem This Project Solves

In a drone, if the IMU fails mid-flight, the flight controller loses all attitude and motion data. The drone crashes.

The standard solution is hardware redundancy - a second physical IMU. APSA adds a software fallback: when the IMU fails, a CNN autoencoder reconstructs IMU output from the last 512 samples in its history buffer and feeds it into the sensor pipeline as if the IMU were still running. This keeps the drone flying for up to 3 minutes while attempting recovery.

The key constraint: the AI output is not trusted blindly. Every reconstructed sample is checked against physics constraints before entering the sensor queue. Outputs that are all-zero, saturated, or whose acceleration magnitude falls outside the physically valid range are rejected and treated as a sensor failure. If the AI itself fails, the drone lands immediately. If 3 minutes pass without IMU recovery, the drone also lands safely.

The AI fallback activated, produced physically valid output, and the system recovered cleanly when the IMU was restored.

---

## State Machine

| IMU | AI | State | Behavior |
|-----|----|-------|----------|
| OK | OK | MONITOR / FUSION | Normal flight |
| OK | FAIL | MONITOR | AI offline, IMU flying |
| FAIL | OK | VIRTUALIZE -> FUSION | AI takes over |
| FAIL | OK, >3 min | LANDING | AI timeout, safe landing |
| FAIL | FAIL | EMERGENCY -> LANDING | Both failed, land now |
| Recovers | any | RECOVERY -> MONITOR | Back to normal |

---

## Hardware

### STM32 - Sensors

| Sensor | Interface | Pins |
|--------|-----------|------|
| IMU - ICM-42688-P | SPI1 | SCK=PA5, MISO=PA6, MOSI=PB5, CS=PD14 |
| MAG - QMC5883P | I2C4 | SCL=PF14, SDA=PF15, addr=0x2C |
| BARO - DPS310 | I2C1 | SCL=PB8, SDA=PB9, addr=0x77 |
| GPS - NEO-M8N | USART1 | TX=PB6, RX=PB7, 9600 baud |
| LoRa - DFRobot LWNode 915MHz | UART4 | TX=PC10, RX=PC11, 9600 baud |

### STM32 - Communication

| Port | Interface | Pins | Purpose |
|------|-----------|------|---------|
| Console | USART3 | TX=PD8, RX=PD9 | Serial log, 115200 baud, ST-LINK VCP |
| micro-ROS | USART6 | TX=PC6, RX=PC7 | ROS2 agent, 115200 baud, FTDI |

### STM32 - LEDs

| LED | Pin |
|-----|-----|
| LD1 Green | PB0 |
| LD2 Yellow | PE1 |
| LD3 Red | PB14 |

### ESP32-S3 - Sensors

| Sensor | Interface | Pins |
|--------|-----------|------|
| Radar C4001 | Serial2 | RX=9, TX=8, 9600 baud |
| Ultrasonic HC-SR04 | GPIO | TRIG=15, ECHO=16 |
| Obstacle S1-S5 | GPIO | S1=10, S2=11, S3=12, S4=13, S5=14 |
| Clip sensor | GPIO | pin 5 |
| Near sensor | GPIO | pin 6 |
| I2C Line Reader | I2C | SDA=1, SCL=2, addr=0x53 |
| LoRa | Serial1 | RX=18, TX=17, 9600 baud |

### ESP32-S3 - LoRa Address

| Device | Address |
|--------|---------|
| ESP32-S3 | 2 |
| STM32 | 1 |

---

## AI Virtual Sensor

| | |
|-|-|
| Model | CNN Autoencoder, int8, X-CUBE-AI 10.2.0 |
| Input | 512 IMU samples - ax, ay, az, mag_x - int8 time-major |
| Output | 512 reconstructed samples - int8 time-major |
| Flash | 59 KB |
| RAM | 26 KB |
| MACC | 2.8M |
| Inference | 7 ms on Cortex-M7 at 480 MHz |

During normal flight the model receives every IMU sample and keeps its history buffer full. No inference runs - it stays on standby. When the IMU fails, the last 512 samples feed the model. Every output sample passes physics constraint validation before entering the sensor queue.

Hardware log:

```
[AI-FALLBACK] Sensor recovered (normal=2801, fallback=395)
[AI-FALLBACK] Sensor recovered (normal=2901, fallback=395)
[AI-FALLBACK] Sensor recovered (normal=3001, fallback=395)
```

---

## ROS2 Topics

| Topic | Type | Rate |
|-------|------|------|
| /drone/imu | sensor_msgs/Imu | 1 Hz |
| /drone/pose | geometry_msgs/PoseStamped | 1 Hz |
| /drone/health | std_msgs/String | 1 Hz |
| /drone/fault | std_msgs/String | on event |
| /drone/control | geometry_msgs/Twist | 1 Hz |

---

## System Architecture

```
ESP32-S3              STM32H755                    PC
--------              ---------                    --
Radar      LoRa       IMU, MAG, BARO, GPS          micro_ros_agent
Ultrasonic <------>   AI Virtual Sensor   <------>  /drone/imu
Obstacle   915MHz     EKF / ESKF                    /drone/pose
Line       P2P        LQR / MPC / SMC               /drone/health
Clip                  micro-ROS (USART6/FTDI)       /drone/fault
                                                    /drone/control

ESP32 -> STM32 : sensor data via LoRa (US:XX.X, S:XXXXX, H:X)
STM32 -> ESP32 : CONFIRM (7 bytes)
STM32 -> PC    : /drone/* topics via micro-ROS
```

---

## Build from Source

**Requirements** - Linux, 10GB free disk space

Download full environment - Zephyr SDK + west + venv + micro-ROS (~7GB):  
https://drive.google.com/file/d/1nJowpYsaRMkd-KCEaE6X97rAPc1WYu8L/view?usp=sharing

```bash
cd ~
tar xzf zephyr_full_setup.tar.gz
```

Verify before building:
```bash
ls -d ~/zephyrproject ~/zephyr_project ~/zephyr_microros_venv ~/.local/zephyr-sdk-0.17.0
```

Build:
```bash
source ~/zephyr_microros_venv/bin/activate
source ~/zephyrproject/zephyr/zephyr-env.sh
cd ~/zephyr_project
west build -b nucleo_h755zi_q/stm32h755xx/m7
```

Full clean build (use when builds act weird):
```bash
rm -rf build
west build -b nucleo_h755zi_q/stm32h755xx/m7
```

Flash:
```bash
west flash
```

Serial monitor:
```bash
picocom -b 115200 /dev/ttyACM1 --logfile ~/run.log
```

> **Note:** `/dev/ttyACM1` is not fixed. The number (ACM0, ACM1, ACM2...) depends
> on the order USB devices are connected. Run `ls /dev/ttyACM*` to see all
> connected ACM devices. The ST-LINK VCP is whichever port appears when you
> plug in the NUCLEO board. If only the NUCLEO is connected, it is usually
> `/dev/ttyACM0`. If ESP32 is also connected, it may shift to `/dev/ttyACM1`.
> Always check with `ls /dev/ttyACM*` before connecting.

---

## Flash Pre-built Binary

```bash
git clone https://github.com/ephemeris-void/apsa.git
cd apsa
west flash --hex-file prebuilt/zephyr.hex
```

---

## micro-ROS Agent

Terminal 1:
```bash
distrobox enter ros2
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200 -v6
```

> **Note:** `/dev/ttyUSB0` is the FTDI adapter port. Run `ls /dev/ttyUSB*`
> to confirm the correct port if multiple USB-serial adapters are connected.

After agent starts, press reset (B2) on the board.

Terminal 2:
```bash
distrobox enter ros2
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 topic list
ros2 topic echo /drone/imu --once
ros2 topic echo /drone/health --once
```

---

## Ports

The port numbers below are typical but **not fixed** — they change depending
on the order devices are plugged in. Always run `ls /dev/ttyACM* /dev/ttyUSB*`
to confirm before connecting.

| Device | Typical port | How to identify |
|--------|-------------|-----------------|
| STM32 ST-LINK VCP (USART3, serial log) | /dev/ttyACM0 or ACM1 | Appears when NUCLEO is plugged in |
| FTDI adapter (USART6, micro-ROS) | /dev/ttyUSB0 | Appears when FTDI cable is plugged in |
| ESP32 USB | /dev/ttyACM0 or ACM1 | Appears when ESP32 is plugged in |

```bash
# Always check first:
ls /dev/ttyACM* /dev/ttyUSB*
```
