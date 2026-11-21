# STM32CubeIDE Implementation

This is a preliminary implementation of the APSA project, built before the Zephyr version.
The Zephyr RTOS version (root) was built later for 1ms timing requirements and
safety-critical system design. Both run on the same NUCLEO-H755ZI-Q hardware.

---

## Board

NUCLEO-H755ZI-Q — STM32H755ZIT6, LQFP144
- CM7: 480 MHz — IMU, EKF, control, virtual sensor logic
- CM4: 240 MHz — AI inference only

---

## What runs where

CM7 (workspace_main):
- IMU driver: ICM-42688-P over SPI1, polled every 1ms
- ESKF: attitude and position estimation, initialized from averaged IMU samples
- Control: MPC output — thrust, roll, pitch, yaw
- AI trigger: sends 512 samples to CM4 every 20ms, receives reconstructed output
- Pipeline output: EKF + control data framed at 50Hz over USART3
- IPC: polls CM4 heartbeat every 500ms

CM4 (CM4_AI_main):
- Waits for input from CM7 over IPC (SRAM4 shared memory at 0x38000000)
- Runs X-CUBE-AI inference: CNN autoencoder, int8, 2.28M MACC
- Sends result back to CM7 with sequence ID and timestamp
- Heartbeat every 100ms

IPC transport: SRAM4 (0x38000000, 64KB), MPU-configured as shareable on both cores.
No RTOS. No interrupts for IPC. Polling only.

---

## Peripherals

| Peripheral | Config |
|------------|--------|
| IMU ICM-42688-P | SPI1, CS=PD14, 8-bit, CPOL=0 CPHA=0, prescaler 64 |
| Serial log | USART3, TX=PD8, RX=PD9, 921600 baud, 8N1 |

Clock: HSI, PLL — CM7 at 480MHz, CM4 at 240MHz, AHB at 240MHz

---

## AI Model

| | |
|-|-|
| Model | CNN autoencoder, Keras, int8 quantized |
| Tool | X-CUBE-AI 10.2.1 on CM4 |
| Input | 512 IMU samples, int8 |
| Output | 512 reconstructed samples, int8 |
| Flash | 107 KB |
| RAM | 32 KB (activation buffer) |
| MACC | 2.28M |

---

## Folder Structure

```text
stm-cube/
├── CM4_AI_main/                  # CM4 project — AI inference
│   ├── CM4/                      # CM4 source, HAL, startup
│   ├── CM7/                      # CM7 stub
│   ├── Common/                   # IPC code shared between cores
│   ├── Drivers/                  # STM32H7 HAL
│   ├── Middlewares/              # X-CUBE-AI 10.2.1
│   └── CM4_AI_main.ioc           # STM32CubeMX configuration
│
├── workspace_main/               # CM7 project — main application
│   ├── CM7/                      # CM7 source, EKF, IMU, control, modules
│   ├── CM4/                      # CM4 stub
│   ├── Common/                   # IPC code shared between cores
│   ├── Drivers/                  # STM32H7 HAL
│   └── Middlewares/              # X-CUBE-AI
│
└── ELF_BINARIES/                 # Pre-built binaries, ready to flash
    ├── CM4_AI_main_CM4.elf
    └── workspace_main_CM7.elf
```
---

## Flash Pre-built Binaries

Flash CM4 first, then CM7:
STM32_Programmer_CLI -c port=SWD -d ELF_BINARIES/CM4_AI_main_CM4.elf -rst
STM32_Programmer_CLI -c port=SWD -d ELF_BINARIES/workspace_main_CM7.elf -rst


Serial monitor:  picocom -b 921600 /dev/ttyACM0 -- *ACM* Is depend on your port

output:H:IMU=1 AI=1 EKF=1 CTRL=1 SYS=1 OK=...


---

## Build from Source

Requirements: STM32CubeIDE 1.13 or later

1. Open STM32CubeIDE
2. File → Import → Existing Projects into Workspace
3. Import both:
   - stm-cube/CM4_AI_main
   - stm-cube/workspace_main
4. Ctrl+B
5. Flash CM4 first, then CM7
6. Serial monitor at 921600 baud on /dev/ttyACM0

---

## Verification

3 minute continuous run on 2026-10-07.

| Check       | Result      |
|-------------|-------------|
| EKF running | 6232 / 6232 |
| System OK   | 6232 / 6232 |
| AI ready    | 178 / 178   |
| Errors      | 0           |

