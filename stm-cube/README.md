# STM32H755ZI AI Virtual Sensor (STM32CubeIDE)

**Board:** NUCLEO-H755ZI-Q (dual core: Cortex-M7 + Cortex-M4)
**Tool:** STM32CubeIDE

## What it does
- **CM7** (`workspace_main/CM7`): main application — IMU, EKF, control, agent, AI
- **CM4** (`CM4_AI_main/CM4`): AI inference (X-CUBE-AI runtime)

## Folder layout

    stm-cube/
    ├── CM4_AI_main/          CM4 project — AI inference
    │   ├── CM4/              CM4 source, HAL, startup
    │   ├── CM7/              CM7 stub
    │   ├── Common/           IPC code shared between cores
    │   ├── Drivers/          STM32H7 HAL
    │   ├── Middlewares/      X-CUBE-AI 10.2.1
    │   └── CM4_AI_main.ioc
    ├── workspace_main/       CM7 project — main application
    │   ├── CM7/              CM7 source, EKF, IMU, control, modules
    │   ├── CM4/              CM4 stub
    │   ├── Common/           IPC code shared between cores
    │   ├── Drivers/          STM32H7 HAL
    │   └── Middlewares/      X-CUBE-AI
    └── firmware/             Pre-built binaries, ready to flash
        ├── CM4_AI_main_CM4.elf
        └── workspace_main_CM7.elf

## Build
1. Open STM32CubeIDE
2. **File → Import → Existing Projects into Workspace**
3. Select `stm-cube/workspace_main` and `stm-cube/CM4_AI_main`
4. **Project → Build All**

## Flash without building
Use **STM32CubeProgrammer** — flash **CM4 first, then CM7**:

| Step | File |
|------|------|
| 1 | `firmware/CM4_AI_main_CM4.elf` |
| 2 | `firmware/workspace_main_CM7.elf` |

## Verify
    picocom -b 921600 /dev/ttyACM0

Expected output:

    H:IMU=1 AI=1 EKF=1 CTRL=1 SYS=1 OK=...
