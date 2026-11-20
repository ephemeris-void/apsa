# ============================================================
# ZEPHYR PROJECT — CMakeLists.txt (FIXED — ALL 5 SENSORS)
# ============================================================
# Board: NUCLEO-H755ZI-Q
# Project: AI Virtual Sensor (Zephyr + micro-ROS)
# ============================================================

cmake_minimum_required(VERSION 3.20.0)

# ---------- micro-ROS external module ----------
list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/micro_ros_zephyr_module/modules/libmicroros
)

# ---------- Find Zephyr ----------
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# ---------- Project ----------
project(stm32_ai_virtual_sensor
    VERSION 2.0.0
    LANGUAGES C CXX ASM
)

# ============================================================
# C / C++ STANDARDS
# ============================================================
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================
# INCLUDE DIRECTORIES
# ============================================================
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/config
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ros
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/ai
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/agent
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/agent/core
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/agent/states
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/agent/decisions
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/agent/actions
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/agent/safety
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/agent/health
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/agent/fdir
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/agent/fallback
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/agent/recovery
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/agent/confidence
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/agent/voting
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/agent/crosscheck
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/agent/escalation
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/agent/wcet
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/agent/trace
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/agent/interfaces
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/apsa_modules
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/apsa_bridge
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/imu
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/cm4_ipc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/cm4_ipc/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/X-CUBE-AI/App
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/X-CUBE-AI
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/X-CUBE-AI/Middlewares/ST/AI/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/apsa_modules
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/apsa_modules/core

    # ✅ ADDED: Sensor Layer
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/sensor_layer
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/sensor_layer/modules
)

# ============================================================
# APPLICATION SOURCES
# ============================================================
target_sources(app PRIVATE
    src/main.c
    src/config/error_handler.c
    src/hal/hal_time.c
    src/hal/hal_gpio.c
    src/hal/hal_spi.c
    src/hal/hal_uart.c
    src/hal/hal_cache.c
    src/hal/hal_ipc.c
    src/hal/hal_flash.c
    src/hal/hal_reset.c
    src/hal/hal_watchdog.c
    src/hal/hal_dma.c
    src/hal/hal_power.c
    src/hal/hal_clock.c
    src/hal/hal_cpu.c
#    src/common/ring_buffer.c
    src/common/metrics.c
#    src/common/state_machine.c
#    src/common/pipeline.c
    src/common/spsc_queue.c
    src/common/logger.c
    src/common/config_loader.c
    src/ros/micro_ros_app.c
)

# ---------- AI ----------
target_sources(app PRIVATE
    src/verified_logic/ai/ai_icm_virtual.c
    src/verified_logic/X-CUBE-AI/App/ai_wrapper.c
    src/verified_logic/X-CUBE-AI/App/app_x-cube-ai.c
    src/verified_logic/X-CUBE-AI/App/network.c
    src/verified_logic/X-CUBE-AI/App/network_data.c
    src/verified_logic/X-CUBE-AI/App/network_data_params.c
)

# ---------- IMU ----------
target_sources(app PRIVATE
    src/verified_logic/imu/imu_icm42688.c
)

# ---------- IPC ----------
target_sources(app PRIVATE
    src/verified_logic/cm4_ipc/Src/cm4_ipc.c
)

# ---------- Agent Core ----------
target_sources(app PRIVATE
    src/verified_logic/agent/core/agent.c
    src/verified_logic/agent/core/agent_context.c
    src/verified_logic/agent/core/agent_init.c
    src/verified_logic/agent/core/agent_main_hook.c
)

# ---------- Agent States ----------
target_sources(app PRIVATE
    src/verified_logic/agent/states/agent_states.c
    src/verified_logic/agent/states/agent_transitions.c
)

# ---------- Agent Decisions ----------
target_sources(app PRIVATE
    src/verified_logic/agent/decisions/agent_decisions.c
    src/verified_logic/agent/decisions/agent_priority.c
    src/verified_logic/agent/ipc/agent_ipc.c
)

# ---------- Agent Actions ----------
target_sources(app PRIVATE
    src/verified_logic/agent/actions/agent_actions.c
    src/verified_logic/agent/actions/agent_actions_ai.c
)

# ---------- Agent Safety ----------
target_sources(app PRIVATE
    src/verified_logic/agent/safety/agent_safety.c
    src/verified_logic/agent/safety/agent_watchdog.c
)

# ---------- Agent Health ----------
target_sources(app PRIVATE
    src/verified_logic/agent/health/agent_health.c
    src/verified_logic/agent/health/agent_health_per_sensor.c
)

# ---------- Agent FDIR ----------
target_sources(app PRIVATE
#    src/verified_logic/agent/fdir/agent_fdir.c
#    src/verified_logic/agent/fdir/agent_fdir_detector.c
#    src/verified_logic/agent/fdir/agent_fdir_isolator.c
)

# ---------- Agent Fallback ----------
target_sources(app PRIVATE
#    src/verified_logic/agent/fallback/agent_fallback.c
)

# ---------- Agent Recovery ----------
target_sources(app PRIVATE
#    src/verified_logic/agent/recovery/agent_recovery.c
#    src/verified_logic/agent/recovery/agent_recovery_strategy.c
)

# ---------- Agent Confidence ----------
target_sources(app PRIVATE
#    src/verified_logic/agent/confidence/agent_confidence.c
)

# ---------- Agent Voting ----------
target_sources(app PRIVATE
#    src/verified_logic/agent/voting/agent_voting.c
)

# ---------- Agent Crosscheck ----------
target_sources(app PRIVATE
#    src/verified_logic/agent/crosscheck/agent_crosscheck.c
)

# ---------- Agent Escalation ----------
target_sources(app PRIVATE
#    src/verified_logic/agent/escalation/agent_escalation.c
)

# ---------- Agent WCET ----------
target_sources(app PRIVATE
#    src/verified_logic/agent/wcet/agent_wcet.c
)

# ---------- Agent Trace ----------
target_sources(app PRIVATE
    src/verified_logic/agent/trace/agent_trace.c
    src/verified_logic/agent/trace/agent_log.c
    src/verified_logic/agent/trace/agent_stats.c
)

# ---------- APSΛ Bridge ----------
target_sources(app PRIVATE
    src/verified_logic/apsa_bridge/apsa_config.c
    src/verified_logic/apsa_bridge/apsa_ekf_adapter.cpp
    src/verified_logic/apsa_bridge/apsa_ctrl_adapter.cpp
)

# ---------- APSΛ Compile Unit ----------
target_sources(app PRIVATE
    src/verified_logic/apsa_modules/apsa_compile.cpp
)

# ============================================================
# ✅ FIXED: Sensor Layer (SAL — all 5 modules)
# ============================================================
target_sources(app PRIVATE
    src/verified_logic/sensor_layer/sal_core.c
    src/verified_logic/sensor_layer/sal_diag.c
    src/verified_logic/sensor_layer/modules/mod_imu.c
    src/verified_logic/sensor_layer/modules/mod_gps.c
    src/verified_logic/sensor_layer/modules/mod_mag.c
    src/verified_logic/sensor_layer/modules/mod_baro.c
    src/verified_logic/sensor_layer/modules/mod_lora.c
)

# ============================================================
# LINK LIBRARIES
# ============================================================
target_link_libraries(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/verified_logic/X-CUBE-AI/Middlewares/ST/AI/Lib/NetworkRuntime1020_CM7_GCC.a
)

target_link_libraries(app PRIVATE m)

# ============================================================
# COMPILE DEFINITIONS
# ============================================================
target_compile_definitions(app PRIVATE
    CORE_CM7
    USE_HAL_DRIVER
    STM32H755xx
    USE_PWR_DIRECT_SMPS_SUPPLY
    PROJECT_NAME="stm32_ai_virtual_sensor"
    PROJECT_VERSION_MAJOR=2
    PROJECT_VERSION_MINOR=0
    PROJECT_VERSION_PATCH=0
    __STDC_WANT_LIB_EXT1__=0
)

# ============================================================
# COMPILE OPTIONS
# ============================================================
target_compile_options(app PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-missing-field-initializers
    -fno-common
)

# C++ specific
target_compile_options(app PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:
        -fno-exceptions
        -fno-rtti
        -fno-use-cxa-atexit
        -fno-threadsafe-statics
    >
)

# ============================================================
# LINKER OPTIONS
# ============================================================
target_link_options(app PRIVATE
    -Wl,--allow-multiple-definition
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/stm32_ai_virtual_sensor.map
)

# ============================================================
# Additional STM32 HAL sources
# ============================================================
target_sources(app PRIVATE
    ${ZEPHYR_BASE}/../modules/hal/stm32/stm32cube/stm32h7xx/drivers/src/stm32h7xx_hal_pwr_ex.c
)