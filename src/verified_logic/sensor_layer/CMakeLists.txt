# ============================================================
# SENSOR ABSTRACTION LAYER — CMakeLists
# ============================================================
# Board: NUCLEO-H755ZI-Q
# Target: workspace_main_CM7
# ============================================================

# ---------- Core Sources ----------
set(SAL_CORE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/sal_core.c
)

# ---------- Sensor Modules (conditional) ----------
set(SAL_MODULE_SOURCES "")

if(SAL_ENABLE_IMU)
    list(APPEND SAL_MODULE_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/mod_imu.c
    )
endif()

if(SAL_ENABLE_GPS)
    list(APPEND SAL_MODULE_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/mod_gps.c
    )
endif()

if(SAL_ENABLE_MAG)
    list(APPEND SAL_MODULE_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/mod_mag.c
    )
endif()

if(SAL_ENABLE_BARO)
    list(APPEND SAL_MODULE_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/mod_baro.c
    )
endif()

if(SAL_ENABLE_LORA)
    list(APPEND SAL_MODULE_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/mod_lora.c
    )
endif()

# ---------- Aggregate ----------
set(SAL_ALL_SOURCES
    ${SAL_CORE_SOURCES}
    ${SAL_MODULE_SOURCES}
)

set(SAL_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/modules
)

# ---------- Export ----------
set(SAL_ALL_SOURCES ${SAL_ALL_SOURCES} PARENT_SCOPE)
set(SAL_INCLUDE_DIRS ${SAL_INCLUDE_DIRS} PARENT_SCOPE)