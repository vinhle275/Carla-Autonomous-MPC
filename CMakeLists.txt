cmake_minimum_required(VERSION 3.20)
project(autonomous_vehicle_mpc C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/libs
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/osqp/inc/public
)

# Thư viện hệ thống socket và IPC trên Windows / Linux / Raspberry Pi 5
if(WIN32)
    set(SYSTEM_SOCKET_LIBS ws2_32)
else()
    set(SYSTEM_SOCKET_LIBS pthread rt)
endif()

# ==============================================================================
# 0. OFFICIAL OSQP v1.0.0 C LIBRARY (STATIC - ZERO EXTERNAL RUNTIME DLL/SO)
# ==============================================================================
add_library(av_osqp_static STATIC
    third_party/osqp/src/algebra_libs.c
    third_party/osqp/src/auxil.c
    third_party/osqp/src/csc_math.c
    third_party/osqp/src/csc_utils.c
    third_party/osqp/src/error.c
    third_party/osqp/src/kkt.c
    third_party/osqp/src/matrix.c
    third_party/osqp/src/osqp_api.c
    third_party/osqp/src/qdldl.c
    third_party/osqp/src/qdldl_interface.c
    third_party/osqp/src/scaling.c
    third_party/osqp/src/util.c
    third_party/osqp/src/vector.c
)
target_include_directories(av_osqp_static
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/osqp/inc/public
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/osqp/inc/private
)

# ==============================================================================
# LIBS: TẦNG THƯ VIỆN DÙNG CHUNG
# ==============================================================================

# 1. common (Header-only)
add_library(av_common INTERFACE)
target_include_directories(av_common INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/libs)

# 2. safety_guard
add_library(av_safety_guard STATIC
    libs/safety_guard/collision_danger_zone.cpp
    libs/safety_guard/emergency_safety_manager.cpp
)
target_link_libraries(av_safety_guard PUBLIC av_common)

# 3. mpc_core
add_library(av_mpc_core STATIC
    libs/mpc_core/linearized_mpc_controller.cpp
)
target_link_libraries(av_mpc_core PUBLIC av_common av_osqp_static)

# 4. perception_core
add_library(av_perception_core STATIC
    libs/perception_core/road_corridor.cpp
    libs/perception_core/obstacle_detector.cpp
    libs/perception_core/yellow_line_detector.cpp
)
target_link_libraries(av_perception_core PUBLIC av_common)

# 5. hal
add_library(av_hal STATIC
    libs/hal/carla_bridge_client.cpp
)
target_link_libraries(av_hal PUBLIC av_common ${SYSTEM_SOCKET_LIBS})

# ==============================================================================
# APPS: CÁC TIẾN TRÌNH AUTOSAR ADAPTIVE RUNTIME
# ==============================================================================

# Process 1: PAIO Vehicle Interface (100Hz)
add_executable(paio_vehicle_interface
    apps/paio_vehicle_interface/main.cpp
)
target_link_libraries(paio_vehicle_interface PRIVATE
    av_hal
    av_common
    ${SYSTEM_SOCKET_LIBS}
)

# Process 2: Autonomous Core (50Hz)
add_executable(autonomous_core
    apps/autonomous_core/main.cpp
)
target_link_libraries(autonomous_core PRIVATE
    av_mpc_core
    av_safety_guard
    av_perception_core
    av_common
    ${SYSTEM_SOCKET_LIBS}
)

# Process 3: HMI Diagnostics (20Hz)
add_executable(hmi_diagnostics
    apps/hmi_diagnostics/main.cpp
)
target_link_libraries(hmi_diagnostics PRIVATE
    av_common
)

# Unified App: Chạy cả 3 tiến trình trong 1 binary (tiện lợi, một lệnh khởi động)
add_executable(autonomous_mpc_unified
    apps/unified_app/main.cpp
)
target_link_libraries(autonomous_mpc_unified PRIVATE
    av_hal
    av_mpc_core
    av_safety_guard
    av_perception_core
    av_common
    ${SYSTEM_SOCKET_LIBS}
)

# Edge Case & Regression Test Suite (local only when tests/ exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_edge_cases.cpp")
    add_executable(test_edge_cases
        tests/test_edge_cases.cpp
    )
    target_link_libraries(test_edge_cases PRIVATE
        av_mpc_core
        av_safety_guard
        av_perception_core
        av_common
        ${SYSTEM_SOCKET_LIBS}
    )
endif()
