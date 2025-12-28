cmake_minimum_required(VERSION 3.10)
project(GNC_Simulator VERSION 1.0.0 LANGUAGES CXX)

# ============================================================================
# Build Configuration
# ============================================================================

# C++17 standard (required for std::filesystem)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")

# ============================================================================
# Dependencies
# ============================================================================

# Find Eigen3 (linear algebra - used minimally here, but good for future extensions)
find_package(Eigen3 3.3 QUIET)
if(EIGEN3_FOUND)
    message(STATUS "Found Eigen3: ${EIGEN3_VERSION}")
else()
    message(STATUS "Eigen3 not found (optional for this project)")
endif()

# Find SDL2 (visualization)
find_package(SDL2 QUIET)
if(NOT SDL2_FOUND)
    # Fallback: Try pkg-config
    find_package(PkgConfig)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(SDL2 QUIET sdl2)
    endif()
endif()

if(SDL2_FOUND)
    message(STATUS "Found SDL2")
    if(SDL2_INCLUDE_DIRS)
        message(STATUS "  Include dirs: ${SDL2_INCLUDE_DIRS}")
    endif()
    if(SDL2_LIBRARIES)
        message(STATUS "  Libraries: ${SDL2_LIBRARIES}")
    endif()
else()
    message(WARNING "SDL2 not found. Install with: sudo apt-get install libsdl2-dev")
    message(WARNING "Building without visualization support.")
endif()

# Find spdlog (structured logging)
find_package(spdlog QUIET)
if(NOT spdlog_FOUND)
    # Try pkg-config fallback
    find_package(PkgConfig)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(SPDLOG QUIET spdlog)
    endif()
endif()

if(spdlog_FOUND OR SPDLOG_FOUND)
    message(STATUS "Found spdlog")
else()
    message(WARNING "spdlog not found. Install with: sudo apt-get install libspdlog-dev")
endif()

# Find Google Test (unit testing)
find_package(GTest QUIET)
if(GTest_FOUND)
    message(STATUS "Found Google Test - unit tests will be built")
    enable_testing()
else()
    message(STATUS "Google Test not found - skipping unit tests")
    message(STATUS "Install with: sudo apt-get install libgtest-dev")
endif()

# ============================================================================
# Include Directories
# ============================================================================

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(SDL2_INCLUDE_DIRS)
    include_directories(${SDL2_INCLUDE_DIRS})
endif()

if(SPDLOG_INCLUDE_DIRS)
    include_directories(${SPDLOG_INCLUDE_DIRS})
endif()

# ============================================================================
# Core Library
# ============================================================================

# gnc_core: Reusable library with all simulation components
add_library(gnc_core STATIC
    src/dynamics/bicycle_model.cpp
    src/control/pid_controller.cpp
    src/planning/trajectory.cpp
    src/visualization/visualizer.cpp
    src/logger.cpp
)

# Link dependencies to library
target_include_directories(gnc_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

# Link libraries
set(GNC_CORE_LIBS "")

if(EIGEN3_FOUND)
    target_link_libraries(gnc_core PUBLIC Eigen3::Eigen)
    list(APPEND GNC_CORE_LIBS Eigen3::Eigen)
endif()

if(SDL2_FOUND)
    if(TARGET SDL2::SDL2)
        target_link_libraries(gnc_core PRIVATE SDL2::SDL2)
    elseif(SDL2_LIBRARIES)
        target_link_libraries(gnc_core PRIVATE ${SDL2_LIBRARIES})
    endif()
endif()

if(spdlog_FOUND)
    target_link_libraries(gnc_core PUBLIC spdlog::spdlog)
    list(APPEND GNC_CORE_LIBS spdlog::spdlog)
elseif(SPDLOG_FOUND)
    target_link_libraries(gnc_core PUBLIC ${SPDLOG_LIBRARIES})
endif()

# ============================================================================
# Main Executable
# ============================================================================

add_executable(gnc_sim src/main.cpp)
target_link_libraries(gnc_sim PRIVATE gnc_core)

message(STATUS "Main executable: gnc_sim")

# ============================================================================
# Unit Tests
# ============================================================================

if(GTest_FOUND)
    # Test for bicycle model
    add_executable(test_bicycle_model tests/test_bicycle_model.cpp)
    target_link_libraries(test_bicycle_model PRIVATE
        gnc_core
        GTest::GTest
        GTest::Main
    )
    add_test(NAME BicycleModelTest COMMAND test_bicycle_model)

    # Test for PID controller
    add_executable(test_pid_controller tests/test_pid_controller.cpp)
    target_link_libraries(test_pid_controller PRIVATE
        gnc_core
        GTest::GTest
        GTest::Main
    )
    add_test(NAME PIDControllerTest COMMAND test_pid_controller)

    message(STATUS "Unit tests configured:")
    message(STATUS "  - test_bicycle_model")
    message(STATUS "  - test_pid_controller")
    message(STATUS "Run with: ctest --output-on-failure")
endif()

# ============================================================================
# Example Programs
# ============================================================================

add_executable(track_trajectory examples/track_trajectory.cpp)
target_link_libraries(track_trajectory PRIVATE gnc_core)

message(STATUS "Example programs:")
message(STATUS "  - track_trajectory")

# ============================================================================
# Build Summary
# ============================================================================

message(STATUS "")
message(STATUS "============================================")
message(STATUS "GNC Simulator Configuration Summary")
message(STATUS "============================================")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "")
message(STATUS "Dependencies:")
message(STATUS "  Eigen3:  ${EIGEN3_FOUND}")
message(STATUS "  SDL2:    ${SDL2_FOUND}")
message(STATUS "  spdlog:  ${spdlog_FOUND}")
message(STATUS "  GTest:   ${GTest_FOUND}")
message(STATUS "")
message(STATUS "Build targets:")
message(STATUS "  - gnc_core (library)")
message(STATUS "  - gnc_sim (main executable)")
message(STATUS "  - track_trajectory (example)")
if(GTest_FOUND)
message(STATUS "  - test_bicycle_model (unit test)")
message(STATUS "  - test_pid_controller (unit test)")
endif()
message(STATUS "============================================")
message(STATUS "")
