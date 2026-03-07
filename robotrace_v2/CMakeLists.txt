cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME robotrace_v2)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
	${CMAKE_SOURCE_DIR}/Core/Src/battery.c
	${CMAKE_SOURCE_DIR}/Core/Src/BMI088.c
	${CMAKE_SOURCE_DIR}/Core/Src/IMU.c
	${CMAKE_SOURCE_DIR}/Core/Src/control.c
	${CMAKE_SOURCE_DIR}/Core/Src/courseAnalysis.c
	${CMAKE_SOURCE_DIR}/Core/Src/DWT.c
	${CMAKE_SOURCE_DIR}/Core/Src/emergencyStop.c
	${CMAKE_SOURCE_DIR}/Core/Src/encoder.c
	${CMAKE_SOURCE_DIR}/Core/Src/sd_benchmark.c
	${CMAKE_SOURCE_DIR}/Core/Src/sd_diskio_spi.c
	${CMAKE_SOURCE_DIR}/Core/Src/sd_functions.c
	${CMAKE_SOURCE_DIR}/Core/Src/sd_spi.c
	${CMAKE_SOURCE_DIR}/Core/Src/lineSensor.c
	${CMAKE_SOURCE_DIR}/Core/Src/main.c
	${CMAKE_SOURCE_DIR}/Core/Src/markerSensor.c
	${CMAKE_SOURCE_DIR}/Core/Src/motor.c
	${CMAKE_SOURCE_DIR}/Core/Src/PIDcontrol.c
	${CMAKE_SOURCE_DIR}/Core/Src/SDcard.c
	${CMAKE_SOURCE_DIR}/Core/Src/setup.c
	${CMAKE_SOURCE_DIR}/Core/Src/ssd1306.c
	${CMAKE_SOURCE_DIR}/Core/Src/ssd1306_fonts.c
	${CMAKE_SOURCE_DIR}/Core/Src/switch.c
	${CMAKE_SOURCE_DIR}/Core/Src/timer.c
	${CMAKE_SOURCE_DIR}/Core/Src/WS2812C.c

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
# Set the linker flags for printf/scanf float support
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -u _printf_float -u _scanf_float")
