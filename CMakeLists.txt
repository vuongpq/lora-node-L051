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
set(CMAKE_PROJECT_NAME IoT_Node)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Keep the combined LoRaWAN stack within the STM32L051 flash budget.
add_compile_options(-Os)

# LoRaMac port settings for the current STM32L051 + SX1276 board.
set(BOARD board CACHE STRING "LoRaMac board target" FORCE)
set(RADIO sx1276 CACHE STRING "LoRaMac radio driver" FORCE)
set(SECURE_ELEMENT SOFT_SE CACHE STRING "LoRaMac secure element" FORCE)

# Region configuration must match runtime region selection in main.c.
set(REGION_EU868 OFF CACHE BOOL "Region EU868" FORCE)
set(REGION_AS923 ON CACHE BOOL "Region AS923" FORCE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add LoRaMac stack modules
add_subdirectory(Core/LoRaMac/board)
add_subdirectory(Core/LoRaMac/system)
add_subdirectory(Core/LoRaMac/radio)
add_subdirectory(Core/LoRaMac/mac)
add_subdirectory(Core/LoRaMac/peripherals)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    $<TARGET_OBJECTS:board>
    $<TARGET_OBJECTS:system>
    $<TARGET_OBJECTS:radio>
    $<TARGET_OBJECTS:mac>
    $<TARGET_OBJECTS:peripherals>
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
    board
    system
    radio
    mac
    peripherals

    # Add user defined libraries
)

# Keep a root-level ELF copy for simpler programmer commands.
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_SOURCE_DIR}/${CMAKE_PROJECT_NAME}.elf
)
