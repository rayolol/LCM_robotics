cmake_minimum_required(VERSION 3.10)

project(LCM_robotics)

set(CMAKE_CXX_STANDARD 14)

# Find and include LCM
find_package(lcm REQUIRED)
include(${LCM_USE_FILE}) # Include LCM's utility functions
include_directories(${LCM_INCLUDE_DIRS})
link_directories(${LCM_LIBRARY_DIRS})

# Include directories for custom types
include_directories(${CMAKE_SOURCE_DIR}/src/my_types)

# Add executables
add_executable(sender src/sender.cpp)
add_executable(bridge src/bridge.cpp)

# Link libraries
target_link_libraries(sender ${LCM_LIBRARIES} lcm) # Ensure LCM is linked properly
target_link_libraries(bridge ${LCM_LIBRARIES} lcm) # Ensure LCM is linked properly

