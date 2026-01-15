# Requires CMake version 3.16 or higher for Qt 6
cmake_minimum_required(VERSION 3.16)

# Define the project name
project(qt LANGUAGES CXX)

# Find the Qt 6 package and required modules
find_package(Qt6 REQUIRED COMPONENTS Widgets)

# Set the C++ standard to 17
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tell CMake to use Qt's compilation features
qt_standard_project_setup()

# Add the executable
add_executable(${PROJECT_NAME}
    main.cpp
    LidarSimulator.cpp
    LidarVisualizer.cpp
    LiveErrorPlot.cpp
    Config.h
    localizor2.cpp
    LidarSimulator.h
    LidarVisualizer.h
    LiveErrorPlot.h
    localizor.h
)

# Link the executable to the Qt modules
target_link_libraries(${PROJECT_NAME} PRIVATE Qt6::Widgets)

# Set up the source files to be processed by Qt's meta-object compiler (MOC)
# NOTE: The Qt 6 CMake integration handles MOC processing automatically for
# files listed above, making explicit calls like qt_add_executable_headers
# and qt_add_executable_sources unnecessary and often a source of errors.