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

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "src/*.cpp")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "include/*.hpp")

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Link the executable to the Qt modules
target_link_libraries(${PROJECT_NAME} PRIVATE Qt6::Widgets)

# Set up the source files to be processed by Qt's meta-object compiler (MOC)
# NOTE: The Qt 6 CMake integration handles MOC processing automatically for
# files listed above, making explicit calls like qt_add_executable_headers
# and qt_add_executable_sources unnecessary and often a source of errors.