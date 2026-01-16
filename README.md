# LiDAR Simulator & Localizer

A C++ and Qt-based simulator designed for testing LiDAR line-segmentation and robot localization algorithms in a virtual 2D environment.

## 🚀 Features
* **LiDAR Simulation:** Generates synthetic point clouds within a 144x144 inch room boundary.
* **Line Segmentation:** Implements Split-and-Merge logic to convert raw point data into clean geometric line segments.
* **Rotation Alignment:** Automatically calculates the rotation offset between the robot's IMU and the room's global axes.
* **Global Localization:** * Anchors robot position based on detected wall distances.
    * Handles edge cases: single wall visibility, parallel wall constraints, and collinear segments.
* **Live Visualization:** Real-time rendering of the robot pose, point cloud, and extracted wall lines using a custom Qt Widget.

## 🛠 Project Structure
* `main.cpp` - Initializes the `QApplication` and sets up the UI layout.
* `LidarVisualizer.cpp` - Custom OpenGL/Painter widget that renders the robot and LiDAR data.
* `LidarProcessor.cpp` - Contains the segmentation algorithms and the `localizeFromLines` solver.
* `Point2d.h` - Utility struct for handling Cartesian coordinates.

## 📐 How it Works
1. **Sensing:** The simulator generates points relative to the robot.
2. **Segmenting:** The segmentor converts points into line segments
3. **Alignment:** The localizer calculates the `measuredRoomAngle` to determine how much the robot is tilted relative to the wall line segments.
4. **Transformation:** The system rotates the `finalLines` by the calculated offset to align them with the global grid.
5. **Positioning:** Lines are bucketed into `Top`, `Bottom`, `Left`, and `Right` categories. The robot's $(x, y)$ coordinate is solved by comparing these relative distances to the known 144-inch room boundaries.

## 💻 Requirements
* **Qt 6.x** (Widgets & Core)
* **C++17** or higher
* **CMake** 3.16+

## 🔨 Building
1. Configure the project for your specific kit.
2. Build and Run.
