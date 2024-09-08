# ROS 2 Tasks Documentation

## Task 1

1. **Copy the `rse_m3` Folder:**
   - Copy the `rse_m3` folder into the `ros2_ws/src` directory.

2. **Install the `urdf_tutorial` Package:**
   - You need to install the `urdf_tutorial` package.

3. **Build the Package:**
   - Before running Task 1, you need to build the package using the following command:
     ```bash
     colcon build
     ```

4. **Run Task 1:**
   - Open a new terminal and run the following command (make sure to update the file path according to your system):
     ```bash
     ros2 launch urdf_tutorial display.launch.py model:=/home/ros/ros2_ws/src/rse_m3/urdf/task1.urdf
     ```
   - In another terminal, run the following command:
     ```bash
     ros2 launch rse_m3 task1.launch.py
     ```

## Task 2

1. **Run Task 2:**
   - Open a terminal and run the following command:
     ```bash
     ros2 launch urdf_tutorial display.launch.py model:=/home/ros/ros2_ws/src/rse_m3/urdf/task2.urdf
     ```

2. **Generate a Frame Tree:**
   - While the previous launch file is running, open a new terminal and run the following command to generate a frame tree:
     ```bash
     ros2 run tf2_tools view_frames
     ```
   - You may need to install the `tf2_tools` package if it is not already installed.

## Task 3-a

1. **Run Task 3-a:**
   - Open a new terminal and run the following command:
     ```bash
     ros2 launch urdf_tutorial display.launch.py model:=/home/ros/ros2_ws/src/rse_m3/urdf/task3.urdf
     ```

## Task 3-b

1. **Run Task 3-b:**
   - Open a new terminal and run the following command:
     ```bash
     ros2 launch urdf_tutorial display.launch.py model:=/home/ros/ros2_ws/src/rse_m3/urdf/task3b.urdf
     ```
