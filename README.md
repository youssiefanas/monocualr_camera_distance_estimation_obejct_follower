# Object Follower and Distance Estimation Project

Distance estimation and object following with a monocular webcam using transfer YOLOv8 custom model

## Description
This GitHub repository contains the code and documentation for the Object Follower and Distance Estimation project. The project focuses on implementing advanced computer vision techniques to detect objects, estimate their distances, and enable object following using a monocular webcam.

## Key Features
- Object detection and distance estimation using transfer learning with the YOLOv8 model in PyTorch
- Real-time distance estimation with 94% accuracy at 20 fps
- Integration with ROS (Robot Operating System) for seamless communication between the object detection system and an Arduino-based robot
- Implementation of an object follower algorithm using a PID controller

## Demo
#### Object follower using monocular camera  
https://youtu.be/gzI_PZzbCxw  
https://youtube.com/shorts/SKCsO3qaNMI?feature=share  

#### Distance estimation using monocular camera  
https://youtu.be/ECuG9YjshBA  
https://youtu.be/4-ry38Dj9bc  

## Project Overview

### 1. Collect and Label Data

1.1. Manually move the camera around, capturing images (using camera_capture.py script) of the objects to be followed at various distances and angles using OpenCV to create your dataset.

1.2. Label the images with the corresponding geometric shapes and colors using the annotation tool (labelImg).

### 2. Develop the Object Detection Model

2.1. Split the labeled images randomly into training, testing, and validation sets.

2.2. Create a folder structure for the dataset:
   ```
   ├── yolov8
     └── train
       └── images (folder including all training images)
       └── labels (folder including all training labels)
     └── test
       └── images (folder including all testing images)
       └── labels (folder including all testing labels)
     └── valid
       └── images (folder including all validation images)
       └── labels (folder including all validation labels)
   ```

2.3. Create a custom configuration file named "custom.yaml" in the current directory.

2.4. Create a deep learning model using a pre-trained YOLOv8 model and fine-tune it on your dataset.

2.5. Train the model on the training set and validate its performance on the validation set.

2.6. Save the trained model.

### 3. Implement Distance Estimation Algorithm

3.1. Write a Python script that captures images from the webcam and processes them using the trained object detection model.

3.2. Calculate the distance from the camera to each detected flag using the size of the bounding box around the object in the image and the actual size of the flag. Ensure the camera is calibrated for accurate distance measurements.

### 4. Implementation with ROS

4.1. Create a ROS package that contains the Python script and the trained model.

4.2. The ROS package publishes the name, position, and distance of the detected object as ROS messages.

4.3. Use the rosserial node to communicate with the Arduino.

4.4. Implement an object follower algorithm to track the detected objects in real-time.

4.5. Control the robot to move to the flag with a specified distance using a PID controller.

## Hardware Used

- Arduino Uno
- L298N Driver
- 2 DC Motors
- Battery Holder
- Logitech Monocular Webcam

## Programs and Platforms Used

- labelImg
- Arduino IDE
-  ROS Noetic

## Dependencies

Python:
```
ros
cv2
cv_bridge
ultralytics
```

Arduino:
```
ros.h
```

## Installation

1. Install the ROSserial package in the Arduino libraries folder:
   ```
   sudo apt-get install ros-noetic-rosserial-arduino
   sudo apt-get install ros-noetic-rosserial
   ```

   Run the ROS core:
   ```
   roscore
   ```

   In the Arduino libraries folder, run the ROS node:
   ```
   rosrun rosserial arduino make libraries.py .
   ```

2. Clone the repository:
   ```
   git clone git@github.com:youssiefanas/monocualr_camera_distance_estimation_obejct_follower.git
   ```

   Copy the "monocular_camera_distance_estimation" folder to your ROS workspace (e.g., catkin_ws).

   Change the path of the model in the "distance_detection/scripts/detecto.py" file:
   ```python
   model = YOLO("/catkin_ws/src/distance_detection/best(1).pt")
   ```
   path to "/best(1).pt"
   

   Then navigate to your workspace directory and run `catkin_make` in your terminal.

   Upload the Arduino sketch (included in this repository) to the board.

## Usage

1. Start the ROS core:
   ```
   roscore
   ```

2. Run the ROSserial Arduino node:
   ```
   rosrun rosserial_arduino serial_node.py /dev/ttyACM0
   ```

3. Run the object follower script:
   ```
   rosrun distance_detection detecto.py
   ```

4. Run the USB camera node:
   ```
   rosrun usb_cam usb_cam_node _pixel_format:=yuyv
   ```
