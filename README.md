# Pallet-Detection-Segmentation
A pallet detection and segmentation application in ROS2 for a manufacturing or warehousing environment

## Project Description

This project focuses on training a deep learning model for real-time detection and segmentation of pallets and the ground in an industrial setting. The process began by curating a custom dataset, which was manually annotated using CVAT and Roboflow platforms to ensure accurate labeling. A pretrained YOLOv11 model was then fine-tuned on this dataset to adapt it for the specific detection task.

Once the model was trained, it was integrated into a ROS2 node for seamless, real-time pallet detection and segmentation within the industrial environment. To simplify deployment and enhance scalability, the entire solution was containerized using Docker, allowing for efficient deployment in varied production environments.

## Key Steps:
- Dataset Creation: Manual annotation of images using CVAT and Roboflow.
- Model Training: Fine-tuning a pretrained YOLOv11 model on the custom dataset.
- Integration: Embedding the trained model into a ROS2 node for real-time processing.
- Containerization: Packaging the solution into a Docker container for easy deployment and scalability.

## Instructions

1. **Setup**
   - Please make sure docker is installed on the host system
   - Please ensure the host system has a cuda version compatible with cuda 12.6 (docker has cuda 12.6 installed)
2. **Pull the docker image**
  ```bash
  docker pull suhasnagaraj1999/umd:pallet_segmentation
  ```
3. **Create a docker container**
  ```bash
  docker run -it --gpus all --net=host -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix --name segmentation_container suhasnagaraj1999/umd:pallet_segmentation
  ```
4. **Open Container Bash Shells**
  ```bash
  docker exec -it --env="DISPLAY=$DISPLAY"  segmentation_container /bin/bash
  ```

- Run the following steps inside the container

5. **Source the ROS version**
  ```bash
  cd
  source /opt/ros/humble/setup.bash
  ```
6. **Navigate to ROS workspace, build and source it**
  ```bash
  cd ros2_ws
  colcon build
  source install/setup.bash
  ```
7. **Launch the main ROS2 Node**
- This node publishes the object detection and segmentation results to `/detected_objects` and `/segmented_feed` respectively
  ```bash
  ros2 run pallet_pkg pallet_seg
  ```
8. **OR Launch the alternative ROS2 node**
- This node directly displayes the object detection and segmentation results
  ```bash
  ros2 run pallet_pkg pallet_seg_alt
  ```
- Note: Please ensure the camera topics match before running the nodes

## Results
- After training the model for 100 epochs, the follwoing results were achieved:
![alt text](https://github.com/suhasnagaraj99/Pallet-Detection-Segmentation/blob/main/seg_val1.png?raw=true)
