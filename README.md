# Smart Traffic Management System

A dynamic traffic simulation system that optimizes signal timing based on real-time vehicle density detection using YOLOv8.

## Overview
This project simulates an intelligent traffic intersection where green signal timers are adjusted adaptively. It uses computer vision (YOLOv8) to detect vehicles from video feeds of traffic lanes and calculates the optimal green light duration for each lane, reducing congestion compared to fixed-timer systems.

## System Workflow

![Smart Traffic Management Process](docs/process_flow.png)


## Features
- **Real-time Vehicle Detection**: Utilizes YOLOv8n to identify cars, bikes, buses, trucks, and rickshaws.
- **Adaptive Signal Timing**: Calculates green light duration based on vehicle count and type (e.g., buses need more time than bikes).
- **Simulation**: Visualizes the traffic flow and signal changes using Pygame.
- **Lane Tracking**: Monitors 4 incoming lanes simultaneously.

## Tech Stack
- **Python**: Core logic.
- **Pygame**: Simulation visualization.
- **YOLOv8 (Ultralytics)**: Object detection.
- **OpenCV**: Video processing.
- **SORT**: Object tracking (Simple Online and Realtime Tracking).
    - *Reference: [SORT: A Simple, Online and Realtime Tracker](https://github.com/abewley/sort) Copyright (C) 2016-2020 Alex Bewley alex@bewley.ai*

## Installation

1.  Clone the repository:
    ```bash
    git clone https://github.com/Rex1704/SmartTrafficManagement.git
    cd SmartTrafficManagement
    ```

2.  Install dependencies:
    ```bash
    pip install -r requirements.txt
    ```

3.  Ensure you have the required data files:
    - `yolo-model/yolov8n.pt`
    - `yolo-model/coco.names`
    - Video input files in `videos/` (vid1.mp4, vid2.mp4, etc.)

## Usage

Run the main signalling controller:

```bash
python TrafficSignalling.py
```

This will:
1.  Process the input traffic videos.
2.  Detect vehicles and calculate density.
3.  Launch the Pygame simulation window to visualize the intersection control.

## Project Structure
- `TrafficSignalling.py`: Main entry point. Handles video processing, detection, and initializes the simulation.
- `Simulation.py`: Handles the Pygame visualization, traffic light logic, and vehicle movement.
- `sort.py`: Implementation of the SORT tracking algorithm.
- `yolo-model/`: Contains YOLO model weights and class names.
- `docs/`: Project reports and presentations.
