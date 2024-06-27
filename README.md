# Aruco Marker Detection and Camera Movement Commands

This project detects Aruco markers in a given MP4 video file and extracts specific information about each detected marker. The detected markers are marked with a green rectangular frame and their IDs in the output video. Additionally, the project outputs a CSV file containing the frame ID, marker ID, 2D corner points, distance to the camera, and yaw angle of the markers. 

In the second part of the project, the system processes live video from the PC camera to determine movement commands (up, down, left, right, forward, backward, turn-left, turn-right) to align the live view with a target frame containing Aruco markers.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Output](#output)
- [Camera Parameters](#camera-parameters)

## Installation

To set up the project, you need to have Python installed on your machine. Follow the steps below to install the required dependencies:

1. Clone the repository or download the project files.
2. Navigate to the project directory.
3. Install the dependencies using the following command:

    ```sh
    pip install -r requirements.txt
    ```

## Usage

### Part 1: Aruco Marker Detection in Video

To run the Aruco marker detection, use the following command:

```sh
python part1.py
```
Ensure that the video file (`challengeB.mp4`) is in the same directory as `part1.py`. The script will process the video and generate the output CSV and video files.

### Part 2: Camera Movement Commands Based on Live Video

To run the live video processing for movement commands, use the following command:

```sh
python part2.py
```

Ensure that the target frame image (`target_frame.jpg`) is in the same directory as `part2.py`. The script will process the live video from the PC camera and display movement commands on the video feed.

## Output

### Part 1

- **CSV File**: `aruco_detection_results.csv` in the `results` folder containing the following columns:
  - Frame ID
  - QR ID
  - QR 2D (4 corner points in frame coordinates)
  - Distance (to the camera)
  - Yaw (angle with respect to the camera "lookAt" point)

- **Output Video**: `output_with_aruco.mp4` in the `results` folder with detected markers marked with a green rectangular frame and their IDs.

### Part 2

- **Live Video Feed**: The live video feed will display the detected markers and movement commands (up, down, left, right, forward, backward, turn-left, turn-right) based on the comparison with the target frame.

## Camera Parameters

The intrinsic camera parameters and distortion coefficients used in this project are taken from a [source](https://tellopilots.com/threads/camera-intrinsic-parameter.2620/). If you need to use different camera parameters, update the `camera_matrix` and `dist_coeffs` variables in both `part1.py` and `part2.py`.
