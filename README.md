# Aruco Marker Detection in Video

This project detects Aruco markers in a given MP4 video file and extracts specific information about each detected marker. The detected markers are marked with a green rectangular frame and their IDs in the output video. Additionally, the project outputs a CSV file containing the frame ID, marker ID, 2D corner points, distance to the camera, and yaw angle of the markers.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Output](#output)
- [Camera Parameters](#camera-parameters)
- [License](#license)

## Installation

To set up the project, you need to have Python installed on your machine. Follow the steps below to install the required dependencies:

1. Clone the repository or download the project files.
2. Navigate to the project directory.
3. Install the dependencies using the following command:

    ```sh
    pip install -r requirements.txt
    ```

## Usage

To run the Aruco marker detection, use the following command:

```sh
python main.py
```
Ensure that the video file (`challengeB.mp4`) is in the same directory as `main.py`. The script will process the video and generate the output CSV and video files.

## Output

- **CSV File**: `aruco_detection_results.csv` in the `results` folder containing the following columns:
  - Frame ID
  - QR ID
  - QR 2D (4 corner points in frame coordinates)
  - Distance (to the camera)
  - Yaw (angle with respect to the camera "lookAt" point)

- **Output Video**: `output_with_aruco.mp4` in the `results` folder with detected markers marked with a green rectangular frame and their IDs.

## Camera Parameters

The intrinsic camera parameters and distortion coefficients used in this project are taken from a [source](https://tellopilots.com/threads/camera-intrinsic-parameter.2620/). If you need to use different camera parameters, update the `camera_matrix` and `dist_coeffs` variables in `main.py`.

