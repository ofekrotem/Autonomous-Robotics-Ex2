import cv2
import numpy as np

# Load the target frame
target_frame_path = 'target_frame.jpg'
target_frame = cv2.imread(target_frame_path)

# Define the aruco dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Camera parameters - Taken from: https://tellopilots.com/threads/camera-intrinsic-parameter.2620/
camera_matrix = np.array([[921.170702, 0.000000, 459.904354], [0.000000, 919.018377, 351.238301], [0.000000, 0.000000, 1.000000]])
dist_coeffs = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])

def calculate_distance(tvec):
    return np.linalg.norm(tvec)

def calculate_yaw(rvec):
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    return np.degrees(yaw)

def calculate_pose(corners):
    marker_length = 0.05  # Assuming a marker size of 5cm
    object_points = np.array([[-marker_length / 2, marker_length / 2, 0],
                              [marker_length / 2, marker_length / 2, 0],
                              [marker_length / 2, -marker_length / 2, 0],
                              [-marker_length / 2, -marker_length / 2, 0]])

    retval, rvec, tvec = cv2.solvePnP(object_points, corners, camera_matrix, dist_coeffs)
    if retval:
        dist = calculate_distance(tvec)
        yaw = calculate_yaw(rvec)
        return dist, yaw, tvec
    else:
        return None, None, None

def get_movement_command(current_poses, target_poses):
    if not current_poses or not target_poses:
        return "No QR detected"

    overall_command = "hold"
    command_counts = {"forward": 0, "backward": 0, "left": 0, "right": 0, "up": 0, "down": 0, "turn-left": 0, "turn-right": 0, "hold": 0}

    for qr_id in current_poses:
        if qr_id in target_poses:
            current_pose = current_poses[qr_id]
            target_pose = target_poses[qr_id]

            current_dist, current_yaw, current_tvec = current_pose
            target_dist, target_yaw, target_tvec = target_pose

            x_diff = target_tvec[0] - current_tvec[0]
            y_diff = target_tvec[1] - current_tvec[1]

            if target_dist > current_dist + 0.3:
                command_counts["backward"] += 1
            elif target_dist < current_dist - 0.3:
                command_counts["forward"] += 1
            elif target_yaw > current_yaw + 5:
                command_counts["turn-right"] += 1
            elif target_yaw < current_yaw - 5:
                command_counts["turn-left"] += 1
            elif y_diff > 0.3:
                command_counts["down"] += 1
            elif y_diff < -0.3:
                command_counts["up"] += 1
            elif x_diff > 0.3:
                command_counts["right"] += 1
            elif x_diff < -0.3:
                command_counts["left"] += 1
            else:
                command_counts["hold"] += 1

    overall_command = max(command_counts, key=command_counts.get)
    return overall_command

# Get the target poses
target_gray = cv2.cvtColor(target_frame, cv2.COLOR_BGR2GRAY)
target_corners, target_ids, _ = detector.detectMarkers(target_gray)
target_poses = {}

if target_ids is not None:
    for i in range(len(target_ids)):
        qr_id = target_ids[i][0]
        qr_corners = target_corners[i][0]
        target_poses[qr_id] = calculate_pose(qr_corners)

# Capture live video from PC camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = detector.detectMarkers(frame)
    current_poses = {}
    if ids is not None:
        for i in range(len(ids)):
            qr_id = ids[i][0]
            qr_corners = corners[i][0]
            current_poses[qr_id] = calculate_pose(qr_corners)

    command = get_movement_command(current_poses, target_poses)
    cv2.putText(frame, f"Command: {command}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.imshow('Live Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
