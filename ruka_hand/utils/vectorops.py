import cv2
import numpy as np
import torch


def moving_average(vector, moving_average_queue, limit):
    moving_average_queue.append(vector)

    if len(moving_average_queue) > limit:
        moving_average_queue.pop(0)

    mean_vector = np.mean(moving_average_queue, axis=0)
    return mean_vector


def normalize_vector(vector):
    return vector / np.linalg.norm(vector)


def calculate_angle(coord_1, coord_2, coord_3, in_degrees=False, finger_id=1):
    vector_1 = coord_2 - coord_1
    vector_2 = coord_3 - coord_2

    inner_product = np.inner(vector_1, vector_2)
    norm = np.linalg.norm(vector_1) * np.linalg.norm(vector_2)
    angle = np.arccos(inner_product / norm)

    cross_product = np.cross(vector_1, vector_2)
    if finger_id == 0:
        if cross_product[2] > 0:  # Info for the thumb
            angle = -angle
    else:
        if cross_product[0] > 0:
            angle = -angle

    if in_degrees:
        angle = np.degrees(angle)

    return angle


def calculate_joint_angles(keypoints):  # This is only for the top four fingers
    # keypoints: (5,5,5) - first element is for the wrist and the thumb
    joint_angles = []
    for finger_id in range(5):
        curr_joint_angles = []
        # Calc the knuckle angle
        for i in range(1, 4):
            curr_joint_angles.append(
                calculate_angle(
                    keypoints[finger_id][i - 1],
                    keypoints[finger_id][i],
                    keypoints[finger_id][i + 1],
                    in_degrees=True,
                    finger_id=finger_id,
                )
            )

        joint_angles.append(np.array(curr_joint_angles))

    joint_angles = np.stack(joint_angles, axis=0)
    np.printoptions(precision=2)
    # print(f"MCPS: {joint_angles[:,0]}")

    return joint_angles


def calculate_fingertips(keypoints):
    # print(f"fingertips: {keypoints[:, -1, :]}")
    return keypoints[:, -1, :]


def convert_keypoints(keypoints, convert_type="fingertips"):
    converted = []
    for kp in keypoints:
        if convert_type == "fingertips":
            converted.append(calculate_fingertips(kp))
        else:
            converted.append(calculate_joint_angles(kp))

    if isinstance(converted[0], np.ndarray):
        converted = np.stack(converted, axis=0)
    else:
        converted = torch.stack(converted, dim=0).to(converted[0].device)

    return converted


def turn_frame_to_homo_mat(frame):
    t = frame[0]
    R = frame[1:]

    homo_mat = np.zeros((4, 4))
    homo_mat[:3, :3] = np.transpose(R)
    homo_mat[:3, 3] = t
    homo_mat[3, 3] = 1

    return homo_mat


def turn_frames_to_homo(rvec, tvec):
    homo_mat = np.zeros((4, 4))
    homo_mat[:3, :3] = rvec
    homo_mat[:3, 3] = tvec
    homo_mat[3, 3] = 1
    return homo_mat


def linear_transform(curr_val, source_bound, target_bound):
    multiplier = (target_bound[1] - target_bound[0]) / (
        source_bound[1] - source_bound[0]
    )
    target_val = ((curr_val - source_bound[0]) * multiplier) + target_bound[0]
    return target_val


def persperctive_transform(input_coordinates, given_bound, target_bound):
    transformation_matrix = cv2.getPerspectiveTransform(
        np.float32(given_bound), np.float32(target_bound)
    )
    transformed_coordinate = np.matmul(
        np.array(transformation_matrix),
        np.array([input_coordinates[0], input_coordinates[1], 1]),
    )
    transformed_coordinate = transformed_coordinate / transformed_coordinate[-1]

    return transformed_coordinate[0], transformed_coordinate[1]


def average_poses(transformations):
    translations = np.array([T[:3, 3] for T in transformations])
    avg_translation = np.mean(translations, axis=0)

    rotations = np.array([T[:3, :3] for T in transformations])
    avg_rotation = np.zeros((3, 3))

    # Averaging rotations using SVD
    for rot in rotations:
        avg_rotation += rot
    avg_rotation /= len(rotations)
    u, _, vt = np.linalg.svd(avg_rotation)
    avg_rotation = np.dot(u, vt)

    # Combine into transformation matrix
    avg_transform = np.eye(4)
    avg_transform[:3, :3] = avg_rotation
    avg_transform[:3, 3] = avg_translation
    return avg_transform
