
import open3d as o3d
import numpy as np
from PIL import Image

from quaternion import as_rotation_matrix, quaternion
from scipy.spatial.transform import Rotation as R


from skimage.transform import resize

import cv2

neck_urdf_path = "../arx5-sdk/models/arx5_iphone15pro.urdf"
arm_urdf_path = "../arx5-sdk/models/arx5_webcam.urdf"

# neck EE frame (body frame/ world frame in tabletop setup) to camera frame transformation matrix
# T_ET = np.array(
#         [[-0.04542549,  0.05847284, -0.99725496,  0.00836426],
#             [-0.01272158, -0.99823837, -0.05795102,  0.00347599],
#             [-0.99888672,  0.01005421,  0.04608934,  0.00240803],
#             [ 0.,          0.,          0.,          1.,        ]]
#         )

T_ET = np.array(
[[-0.02314089,  0.03232146, -0.9992096,   0.00359406],
 [-0.00316231, -0.99947461, -0.0322568,   0.00362895],
 [-0.99972721,  0.00241336,  0.02323095,  0.0024466 ],
 [ 0.,          0.,          0.,          1.,        ]]
        )




# Right arm to neck EE frame (body frame/ world frame in tabletop setup) transformation matrix
T_right_shoulder_to_body = np.array([
            [1,  0,     0,       0.18],
            [0, -0.707, -0.707, -0.14563],
            [0, 0.707, -0.707,  0.037426],
            [0,  0,     0,          1]
            ])  
# Left arm to neck EE frame (body frame/ world frame in tabletop setup) transformation matrix
T_left_shoulder_to_body = np.array([
            [1,  0,     0,       0.18],
            [0, -0.707, 0.707, 0.14563],
            [0, -0.707, -0.707,  0.037426],
            [0,  0,     0,          1]
            ])  
neck_start_pose = np.array([0.35, 0, 0.025, 0, np.pi/6, 0])
left_start_body_pose = np.array([0.6, 0.4, -0.2, np.pi, 0, 0]) 
right_start_body_pose = np.array([0.6, -0.4, -0.2, np.pi, 0, 0]) 

init_neck_pose = np.array(
        [[ 0.71, -0.,    0.71,  0.33],
        [ 0.,    1.,    0.,   -0.,  ],
        [-0.71,  0.,    0.71,  0.1, ],
        [ 0.,    0.,    0.,    1.,  ]])




pick_place_neck_start_pose = np.array([0.38, -0.14, 0.03, 0, np.pi/2.5,0])


# tabletop workspace
x_min = 0.2
x_max = 0.9
y_min = -0.8
y_max = 0.4
z_min = -0.51
z_max = 0.4


def resize_depth_and_rgb(depth_image, rgb_image, resolution):



    pil_depth = Image.fromarray(depth_image)
    pil_depth = np.asarray(pil_depth)


    new_shape = (resolution[0], resolution[1])  # Scale up by a factor of 2
    reshaped_depth = resize(pil_depth, new_shape, mode='reflect', order=0)

    
    
    rgb = Image.fromarray(rgb_image)
    reshaped_rgb = rgb.resize((resolution[1], resolution[0]), )#Image.NEAREST)
    reshaped_rgb = np.asarray(reshaped_rgb)
    


    return reshaped_depth, reshaped_rgb

def inverse_transformation_matrix(T):
    """Compute the inverse of a 4x4 homogeneous transformation matrix."""
    R_inv = T[:3, :3].T  # Transpose of the rotation matrix
    t_inv = -R_inv @ T[:3, 3]  # Invert the translation
    
    # Construct the inverse transformation matrix
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    return T_inv




def quaternion_to_homogeneous_matrix(pose):
    # Convert quaternion to rotation matrix
    r = R.from_quat(pose[-4:])
    rotation_matrix = r.as_matrix()
    
    # Create the homogeneous transformation matrix (4x4)
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix  # Top-left 3x3 is the rotation matrix
    homogeneous_matrix[:3, 3] = pose[:3]  # Top-right 3x1 is the translation vector
    
    return homogeneous_matrix


def pose_to_homogeneous(pose):
    """Convert a 6D pose (x, y, z, roll, pitch, yaw) into a 4x4 homogeneous matrix."""
    position = pose[:3]
    rpy_angles = pose[3:]
    
    # Convert Euler angles to a rotation matrix
    rotation_matrix = R.from_euler('xyz', rpy_angles, degrees=False).as_matrix()
    
    # Create homogeneous transformation matrix
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix
    homogeneous_matrix[:3, 3] = position
    return homogeneous_matrix

def transform_pose(pose_homogeneous, transformation_matrix, is_quaternion = 0, is_rpy = 0):
    """Transform a 6D pose using a transformation matrix."""
    # Convert the input pose to a homogeneous matrix
    
    # Apply the transformation matrix (multiplication)
    transformed_pose_homogeneous = transformation_matrix @ pose_homogeneous
    
    # Extract position and orientation from the transformed homogeneous matrix
    transformed_position = transformed_pose_homogeneous[:3, 3]
    transformed_rotation_matrix = transformed_pose_homogeneous[:3, :3]
    
    # Convert the rotation matrix back to Euler angles (xyz convention)

    transformed_rpy = R.from_matrix(transformed_rotation_matrix).as_euler('xyz')
    transformed_rotvec = R.from_matrix(transformed_rotation_matrix).as_rotvec()
    quaternion = R.from_matrix(transformed_rotation_matrix).as_quat()  # Quaternion is [x, y, z, w]

    # Return the transformed 6D pose
    if is_quaternion: 
        return np.concatenate([transformed_position, quaternion])

    if is_rpy:
        return np.concatenate([transformed_position, transformed_rpy])








def quaternion_camera_pose_to_extrinsic_matrix(camera_pose):
    """
    Convert a pose to an extrinsic matrix.
    """

    extrinsic_matrix = np.eye(4)
    qx, qy, qz, qw, px, py, pz = camera_pose.qx, camera_pose.qy, camera_pose.qz, camera_pose.qw, camera_pose.tx, camera_pose.ty, camera_pose.tz
    extrinsic_matrix[:3, :3] = as_rotation_matrix(quaternion(qw, qx, qy, qz))
    extrinsic_matrix[:3, -1] = [px, py, pz]

    return extrinsic_matrix



def rpy_pose_extrinsic_matrix(pose):
    x, y, z, roll, pitch, yaw = pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]
    # Create rotation matrix from RPY angles
    rotation_matrix = R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_matrix()
    
    # Construct the 4x4 extrinsic transformation matrix
    extrinsic_matrix = np.eye(4)
    extrinsic_matrix[:3, :3] = rotation_matrix
    extrinsic_matrix[:3, 3] = [x, y, z]
    
    return extrinsic_matrix

def mat2pose(matrix):
    # Extract the rotation matrix
    R = matrix[:3, :3]
    
    # Extract the translation vector
    translation = matrix[:3, 3]
    
    # Calculate pitch (theta)
    theta = np.arcsin(-R[2, 0])
    
    # Calculate the cosine of theta for use in other calculations
    cos_theta = np.cos(theta)
    
    # Calculate yaw (psi)
    if cos_theta != 0:
        psi = np.arctan2(R[1, 0] / cos_theta, R[0, 0] / cos_theta)
    else:
        psi = 0  # When cos_theta is zero, the yaw cannot be computed directly
    
    # Calculate roll (phi)
    if cos_theta != 0:
        phi = np.arctan2(R[2, 1] / cos_theta, R[2, 2] / cos_theta)
    else:
        phi = 0  # When cos_theta is zero, the roll cannot be computed directly

    yaw = np.array(psi)
    pitch = np.array(theta)
    roll = np.array(phi)
    
    # Convert radians to degrees
    
    # yaw = np.degrees(psi)
    # pitch = np.degrees(theta)
    # roll = np.degrees(phi)
    
    # Return results
    return [translation[0], translation[1], translation[2], yaw, pitch, roll]


def rgbd_to_camera_frame_pcd(depth, rgb, intrinsics, depth_scale=1000.0,):



    depth_width = int(depth.shape[1])
    depth_height = int(depth.shape[0])




    depth_o3d = o3d.geometry.Image(
        np.ascontiguousarray(depth_scale * depth).astype(np.float32)
    )
    rgb_o3d = o3d.geometry.Image(
        np.ascontiguousarray(rgb).astype(np.uint8)
    )

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgb_o3d, depth_o3d, convert_rgb_to_intensity=False
    )



    camera_intrinsics = o3d.camera.PinholeCameraIntrinsic(
        width=int(depth_width),
        height=int(depth_height),
        fx=intrinsics[0, 0] * depth_width / 720,
        fy=intrinsics[1, 1] * depth_height / 960,
        cx=intrinsics[0, 2] * depth_width / 720,
        cy=intrinsics[1, 2] * depth_height / 960,
    )


    temp = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, camera_intrinsics)
    temp.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    
    

    return temp





def reshape_depth_and_rgb(self, depth_image, rgb_image):


    # pil_depth = Image.fromarray(depth_image)
    # reshaped_depth = pil_depth.resize((self.depth_width, self.depth_height))
    reshaped_depth = np.asarray(depth_image)

    
    
    rgb = Image.fromarray(rgb_image)
    reshaped_rgb = rgb.resize((self.depth_width, self.depth_height))
    reshaped_rgb = np.asarray(reshaped_rgb)
    

    return reshaped_depth, reshaped_rgb



def center_crop_and_resize(image, crop_size, target_size):
    # Calculate the center of the image
    h, w = image.shape[:2]
    crop_h, crop_w = crop_size

    # Define the cropping box (centered)
    start_x = (w - crop_w) // 2
    start_y = (h - crop_h) // 2

    # Perform the center crop
    cropped_image = image[start_y:start_y+crop_h, start_x:start_x+crop_w]

    # Resize the cropped image to the target size
    resized_image = cv2.resize(cropped_image, target_size)

    return resized_image