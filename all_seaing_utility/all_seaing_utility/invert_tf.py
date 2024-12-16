import numpy as np
from scipy.spatial.transform import Rotation as R

def invert_transform(original_tf):
    """
    Invert a transform that includes a translation vector and a quaternion.

    Args:
        original_tf (list): A list of 7 values [tx, ty, tz, qx, qy, qz, qw]
                            where tx, ty, tz are translations and 
                            qx, qy, qz, qw is the quaternion.
    
    Returns:
        list: Inverted transform [tx_inv, ty_inv, tz_inv, qx_inv, qy_inv, qz_inv, qw_inv]
    """
    # Split translation and quaternion
    translation = np.array(original_tf[:3])  # [tx, ty, tz]
    quat = original_tf[3:]                   # [qx, qy, qz, qw]

    # Invert the quaternion
    rotation = R.from_quat(quat)
    inv_rotation = rotation.inv()  # Inverted rotation
    inv_quat = inv_rotation.as_quat()

    # Apply inverted rotation to the negated translation vector
    inv_translation = -inv_rotation.apply(translation)

    # Combine inverted translation and quaternion
    inverted_tf = list(inv_translation) + list(inv_quat)
    return inverted_tf

# Original transform: T_lidar_camera
T_lidar_camera = [
    -0.05170783940451991,
    -0.05187798511171696,
    -0.39700142732955157,
    -0.658623187450687,
    0.013786635156961146,
    0.004802448059092996,
    0.7523312848313473
]

# Invert the transform
T_camera_lidar = invert_transform(T_lidar_camera)

# Print the result
print("Inverted Transform (Camera to Lidar):", T_camera_lidar)
