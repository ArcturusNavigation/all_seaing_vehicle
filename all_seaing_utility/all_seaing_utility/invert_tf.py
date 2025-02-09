#!/usr/bin/env python3

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

def main():
    # Original tf
    original_tf = [
        -0.06897013653066601,
        0.1117286743564149,
        -0.22846882535774413,
        -0.7258609179830707,
        -0.025729045448789383,
        0.023466890166381215,
        0.6869594231328277
    ]

    # Invert the transform
    inverted_tf = invert_transform(original_tf)

    # Print the result
    print("Original Transform:", original_tf)
    print("Inverted Transform:", inverted_tf)

if __name__ == "__main__":
    main()

