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
        -0.07004260508065621,
        -0.02048465906683186,
        -0.10549336867056447,
        -0.7174633834551565,
        -0.004810627144703515,
        0.005718431558079253,
        0.6965561361498939
    ]

    # Invert the transform
    inverted_tf = invert_transform(original_tf)

    # Print the result
    print("Original Transform:", original_tf)
    print("Inverted Transform:", inverted_tf)

if __name__ == "__main__":
    main()

