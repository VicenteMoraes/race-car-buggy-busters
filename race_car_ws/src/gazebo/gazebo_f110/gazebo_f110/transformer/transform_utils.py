
import numpy as np
from tf_transformations import quaternion_from_matrix
from tf_transformations import quaternion_multiply

def transform_pose_gazebo_to_ros2(pos_g):
    """
    Transform pose from Gazebo coordinate system (G) to
    ROS2 coordinate system (R).
    """

    # -------------------------
    # 1) Define the rotation matrix that maps a vector
    #    from Gazebo (G) to ROS2 (R).
    #
    #   x^R = -y^G
    #   y^R = -x^G
    #   z^R = -z^G
    #
    # So, in matrix form:
    #   [ x^R ]   [  0   -1    0 ] [ x^G ]
    #   [ y^R ] = [ -1    0    0 ] [ y^G ]
    #   [ z^R ]   [  0    0   -1 ] [ z^G ]

    R_g_to_r = np.array([
        [ 0, -1,  0],
        [-1,  0,  0],
        [ 0,  0, -1]
    ])

    # -------------------------
    # 2) Transform the position
    #
    # pos_r = R_g_to_r * pos_g

    pos_g_np = np.array(pos_g)  # (3,)
    pos_r_np = R_g_to_r @ pos_g_np
    x_r, y_r, z_r = pos_r_np.tolist()

    # Return as (x_r, y_r, z_r)
    return (x_r, y_r, z_r)


if __name__ == "__main__":
    # Example test
    pos_g = (1.0, 2.0, 0.5)

    pos_r = transform_pose_gazebo_to_ros2(pos_g)

    print("Gazebo Pose:")
    print("  Position G =", pos_g)
    print("\nROS2 Pose:")
    print("  Position R =", pos_r)
