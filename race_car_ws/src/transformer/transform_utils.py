
import numpy as np
from tf_transformations import quaternion_from_matrix
from tf_transformations import quaternion_multiply

def transform_pose_gazebo_to_ros2(pos_g, quat_g):
    """
    Transform pose from Gazebo coordinate system (G) to
    ROS2 coordinate system (R).

    :param pos_g:  (x_g, y_g, z_g) position in Gazebo frame
    :param quat_g: (qx_g, qy_g, qz_g, qw_g) orientation in Gazebo frame
    :return: (pos_r, quat_r)
        pos_r  = (x_r, y_r, z_r)
        quat_r = (qx_r, qy_r, qz_r, qw_r)
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

    # -------------------------
    # 3) Transform the orientation
    #
    # We need a quaternion that represents R_g_to_r itself.
    # Then we do quaternion multiplication:
    #
    # 3.1 Convert the rotation matrix to a quaternion
    #     tf_transformations expects a 4x4 hom. transform
    #     or a 3x3 matrix. We'll convert R to a 4x4 first.

    # Build a homogeneous transform with R_g_to_r
    T_g_to_r = np.eye(4)
    T_g_to_r[:3, :3] = R_g_to_r

    q_g_to_r = quaternion_from_matrix(T_g_to_r)
    # q_g_to_r is (qx, qy, qz, qw)

    # 3.2 Multiply: q_r = q_g_to_r * q_g
    #
    # But tf_transformations uses:
    # quaternion_multiply(q1, q2) = q1 * q2
    q_r_np = quaternion_multiply(q_g_to_r, quat_g)

    # 3.3 Normalize the resulting quaternion
    norm_q = np.linalg.norm(q_r_np)
    if norm_q > 0:
        q_r_np /= norm_q

    # Return as (x_r, y_r, z_r), (qx_r, qy_r, qz_r, qw_r)
    return (x_r, y_r, z_r), tuple(q_r_np)


if __name__ == "__main__":
    # Example test
    pos_g = (1.0, 2.0, 0.5)
    quat_g = (0.0, 0.0, 0.0, 1.0)  # no rotation

    pos_r, quat_r = transform_pose_gazebo_to_ros2(pos_g, quat_g)

    print("Gazebo Pose:")
    print("  Position G =", pos_g)
    print("  Quaternion G =", quat_g)
    print("\nROS2 Pose:")
    print("  Position R =", pos_r)
    print("  Quaternion R =", quat_r)
