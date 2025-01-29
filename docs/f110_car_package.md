# F110 Car package

This package contains all ROS2 nodes and logic that will run on the car platform.

## Move to Point

Uses the 

Since we are in 2D convert the Quaternion of the vehicle to a rotation on the Z-axis (`2 * np.arctan2(z, w)`) where $z$ and $w$ are components of a Quaternion $q$. 

$$
\text{vehicle}_\text{rot} = 2 \cdot \arctan{(q_z, q_w)}
$$

Now calculate the direction vector `a - b` (component wise subtraction) for vector length $n$.

$$
dir = a^T - b = (a_1 - b_1, \ldots, a_n - b_n)^T
$$

Calculate the distance to the target by taking the norm of the direction vector.

$$
d = ||dir|| = \sqrt{\langle dir, dir \rangle}
$$

The direction vector can be converted to a rotation using `np.arctan2(*v[::-1])`. Note that the function expects the vector in the reverse order.

$$
d_\text{rot} = \arctan{(d_y, d_x)}
$$

Calculate the steering angle by the difference between the current orientation (Z-rotation) of the vehicle and the rotation of the direction vector.

$$
s = \text{vehicle}_\text{rot} - d_\text{rot}
$$

!!!Warning "Sign Flip in calculations"
    Some math modules tend to flip the sign crossing the 180° boundary thus producing $180° + 1° = -179°$. In this case flip the sign using $360°$ as reference. ($360° - 179° = 181°$)


::: f110_car.m2p_node
