import numpy as np
import math
import rospy
from tf import transformations as tfs
from geometry_msgs.msg import TransformStamped, PoseStamped
import numpy.matlib as npm
from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf.transformations import quaternion_matrix
from tf.transformations import quaternion_from_matrix
import tf2_ros
from collections import deque
from geometry_msgs.msg import Vector3


def posemsg_to_quaternion(pose_msg):
    """
    Extracts quaternion vector from pose message.
    Args:
        pose_msg(PoseStamped / Pose): Pose msg.

    Returns:
        np.array: quaternion vector - [x, y, z, w]
    """
    try:
        return np.array(
            [
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w,
            ]
        )
    except AttributeError:
        return np.array(
            [
                pose_msg.orientation.x,
                pose_msg.orientation.y,
                pose_msg.orientation.z,
                pose_msg.orientation.w,
            ]
        )


def posemsg_to_translation(pose_msg):
    """
    Extracts translation vector from pose message.
    Args:
        pose_msg(PoseStamped / Pose): Pose msg.

    Returns:
        np.array: translation vector - [x, y, z]
    """
    try:
        return np.array(
            [
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z,
            ]
        )
    except AttributeError:
        return np.array(
            [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        )


def posemsg_to_array(pose_msg):
    """
    Extracts transformation matrix from pose message.
    Args:
        pose_msg(PoseStamped / Pose): Pose msg.

    Returns:
        np.array: transformation matrix [4,4]
    """
    return np.dot(
        tfs.translation_matrix(posemsg_to_translation(pose_msg)),
        tfs.quaternion_matrix(posemsg_to_quaternion(pose_msg)),
    )


def tarray_to_posemsg(transform_array, frame_id):
    """
    Composes pose message from transform array.
    Args:
        transform_array(np.array): Transformation matrix.
        frame_id(str): Parent frame ID

    Returns:
        PoseStamped: Pose msg.
    """
    # Get quaternions
    q = tfs.quaternion_from_matrix(transform_array)

    # Create transform message
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.header.stamp = rospy.Time.now()
    p.pose.position.x = transform_array[0, 3]
    p.pose.position.y = transform_array[1, 3]
    p.pose.position.z = transform_array[2, 3]
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]

    return p


def tmsg_to_posemsg(transform_msg):
    """
    Converts transform message to pose message.
    Args:
        transform_msg(TransformStamped): Transform msg.

    Returns:
        PoseStamped: Pose msg.
    """
    p = PoseStamped()
    p.header.frame_id = transform_msg.child_frame_id
    p.header.stamp = rospy.Time.now()
    p.pose.position.x = transform_msg.transform.translation.x
    p.pose.position.y = transform_msg.transform.translation.y
    p.pose.position.z = transform_msg.transform.translation.z
    p.pose.orientation.x = transform_msg.transform.rotation.x
    p.pose.orientation.y = transform_msg.transform.rotation.y
    p.pose.orientation.z = transform_msg.transform.rotation.z
    p.pose.orientation.w = transform_msg.transform.rotation.w

    return p


def tmsg_to_quaternion(transform_msg):
    """
    Extracts quaternion vector from transform message.
    Args:
        transform_msg(TransformStamped): Transform msg.

    Returns:
        np.array: quaternion vector - [x, y, z, w]
    """
    return np.array(
        [
            transform_msg.transform.rotation.x,
            transform_msg.transform.rotation.y,
            transform_msg.transform.rotation.z,
            transform_msg.transform.rotation.w,
        ]
    )


def tmsg_to_translation(transform_msg):
    """
    Extracts translation vector from transform message.
    Args:
        transform_msg(TransformStamped): Transform msg.

    Returns:
        np.array: translation vector - [x, y, z]
    """
    return np.array(
        [
            transform_msg.transform.translation.x,
            transform_msg.transform.translation.y,
            transform_msg.transform.translation.z,
        ]
    )


def tmsg_to_rotation_matrix(transform_msg):
    """
    Extracts rotation matrix from transform message.
    Args:
        transform_msg(TransformStamped): Transform msg.

    Returns:
        np.array: transformation matrix [4,4]
    """
    return tfs.quaternion_matrix(tmsg_to_quaternion(transform_msg))[:3, :3]


def tmsg_to_array(transform_msg):
    """
    Extracts transformation matrix from transform message.
    Args:
        transform_msg(TransformStamped): Transform msg.

    Returns:
        np.array: transformation matrix [4,4]
    """
    return np.dot(
        tfs.translation_matrix(tmsg_to_translation(transform_msg)),
        tfs.quaternion_matrix(tmsg_to_quaternion(transform_msg)),
    )


def tarray_to_tmsg(transform_array, frame_id, child_frame_id):
    """
    Composes transform message from transform array.
    Args:
        transform_array(np.array): Transformation matrix.
        frame_id(str): Parent frame ID
        child_frame_id(str): Child frame ID.

    Returns:
        TransformStamped: Transform msg.
    """
    # Get quaternions
    q = tfs.quaternion_from_matrix(transform_array)

    # Create transform message
    t = TransformStamped()
    t.header.frame_id = frame_id
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child_frame_id
    t.transform.translation.x = transform_array[0, 3]
    t.transform.translation.y = transform_array[1, 3]
    t.transform.translation.z = transform_array[2, 3]
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    return t


def mean_quaternion(q_list):
    """
    Computes the average orientation from the provided quaternions.
    Based on: http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf
    Args:
        q_list(list[np.array]): List of normalized quaternion vectors to average. q = [x, y, z, w]

    Returns:
        np.array: mean quaternion
    """
    # Index checking
    n_quat = len(q_list)

    # Construct Q matrix
    Q = np.zeros([4, n_quat])
    for i in range(n_quat):
        Q[:, i] = np.asarray(q_list[i])

    # Compute M matrix
    M = np.dot(Q, Q.T)

    # Compute mean orientation as the eigenvector corresponding to the largest eigenvalue
    w, v = np.linalg.eig(M)
    q_avg = v[:, np.argmax(w)]

    return q_avg


def mean_transform(transform_list):
    """
    Computes the average transform from the provided list.
    Args:
        transform_list(list[np.array]): List of transforms to average.

    Returns:
        np.array: mean transform
    """
    # Decompose transforms into translation and rotation
    q_lst = []
    t_lst = []
    for transform in transform_list:
        t_lst.append(transform[:, 3])
        q = tfs.quaternion_from_matrix(transform)
        q_lst.append(q)

    # Get position average
    t_avg = np.mean(np.asarray(t_lst), axis=0)

    # Get rotation average
    q_avg = mean_quaternion(q_lst)

    # Construct transform
    transform_avg = tfs.quaternion_matrix(q_avg)
    transform_avg[:, 3] = t_avg

    return transform_avg


def align_tf_sym(input_tf, target_tf, symmetry_axis, allow_flipping=True):
    """
    Aligns input matrix to target, while not rotating it around the axis of symmetry.
    Args:
        input_tf(np.array): Input transform.
        target_tf(np.array): Target transform.
        symmetry_axis(int): Axis of symmetry - rotation around this axis will not be aligned.
        allow_flipping: It True, allow the symmetry axis to be flipped in the solution.

    Returns:
        np.array: Aligned transform.
    """
    # Compute rotational alignment
    aligned_tf = align_tf_rotation_sym(
        input_tf, target_tf, symmetry_axis, allow_flipping
    )

    # Compute translation
    aligned_tf[:3, 3] = target_tf[:3, 3]

    return aligned_tf


def align_tf_rotation_sym(
    input_tf, target_tf, symmetry_axis, allow_flipping=True
):
    """
    Aligns rotation of the input matrix to target, taking into account the axis of symmetry.
    Args:
        input_tf(np.array): Input transform.
        target_tf(np.array): Target transform.
        symmetry_axis(int): Axis of symmetry - rotation around this axis will not be aligned.
        allow_flipping: It True, allow the symmetry axis to be flipped in the solution.

    Returns:
        np.array: Aligned transform.
    """
    # Copy tfs locally
    input_tf = input_tf.copy()
    target_tf = target_tf.copy()

    # Handle flipping
    flip = False
    if allow_flipping:
        symvec_input = input_tf[:3, symmetry_axis]
        symvec_target = target_tf[:3, symmetry_axis]
        if np.dot(symvec_input, symvec_target) < 0:
            flip = True

    # Define axes
    if symmetry_axis == 0:
        axes = "rzyx"
    elif symmetry_axis == 1:
        axes = "rxzy"
    elif symmetry_axis == 2:
        axes = "rxyz"
    else:
        raise ValueError("Invalid symmetry axis: must be 0, 1 or 2")

    # Flip tf if required
    if flip:
        input_tf = np.dot(input_tf, tfs.euler_matrix(math.pi, 0, 0, axes))

    # Get transform between poses
    t_input_target = np.dot(np.linalg.inv(input_tf), target_tf)

    # Perform alignment
    rpy = tfs.euler_from_matrix(t_input_target, axes)
    t_aligned = np.dot(input_tf, tfs.euler_matrix(rpy[0], rpy[1], 0, axes))

    # Flip result back
    if flip:
        t_aligned = np.dot(t_aligned, tfs.euler_matrix(math.pi, 0, 0, axes))

    return t_aligned


def quaternion_conj(q):
    """
    Computes the conjugate of the quaternion q.
    Args:
        q(np.array): Quaternion of the form [x, y, z, w]

    Returns:
        np.array: conjugate quaternion of the form [x, y, z, w]
    """
    q_conj = np.array(q)
    q_conj[:3] = -q_conj[:3]
    return q_conj


def quaternion_inv(q):
    """
    Computes the inverse of the quaternion q.
    Args:
        q(np.array): Quaternion of the form [x, y, z, w]

    Returns:
        np.array: inverse quaternion of the form [x, y, z, w]
    """
    return quaternion_conj(q) / np.dot(q, q)


def quaternion_mult(q1, q0):
    """
    Multiplies the two input quaternions q1*q0.
    This corresponds to rotation q0 followed by q1.
    Args:
        q1(np.array): Quaternion of the form [x, y, z, w]
        q0(np.array): Quaternion of the form [x, y, z, w]

    Returns:
        np.array: multiplication result of the form [x, y, z, w]
    """
    # Decompose input
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    return np.array(
        [
            x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
            -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
            x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
            -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
        ],
        dtype=np.float64,
    )


def quaternion_diff(q1, q0):
    """
    Computes a quaternion difference qd, such that qd * q0 = q1
    Args:
        q1(np.array): Quaternion of the form [x, y, z, w]
        q0(np.array): Quaternion of the form [x, y, z, w]

    Returns:
        np.array: quaternion difference of the form [x, y, z, w]
    """
    return quaternion_mult(q1, quaternion_inv(q0))


def quaternion_angle(q):
    """
    Computes the minimum scalar rotation angle of the quaternion.
    Args:
        q(np.array): Quaternion of the form [x, y, z, w]

    Returns:
        float: rotation angle, in range between -pi and pi
    """
    # Compute angle
    theta = 2 * math.atan2(np.linalg.norm(q[:3]), q[3])
    # Wrap angle to lie between -pi and pi -> double coverage of the hypersphere
    result = ((theta + math.pi) % (2 * math.pi)) - math.pi
    if result == -math.pi:
        result = math.pi
    return result


def quaternion_diff_angle(q1, q0):
    """
    Computes the minimum angle between the two quaternions.
    Args:
        q1(np.array): Quaternion of the form [x, y, z, w]
        q0(np.array): Quaternion of the form [x, y, z, w]

    Returns:
        float: rotation angle
    """
    return quaternion_angle(quaternion_diff(q1, q0))


def averageQuaternions(Q):
    # Number of quaternions to average
    M = Q.shape[0]
    A = npm.zeros(shape=(4, 4))

    for i in range(0, M):
        q = Q[i, :]
        # multiply q with its transposed version q' and add A
        A = np.outer(q, q) + A

    # scale
    A = (1.0 / M) * A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:, eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:, 0].A1)


def weightedAverageQuaternions(Q, w):
    # Number of quaternions to average
    M = Q.shape[0]
    A = npm.zeros(shape=(4, 4))
    weightSum = 0

    for i in range(0, M):
        q = Q[i, :]
        A = w[i] * np.outer(q, q) + A
        weightSum += w[i]

    # scale
    A = (1.0 / weightSum) * A

    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)

    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:, eigenValues.argsort()[::-1]]

    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:, 0].A1)


def get_transform(
    tf_buffer: Buffer,
    source_frame: str,
    dest_frame: str,
    time: rospy.Time = None,
    max_wait_time: float = 0.0,
    is_active: bool = False,
    supress_error_msg=False,
):
    """
    Returns the transform between two frames.

            Parameters:
                    tf_buffer (Buffer): A tf2_ros transform buffer
                    source_frame (str): The source we're transforming from
                    dest_frame (str): The source we're transforming to
                    time (rospy.Time): Optional. The timestamp of the transform. Defaults to 'rospy.Time.now()'
                    max_wait_time (float): Optional. The maximum time in seconds we should wait for the transform to
                            be available. Defaults to 0 (no waiting time)
                    is_active (bool): Optional. If set to True, return the active transform between the two frames. For
                            more info refer to https://en.wikipedia.org/wiki/Active_and_passive_transformation. Defaults
                            to False.
                    supress_error_msg (bool): Optional. If set to True, supresses error messages. Defaults to False

            Returns:
                    tf_array (numpy.ndarray): The homogeneous transform as a 4x4 numpy array
                    p (numpy.ndarray): The transform translation as a 1x3 numpy array
                    q (numpy.ndarray): The transform rotation quaternion (x, y, z, w) as 1x4 numpy array
                    tf_msg (geometry_msgs.msg.TransformStamped): The transform stamped message, as returned from
                            lookup_transform
    """
    if time is None:
        time = rospy.Time.now()
    if is_active:
        dest_frame, source_frame = source_frame, dest_frame
    try:
        tf_msg = tf_buffer.lookup_transform(
            target_frame=dest_frame,
            source_frame=source_frame,
            time=time,
            timeout=rospy.Duration(secs=0, nsecs=int(max_wait_time * 1e9)),
        )
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ) as e:
        if not supress_error_msg:
            rospy.logerr(f"{type(e)}: {e}")
        return None, None, None, None
    q_msg = tf_msg.transform.rotation
    q = np.array([q_msg.x, q_msg.y, q_msg.z, q_msg.w])
    p = np.array(
        [
            tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z,
        ]
    )
    tf_array = quaternion_matrix(q)
    tf_array[0, 3] = p[0]
    tf_array[1, 3] = p[1]
    tf_array[2, 3] = p[2]
    return tf_array, p, q, tf_msg


def get_pose(
    tf_buffer: Buffer,
    reference_frame: str,
    frame: str,
    time=None,
    max_wait_time=0.0,
    supress_error_msg=False,
):
    """
    Returns the pose of one frame relative to the other.

            Parameters:
                    tf_buffer (Buffer): A tf2_ros transform buffer
                    reference_frame (str): The base frame of the pose
                    frame (str): The name of the frame we want to know the pose of
                    time (rospy.Time): Optional. The timestamp of the pose. Defaults to 'rospy.Time.now()'
                    max_wait_time (float): Optional. The maximum time in seconds we should wait for the pose to
                            be available. Defaults to 0 (no waiting time)

            Returns:
                    tf_array (numpy.ndarray): The pose in homogeneous transform form, as a 4x4 numpy array
                    p (numpy.ndarray): The pose's position as 1x3 numpy array
                    q (numpy.ndarray): The pose's orientation quaternion (x, y, z, w) as a 1x4 numpy array
                    tf_msg (geometry_msgs.msg.TransformStamped): The transform stamped message, as returned from
                            lookup_transform
    """
    return get_transform(
        tf_buffer,
        frame,
        reference_frame,
        time,
        max_wait_time,
        False,
        supress_error_msg,
    )


def sample_stable_poses(
    tf_buffer: Buffer,
    frame: str,
    reference_frame: str,
    sample_size: int,
    timeout: float = 1.0,
    window_size: int = None,
    pos_tol: float = 1e-3,
    ori_tol: float = 1e-3,
    pool_rate: int = 10,
    verbose=False,
):
    if window_size is None:
        window_size = sample_size
    p_window = deque(maxlen=window_size)
    q_window = deque(maxlen=window_size)
    H_sample = []
    p_sample = []
    q_sample = []
    P_msg_sample = []
    rate = rospy.Rate(pool_rate)
    max_wait_time_ns = int(timeout * 1e9)

    start_time = None

    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        H, p, q, P_msg = get_pose(
            tf_buffer=tf_buffer,
            reference_frame=reference_frame,
            frame=frame,
            time=rospy.Time.now(),
            max_wait_time=0.5,
            supress_error_msg=True,
        )
        if p is not None:
            # We got a new sample
            # Lets check if the current window is stable
            p_window.append(p)
            q_window.append(q)
            # If the window is not full, we do nothing
            if len(p_window) == window_size:
                p_np = np.array(p_window)
                q_np = np.array(q_window)
                p_mean_std = np.mean(np.std(p_np, axis=0))
                q_mean_std = np.mean(np.std(q_np, axis=0))
                if p_mean_std < pos_tol and q_mean_std < ori_tol:
                    # If the current window stds are inside the tolerances, we store the sample
                    H_sample.append(H)
                    p_sample.append(p)
                    q_sample.append(q)
                    P_msg_sample.append(P_msg)
                    if verbose:
                        rospy.loginfo(
                            "New sample stored"
                            f" ({len(p_sample)}/{sample_size})"
                        )
                else:
                    # If not, we clear the current sample list because the tf might have moved, and we can no longer trust the stored values.
                    H_sample.clear()
                    p_sample.clear()
                    p_sample.clear()
                    P_msg_sample.clear()
                    if verbose:
                        reason = ""
                        if p_mean_std >= pos_tol:
                            reason = (
                                f"mean p std = {p_mean_std}, max {pos_tol}"
                            )
                        if q_mean_std >= ori_tol:
                            if reason != "":
                                reason += ", "
                            reason += (
                                f"mean q std = {q_mean_std}, max {ori_tol}"
                            )
                        rospy.loginfo(
                            f"Frame {frame} not stable ({reason}), clearing"
                            " sample list."
                        )
        delta_ns = (rospy.Time.now() - start_time).to_nsec()
        if delta_ns >= max_wait_time_ns:
            return None, None, None, None
        elif len(p_sample) == sample_size:
            return H_sample, p_sample, q_sample, P_msg_sample
        rate.sleep()


def sample_stable_transforms(
    tf_buffer: Buffer,
    source_frame: str,
    dest_frame: str,
    sample_size: int,
    timeout: float = 1.0,
    window_size: int = None,
    pos_tol: float = 5e-4,
    ori_tol: float = 5e-4,
    pool_rate: int = 10,
    verbose=False,
):
    return sample_stable_poses(
        tf_buffer,
        source_frame,
        dest_frame,
        sample_size,
        timeout,
        window_size,
        pos_tol,
        ori_tol,
        pool_rate,
        verbose,
    )


def pos_ori_from_matrix(T_matrix):
    return T_matrix[:3, 3], quaternion_from_matrix(T_matrix)


def process_positions(positions):
    return np.mean(positions, axis=0)


def process_orientations(orientations):
    ori_list_array = np.array(orientations)
    ori_list_array[:, [0, 3]] = ori_list_array[:, [3, 0]]
    avg = averageQuaternions(ori_list_array)
    avg[[0, 3]] = avg[[3, 0]]
    return avg


def process_H_matrices(H_mat_list):
    p_list = [H_mat[:3, 3] for H_mat in H_mat_list]
    q_list = [quaternion_from_matrix(H_mat) for H_mat in H_mat_list]
    p = process_positions(p_list)
    q = process_orientations(q_list)
    H_ret = quaternion_matrix(q)
    H_ret[:3, 3] = p
    return H_ret


def vector3_msg_to_numpy(vector: Vector3):
    return np.array([vector.x, vector.y, vector.z])


def numpy_to_vector3_msg(array: np.ndarray):
    v = Vector3()
    v.x = array[0]
    v.y = array[1]
    v.z = array[2]
    return v
