import numpy as np
# import soft_utility as su
# quat = np.array(quat)
# print(quat[0])

# eul = su.quat2eul(quat,'ZYX')
# print(np.rad2deg(eul))

# q = [0.0, 0.0, 0.7071068, 0.7071068]
# eul = su.quat2eul(q,'ZYX')
# print(np.rad2deg(eul))

def normalize_quaternion(q):
    """
    将四元数归一化，使其长度为1
    q: 四元数 [w, x, y, z] 或形状为 (n, 4) 的二维数组
    返回归一化后的四元数
    """
    q = np.array(q)
    if q.ndim == 1:  # 单个四元数的情况
        norm = np.linalg.norm(q)
        if norm == 0:
            raise ValueError("零向量不能归一化为四元数")
        return q / norm
    elif q.ndim == 2:  # 批量处理多个四元数的情况
        norms = np.linalg.norm(q, axis=1, keepdims=True)
        norms[norms == 0] = 1  # 防止除以0
        return q / norms
    else:
        raise ValueError("输入应为一维或二维数组")
    
def quatinv(quat):
    """
    Calculate the inverse (conjugate) of a quaternion.
    Input:
        quat: Quaternion as a 1D array or list [w, x, y, z]
    Output:
        q_inv: Inverted quaternion [w, -x, -y, -z]
    """
    quat = np.array(quat)
    # Ensure the quaternion is in the form [w, x, y, z] and invert the vector part
    q_inv = quat * np.array([1, -1, -1, -1])
    return q_inv

def quatmultiply(q1, q2):
    """
    Multiply two quaternions.
    Input:
        q1: First quaternion as a 1D array or list [w1, x1, y1, z1]
        q2: Second quaternion as a 1D array or list [w2, x2, y2, z2]
    Output:
        qm: Result of quaternion multiplication [w, x, y, z]
    """
    # Extract quaternion components for q1
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    # Extract quaternion components for q2
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
    
    # Quaternion multiplication formula
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    
    # Return the resulting quaternion in [w, x, y, z] format
    qm = np.array([w, x, y, z])
    return qm

def quat_to_rot_matrix(q):
    """
    将四元数转换为旋转矩阵
    输入:
        q: 四元数列表或数组 [w, x, y, z]
    输出:
        R: 3x3旋转矩阵
    """
    # 确保四元数是单位四元数（归一化）
    q = np.array(q)
    q = q / np.linalg.norm(q)
    
    # 提取四元数分量
    w, x, y, z = q
    
    # 计算旋转矩阵
    R = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
    ])
    
    return R

def rot_matrix_to_quat(R):
    """
    将旋转矩阵转换为四元数
    输入:
        R: 3x3旋转矩阵
    输出:
        q: 四元数 [w, x, y, z]
    """
    # 计算四元数的分量
    w = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
    x = (R[2, 1] - R[1, 2]) / (4 * w)
    y = (R[0, 2] - R[2, 0]) / (4 * w)
    z = (R[1, 0] - R[0, 1]) / (4 * w)
    
    # 组合成四元数
    q = np.array([w, x, y, z])
    return q

def rot_matrix_to_euler_ZYX(R):
    """
    将旋转矩阵转换为ZYX顺序的欧拉角
    输入:
        R: 3x3旋转矩阵
    输出:
        euler_angles: [yaw, pitch, roll] (ZYX顺序)
    """
    # 检查特殊情况
    if R[2, 0] < 1:
        if R[2, 0] > -1:
            yaw = np.arctan2(R[1, 0], R[0, 0])
            pitch = np.arcsin(-R[2, 0])
            roll = np.arctan2(R[2, 1], R[2, 2])
        else:
            # R[2, 0] == -1, 特殊情况
            yaw = -np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.pi / 2
            roll = 0
    else:
        # R[2, 0] == 1, 特殊情况
        yaw = np.arctan2(-R[1, 2], R[1, 1])
        pitch = -np.pi / 2
        roll = 0

    euler_angles = [yaw, pitch, roll]
    return euler_angles

def rot_matrix_to_euler_XYZ(R):
    """
    将旋转矩阵转换为XYZ顺序的欧拉角
    输入:
        R: 3x3旋转矩阵
    输出:
        euler_angles: [roll, pitch, yaw] (XYZ顺序)
    """
    # 检查特殊情况
    if R[2, 2] < 1:
        if R[2, 2] > -1:
            roll = np.arctan2(-R[1, 2], R[2, 2])
            pitch = np.arcsin(R[0, 2])
            yaw = np.arctan2(-R[0, 1], R[0, 0])
        else:
            # R[2, 2] == -1, 特殊情况
            roll = -np.arctan2(R[0, 1], R[0, 0])
            pitch = -np.pi / 2
            yaw = 0
    else:
        # R[2, 2] == 1, 特殊情况
        roll = np.arctan2(R[0, 1], R[0, 0])
        pitch = np.pi / 2
        yaw = 0

    euler_angles = [roll, pitch, yaw]
    return euler_angles

def euler_to_rot_matrix_ZYX(eul):
    """
    Convert ZYX order Euler angles to a rotation matrix
    Input:
        eul: [yaw, pitch, roll] (Euler angles in ZYX order)
    Output:
        R: 3x3 rotation matrix
    """
    yaw, pitch, roll = eul[0], eul[1], eul[2]

    # Compute individual rotation matrices
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0,           1]
    ])
    
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0,             1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    Rx = np.array([
        [1, 0,            0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])

    # Combine rotation matrices
    R = Rz @ Ry @ Rx
    return R

def euler_to_rot_matrix_XYZ(eul):
    """
    Convert XYZ order Euler angles to a rotation matrix
    Input:
        eul: [roll, pitch, yaw] (Euler angles in XYZ order)
    Output:
        R: 3x3 rotation matrix
    """
    roll, pitch, yaw = eul[0], eul[1], eul[2]

    # Compute individual rotation matrices
    Rx = np.array([
        [1, 0,           0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])
    
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0,             1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0,           1]
    ])

    # Combine rotation matrices
    R = Rx @ Ry @ Rz
    return R

# quat = np.array([0.886887541149496, 0.306290279753173, 0.339564746635491, 0.065669907300832])
# print(quat)
# quat = normalize_quaternion(quat)
# print(quat)
# rotm = quat_to_rot_matrix(quat)
# print(rotm)
# quat1 = rot_matrix_to_quat(rotm)
# print(quat1)
# eul_zyx = np.rad2deg(rot_matrix_to_euler_ZYX(rotm))
# print(eul_zyx)
# eul_xyz = np.rad2deg(rot_matrix_to_euler_XYZ(rotm))
# print(eul_xyz)

eul = np.deg2rad([30.2, 53.4, 45.6])
print(eul)
R = euler_to_rot_matrix_ZYX(eul)
print(R)
R = euler_to_rot_matrix_XYZ(eul)
print(R)