import numpy as np

def normalize_quaternion(q):
    """
    Normalize a quaternion to make its length 1
    q: quaternion [w, x, y, z] or a 2D array of shape (n, 4)
    Returns the normalized quaternion
    """
    q = np.array(q)
    if q.ndim == 1:  # Single quaternion case
        norm = np.linalg.norm(q)
        if norm == 0:
            raise ValueError("Zero vector cannot be normalized as a quaternion")
        return q / norm
    elif q.ndim == 2:  # Batch processing of multiple quaternions
        norms = np.linalg.norm(q, axis=1, keepdims=True)
        norms[norms == 0] = 1  # Prevent division by zero
        return q / norms
    else:
        raise ValueError("Input should be a 1D or 2D array")

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
    
def quat2rotm(q):
    """
    Convert a quaternion to a rotation matrix
    Input:
        q: Quaternion list or array [w, x, y, z]
    Output:
        R: 3x3 rotation matrix
    """
    # Ensure the quaternion is a unit quaternion (normalize)
    q = np.array(q)
    q = q / np.linalg.norm(q)
    
    # Extract quaternion components
    w, x, y, z = q
    
    # Calculate the rotation matrix
    R = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
    ])
    
    return R

def rotm2quat(R):
    """
    Convert a rotation matrix to a quaternion
    Input:
        R: 3x3 rotation matrix
    Output:
        q: Quaternion [w, x, y, z]
    """
    # Calculate quaternion components
    w = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
    x = (R[2, 1] - R[1, 2]) / (4 * w)
    y = (R[0, 2] - R[2, 0]) / (4 * w)
    z = (R[1, 0] - R[0, 1]) / (4 * w)
    
    # Combine components into a quaternion
    q = np.array([w, x, y, z])
    return q

def rotm2eul(R, order='ZYX'):
    """
    Convert a rotation matrix to Euler angles in specified order
    Input:
        R: 3x3 rotation matrix
        order: Rotation order, either 'XYZ' or 'ZYX'
    Output:
        euler_angles: Euler angles [roll, pitch, yaw] for XYZ order 
                      or [yaw, pitch, roll] for ZYX order
    """
    if order == 'XYZ':
        # XYZ rotation order
        if R[2, 2] < 1:
            if R[2, 2] > -1:
                roll = np.arctan2(-R[1, 2], R[2, 2])
                pitch = np.arcsin(R[0, 2])
                yaw = np.arctan2(-R[0, 1], R[0, 0])
            else:
                # R[2, 2] == -1, special case
                roll = -np.arctan2(R[0, 1], R[0, 0])
                pitch = -np.pi / 2
                yaw = 0
        else:
            # R[2, 2] == 1, special case
            roll = np.arctan2(R[0, 1], R[0, 0])
            pitch = np.pi / 2
            yaw = 0

        euler_angles = [roll, pitch, yaw]
    
    elif order == 'ZYX':
        # ZYX rotation order
        if R[2, 0] < 1:
            if R[2, 0] > -1:
                yaw = np.arctan2(R[1, 0], R[0, 0])
                pitch = np.arcsin(-R[2, 0])
                roll = np.arctan2(R[2, 1], R[2, 2])
            else:
                # R[2, 0] == -1, special case
                yaw = -np.arctan2(-R[1, 2], R[1, 1])
                pitch = np.pi / 2
                roll = 0
        else:
            # R[2, 0] == 1, special case
            yaw = np.arctan2(-R[1, 2], R[1, 1])
            pitch = -np.pi / 2
            roll = 0

        euler_angles = [yaw, pitch, roll]
    
    else:
        raise ValueError("Invalid rotation order. Supported orders are 'XYZ' and 'ZYX'.")

    return euler_angles

def eul2rotm(eul, order='ZYX'):
    """
    Convert Euler angles to a rotation matrix
    Input:
        eul: [yaw, pitch, roll] (Euler angles in ZYX order) or
             [roll, pitch, yaw] (Euler angles in XYZ order)
        order: Rotation order, either 'ZYX' or 'XYZ'
    Output:
        R: 3x3 rotation matrix
    """
    if order == 'XYZ':
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
    
    elif order == 'ZYX':
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
    
    else:
        raise ValueError("Invalid rotation order. Supported orders are 'XYZ' and 'ZYX'.")

    return R

def quat2eul(quat, order='ZYX'):
    """
    Convert a quaternion to Euler angles in specified order
    Input:
        quat: 1x4 quaternion
        order: Rotation order, either 'XYZ' or 'ZYX'
    Output:
        euler_angles: Euler angles [roll, pitch, yaw] for XYZ order 
                      or [yaw, pitch, roll] for ZYX order
    """
    if order == 'ZYX' or order == 'XYZ':
        rotm = quat2rotm(quat)
        euler_angles = rotm2eul(rotm, order)
    else:
        raise ValueError("Invalid rotation order. Supported orders are 'XYZ' and 'ZYX'.")

    return euler_angles

def eul2quat(eul, order='ZYX'):
    """
    Convert Euler angles to a quaternion
    Input:
        eul: [yaw, pitch, roll] (Euler angles in ZYX order) or
             [roll, pitch, yaw] (Euler angles in XYZ order)
        order: Rotation order, either 'ZYX' or 'XYZ'
    Output:
        quat: 1x4 quaternion
    """
    if order == 'ZYX' or order == 'XYZ':
        rotm = eul2rotm(eul,order)
        quat = rotm2quat(rotm)
    else:
        raise ValueError("Invalid rotation order. Supported orders are 'XYZ' and 'ZYX'.")

    return quat