import numpy as np

def FK_L2S_jones(L, d_):
    """
    Compute the forward kinematics using the FK_L2S_jones method.
    
    Parameters:
    L : list or array-like
        Input lengths [l1, l2, l3].
    d_ : float
        Parameter d_ used in the computation.
    
    Returns:
    out1 : list
        Outputs [s, k, phi].
    """
    l_1 = L[0]
    l_2 = L[1]
    l_3 = L[2]
    
    # Compute the outputs
    s = (l_1 / 3.0) + (l_2 / 3.0) + (l_3 / 3.0)
    k = (2.0 * np.sqrt(-l_1 * l_2 - l_1 * l_3 - l_2 * l_3 + l_1**2 + l_2**2 + l_3**2)) / (d_ * (l_1 + l_2 + l_3))
    phi = np.arctan2(np.sqrt(3.0) * (-2.0 * l_1 + l_2 + l_3), 3.0 * l_2 - 3.0 * l_3)
    
    # Combine outputs into a list
    out1 = [s, k, phi]
    
    return out1

def FK_L2S_cosimo_old(L, d_):
    """
    Compute the forward kinematics using the FK_L2S_cosimo_old method.
    
    Parameters:
    in1 : numpy.ndarray
        Input array of shape (n, 3), where each row contains [l1, l2, l3].
    d_ : float
        Parameter d_ used in the computation.
    
    Returns:
    out1 : numpy.ndarray
        Outputs array of shape (n, 3), where each row contains [s, phi, theta].
    """
    # Extract lengths from input array
    l_1 = L[0]
    l_2 = L[1]
    l_3 = L[2]
    
    # Compute intermediate terms
    t2 = l_1 / 3.0
    t3 = l_2 / 3.0
    t4 = l_3 / 3.0
    t5 = t2 + t3 + t4
    
    # Compute outputs
    s = t5
    phi = np.arctan2(np.sqrt(3.0) * (-2.0 * l_1 + l_2 + l_3), 3.0 * l_2 - 3.0 * l_3)
    theta = (t5 * np.sqrt(-l_1 * l_2 - l_1 * l_3 - l_2 * l_3 + l_1**2 + l_2**2 + l_3**2) * 2.0) / (d_ * (l_1 + l_2 + l_3))
    
    # Combine results into a single array
    out1 = np.column_stack((s, phi, theta))
    
    return out1

def FK_L2S_cosimo_new(L, d):
    """
    Inputs: L = [l1, l2, l3], d

    Outputs: [s, deltax, deltay]

    This function is a Python translation of the MATLAB function FK_L2S_cosimo_new.
    """
    # 输入分量
    l1 = L[0]
    l2 = L[1]
    l3 = L[2]

    # 计算中间变量
    t2 = np.imag(l2)
    t3 = np.imag(l3)
    t4 = np.real(l2)
    t5 = np.real(l3)
    t6 = l1 * l2
    t7 = l1 * l3
    t8 = l2 * l3
    t9 = l1 * 2.0
    t10 = l2 * 3.0
    t11 = l3 * 3.0
    t12 = l1 ** 2
    t13 = l2 ** 2
    t14 = l3 ** 2
    t15 = l1 + l2 + l3
    t16 = 1.0 / d
    t22 = np.sqrt(3.0)
    t23 = l1 / 3.0
    t24 = l2 / 3.0
    t25 = l3 / 3.0
    t17 = -t9
    t18 = -t11
    t19 = -t6
    t20 = -t7
    t21 = -t8
    t27 = 1.0 / t15
    t28 = t23 + t24 + t25
    t26 = l2 + l3 + t17
    t32 = t12 + t13 + t14 + t19 + t20 + t21
    t29 = t22 * t26 * 1j
    t34 = np.sqrt(t32)
    t30 = t10 + t18 + t29
    t31 = np.abs(t30)
    t33 = 1.0 / t31

    # 输出计算
    s = t28
    deltax = t16 * t27 * t28 * t33 * t34 * (-3.0 * t4 + 3.0 * t5 + t22 * (t2 + t3 - 2.0 * np.imag(l1))) * -2.0
    deltay = t16 * t27 * t28 * t33 * t34 * (3.0 * t2 - 3.0 * t3 + t22 * (t4 + t5 - 2.0 * np.real(l1))) * 2.0

    # 转置输出
    out1 = np.transpose([s, deltax, deltay])
    return out1

def FK_L2S_new_3sections(in1, d):
    """
    Compute forward kinematics for three sections using the FK_L2S_new_3sections method.

    Parameters:
    in1 : numpy.ndarray
        Input array of shape (n, 9), where each row contains [l1, l2, ..., l9].
    d : float
        A parameter used in the computation.

    Returns:
    out1 : numpy.ndarray
        Outputs array with results.
    """
    # Extract real and imaginary parts of input
    l1, l2, l3, l4, l5, l6, l7, l8, l9 = [in1[i] for i in range(9)]
    sdxdy1 = FK_L2S_cosimo_new([l1, l2, l3], d)
    sdxdy2 = FK_L2S_cosimo_new([l4, l5, l6], d)
    sdxdy3 = FK_L2S_cosimo_new([l7, l8, l9], d)

    sdxdy = [sdxdy1[0], sdxdy1[1], sdxdy1[2], sdxdy2[0], sdxdy2[1], sdxdy2[2], sdxdy3[0], sdxdy3[1], sdxdy3[2]]
    sdxdy = np.transpose(sdxdy)

    return sdxdy