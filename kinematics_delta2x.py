import numpy as np

def FK_S2X_jones(s, k, phi):
    """
    Compute the forward kinematics using the FK_S2X_jones method.
    
    Parameters:
    s : float
        Input parameter s.
    k : float
        Input parameter k.
    phi : float
        Input parameter phi (angle in radians).
    
    Returns:
    out1 : numpy.ndarray
        Output matrix of shape (3, 4).
    """
    # Compute trigonometric and intermediate terms
    t2 = np.cos(phi)
    t3 = np.sin(phi)
    t4 = k * s
    t8 = 1.0 / k
    t5 = t2**2
    t6 = np.cos(t4)
    t7 = np.sin(t4)
    t9 = t6 - 1.0
    t10 = t2 * t7
    t11 = t3 * t7
    t12 = t2 * t3 * t9
    t13 = t5 * t9

    # Compute the reshaped output matrix
    out1 = np.reshape(
    [
        t13 + 1.0, t12, t10, t12,
        t6 - t13, t11, -t10, -t11,
        t6, t2 * t8 * t9, t3 * t8 * t9, t7 * t8
    ],
    (3, 4),order='F'
    )
    
    return out1

def FK_S2X_cosimo_old(s, phi, theta):
    """
    Compute the forward kinematics using the FK_S2X_cosimo_old method.
    
    Parameters:
    s : float
        Input parameter s.
    phi : float
        Input parameter phi (angle in radians).
    theta : float
        Input parameter theta (angle in radians).
    
    Returns:
    out1 : numpy.ndarray
        Output matrix of shape (3, 4).
    """
    # Compute trigonometric terms
    t2 = np.cos(phi)
    t3 = np.cos(theta)
    t4 = np.sin(phi)
    t5 = np.sin(theta)
    t6 = 1.0 / theta
    t7 = t2 * t5
    t8 = t4 * t5
    t9 = t3 - 1.0
    t10 = t2 * t4 * t9
    
    # Compute the reshaped output matrix
    out1 = np.reshape(
    [
        t2**2 * t9 + 1.0, t10, -t7, t10,
        t4**2 * t9 + 1.0, -t8, t7, t8,
        t3, -s * t2 * t6 * t9, -s * t4 * t6 * t9, s * t5 * t6
    ],
    (3, 4),order='F'
    )
    return out1

def FK_S2X_cosimo_new(s, deltax, deltay):
    """
    Python translation of FK_S2X_cosimo_new.
    
    Args:
        s: Scalar input representing some parameter.
        deltax: Scalar input for displacement in the x direction.
        deltay: Scalar input for displacement in the y direction.
    
    Returns:
        out1: A 3x4 numpy array.
    """
    # 计算中间变量
    t2 = deltax ** 2
    t3 = deltay ** 2
    t4 = t2 + t3
    t5 = 1.0 / t4
    t6 = np.sqrt(t4)
    t7 = 1.0 / t6
    t8 = np.cos(t6)
    t9 = np.sin(t6)
    t10 = t8 - 1.0
    t11 = deltax * t7 * t9
    t12 = deltay * t7 * t9
    t13 = deltax * deltay * t5 * t10

    # 构造 reshape 的数据
    data = [
        t2 * t5 * t10 + 1.0, t13, t11, 
        t13, t3 * t5 * t10 + 1.0, t12, 
        -t11, -t12, t8, 
        deltax * s * t5 * t10, deltay * s * t5 * t10, s * t7 * t9
    ]

    # 按 MATLAB reshape 逻辑，转换为 3x4 矩阵
    out1 = np.reshape(data, (3, 4), order='F')  # MATLAB 的列优先模式为 'F'

    return out1

def FK_realrobot(S,Deltax,Deltay):
    deltax1 = Deltax[0]
    deltax2 = Deltax[1]
    deltay1 = Deltay[0]
    deltax3 = Deltax[2]
    deltay2 = Deltay[1]
    deltay3 = Deltay[2]
    s1 = S[0]
    s2 = S[1]
    s3 = S[2]

    t2 = deltax1 ** 2
    t3 = deltax2 ** 2
    t4 = deltay1 ** 2
    t5 = deltax3 ** 2
    t6 = deltay2 ** 2
    t7 = deltay3 ** 2
    t8 = np.sqrt(3.0)
    t9 = t2 + t4
    t10 = t3 + t6
    t11 = t5 + t7
    t12 = 1.0 / t9
    t13 = 1.0 / t10
    t14 = 1.0 / t11
    t15 = np.sqrt(t9)
    t16 = np.sqrt(t10)
    t17 = np.sqrt(t11)
    t18 = 1.0 / t15
    t19 = 1.0 / t16
    t20 = 1.0 / t17
    t21 = np.cos(t15)
    t22 = np.cos(t16)
    t23 = np.cos(t17)
    t24 = np.sin(t15)
    t25 = np.sin(t16)
    t26 = np.sin(t17)
    t27 = t21 - 1.0
    t28 = t22 - 1.0
    t29 = t23 - 1.0
    t30 = t21 * t22
    t31 = deltax1 * t18 * t24
    t32 = deltay1 * t18 * t24
    t33 = s1 * t18 * t24
    t62 = deltax2 * t19 * t21 * t25
    t63 = deltay2 * t19 * t21 * t25
    t64 = s2 * t19 * t21 * t25
    t34 = deltax1 * deltay1 * t12 * t27
    t35 = deltax1 * s1 * t12 * t27
    t36 = deltay1 * s1 * t12 * t27
    t37 = t2 * t12 * t27
    t38 = t4 * t12 * t27
    t39 = t3 * t13 * t28
    t40 = t6 * t13 * t28
    t41 = t5 * t14 * t29
    t42 = t7 * t14 * t29
    t43 = t31 / 2.0
    t44 = t32 / 2.0
    t57 = t8 * t31 * (-1.0 / 2.0)
    t60 = t22 * t31
    t61 = t22 * t32
    t67 = t62 / 2.0
    t68 = t63 / 2.0
    t69 = deltax2 * t19 * t25 * t31
    t70 = deltay2 * t19 * t25 * t31
    t71 = deltax2 * t19 * t25 * t32
    t72 = deltay2 * t19 * t25 * t32
    t73 = s2 * t19 * t25 * t31
    t74 = s2 * t19 * t25 * t32
    t45 = t34 / 2.0
    t46 = t37 + 1.0
    t47 = t38 + 1.0
    t48 = t39 + 1.0
    t49 = t40 + 1.0
    t50 = t41 + 1.0
    t51 = t42 + 1.0
    t52 = t37 / 2.0
    t53 = t38 / 2.0
    t56 = t8 * t44
    t59 = t8 * t34 * (-1.0 / 2.0)
    t77 = -t73
    t78 = -t74
    t79 = deltax2 * t19 * t25 * t43
    t81 = deltax2 * t19 * t25 * t44
    t84 = t72 * (-1.0 / 2.0)
    t86 = t44 + t57
    t58 = t8 * t45
    t65 = (t8 * t46) / 2.0
    t66 = (t8 * t47) / 2.0
    t85 = t43 + t56
    t90 = t53 + t59 + 1.0 / 2.0
    t92 = deltax2 * t19 * t25 * t86
    t96 = deltax2 * deltay2 * t13 * t28 * t86
    t97 = deltax2 * s2 * t13 * t28 * t86
    t103 = t48 * t86
    t87 = t45 + t66
    t88 = t52 + t58 + 1.0 / 2.0
    t91 = deltay2 * t19 * t25 * t85
    t94 = deltax2 * deltay2 * t13 * t28 * t85
    t95 = deltay2 * s2 * t13 * t28 * t85
    t98 = -t96
    t100 = -t97
    t101 = t49 * t85
    t102 = t96 / 2.0
    t107 = -deltax2 * t19 * t25 * (t45 - t65)
    t108 = deltax2 * t19 * t25 * t90
    t109 = -t103
    t113 = t103 / 2.0
    t116 = deltax2 * deltay2 * t13 * t28 * t90
    t117 = deltax2 * s2 * t13 * t28 * t90
    t121 = -deltax2 * s2 * t13 * t28 * (t45 - t65)
    t129 = deltax2 * deltay2 * t13 * t28 * (t45 - t65) * (-1.0 / 2.0)
    t133 = t48 * t90
    t134 = (deltax2 * deltay2 * t13 * t28 * (t45 - t65)) / 2.0
    t93 = -t91
    t99 = t94 / 2.0
    t104 = -t102
    t105 = deltay2 * t19 * t25 * t87
    t106 = deltay2 * t19 * t25 * t88
    t110 = t101 / 2.0
    t111 = deltax2 * deltay2 * t13 * t28 * t88
    t112 = deltay2 * s2 * t13 * t28 * t88
    t114 = deltax2 * deltay2 * t13 * t28 * t87
    t115 = deltay2 * s2 * t13 * t28 * t87
    t118 = -t108
    t119 = -t113
    t124 = -t117
    t126 = t116 / 2.0
    t127 = t49 * t87
    t128 = t49 * t88
    t139 = t133 / 2.0
    t142 = t62 + t94 + t109
    t143 = t63 + t98 + t101
    t122 = -t114
    t123 = t111 / 2.0
    t125 = t114 / 2.0
    t131 = -t126
    t135 = -t127
    t136 = t127 / 2.0
    t137 = t128 / 2.0
    t141 = t30 + t92 + t93
    t144 = t60 + t106 + t107
    t145 = t61 + t105 + t118
    t146 = (t8 * t142) / 2.0
    t147 = (t8 * t143) / 2.0
    t153 = t8 * (t69 - t111 + t48 * (t45 - t65)) * (-1.0 / 2.0)
    t154 = t8 * (t70 - t128 + deltax2 * deltay2 * t13 * t28 * (t45 - t65)) * (-1.0 / 2.0)
    t130 = -t125
    t140 = -t137
    t148 = -t146
    t151 = t71 + t122 + t133
    t152 = t72 + t116 + t135
    t157 = t67 + t99 + t119 + t147
    t155 = (t8 * t151) / 2.0
    t156 = (t8 * t152) / 2.0
    t158 = t68 + t104 + t110 + t148
    t160 = t81 + t130 + t139 + t156

    # mt1, mt2, mt3, mt4, mt5, mt6 are arrays created as per the MATLAB code
    mt1 = [
        t46, t34, t31, 0.0,
        t70 * (-1.0 / 2.0) + t129 + t137 + t153,
        t84 + t131 + t136 - t155,
        t68 + t104 + t110 + t146, 0.0,
        t8 * (t51 * (t79 - t123 + (t8 * (t70 - t128 + deltax2 * deltay2 * t13 * t28 * (t45 - t65))) / 2.0 +
                    (t48 * (t45 - t65)) / 2.0) +
            deltay3 * t20 * t26 * t144 -
            deltax3 * deltay3 * t14 * t29 * (t70 / 2.0 + t134 + t140 + t153)) * (-1.0 / 2.0) +
        (t50 * (t70 / 2.0 + t134 + t140 + t153)) / 2.0 -
        (deltax3 * t20 * t26 * t144) / 2.0 -
        (deltax3 * deltay3 * t14 * t29 * (t79 - t123 + (t8 * (t70 - t128 + deltax2 * deltay2 * t13 * t28 * (t45 - t65))) / 2.0 +
                                        (t48 * (t45 - t65)) / 2.0)) / 2.0
    ]

    mt2 = [
        (t50 * (t72 / 2.0 + t126 - t136 - t155)) / 2.0 -
        (t8 * (t51 * t160 + deltay3 * t20 * t26 * t145 -
            deltax3 * deltay3 * t14 * t29 * (t72 / 2.0 + t126 - t136 - t155))) / 2.0 -
        (deltax3 * t20 * t26 * t145) / 2.0 -
        (deltax3 * deltay3 * t14 * t29 * t160) / 2.0,
        (t8 * (t51 * t157 + deltay3 * t20 * t26 * t141 -
            deltax3 * deltay3 * t14 * t29 * t158)) / 2.0 -
        (t50 * t158) / 2.0 +
        (deltax3 * t20 * t26 * t141) / 2.0 +
        (deltax3 * deltay3 * t14 * t29 * t157) / 2.0,
        0.0, t34, t47, t32, 0.0,
        t79 - t123 + t154 + (t48 * (t45 - t65)) / 2.0,
        t81 + t130 + t139 - t156,
        -t67 - t99 + t113 + t147, 0.0
    ]

    mt3 = [
        (t8 * (-t50 * (t70 / 2.0 + t134 + t140 + t153) + deltax3 * t20 * t26 * t144 +
            deltax3 * deltay3 * t14 * t29 * (t79 - t123 +
                                            (t8 * (t70 - t128 + deltax2 * deltay2 * t13 * t28 * (t45 - t65))) / 2.0 +
                                            (t48 * (t45 - t65)) / 2.0))) / 2.0 -
        (t51 * (t79 - t123 +
                (t8 * (t70 - t128 + deltax2 * deltay2 * t13 * t28 * (t45 - t65))) / 2.0 +
                (t48 * (t45 - t65)) / 2.0)) / 2.0 -
        (deltay3 * t20 * t26 * t144) / 2.0 +
        (deltax3 * deltay3 * t14 * t29 * (t70 / 2.0 + t134 + t140 + t153)) / 2.0
    ]

    mt4 = [
        t51 * t160 * (-1.0 / 2.0) +
        (t8 * (-t50 * (t72 / 2.0 + t126 - t136 - t155) + deltax3 * t20 * t26 * t145 +
            deltax3 * deltay3 * t14 * t29 * t160)) / 2.0 -
        (deltay3 * t20 * t26 * t145) / 2.0 +
        (deltax3 * deltay3 * t14 * t29 * (t72 / 2.0 + t126 - t136 - t155)) / 2.0,
        t8 * (-t50 * t158 + deltax3 * t20 * t26 * t141 +
            deltax3 * deltay3 * t14 * t29 * t157) * (-1.0 / 2.0) +
        (t51 * t157) / 2.0 +
        (deltay3 * t20 * t26 * t141) / 2.0 -
        (deltax3 * deltay3 * t14 * t29 * t158) / 2.0,
        0.0, -t31, -t32, t21, 0.0,
        -t60 - t106 + deltax2 * t19 * t25 * (t45 - t65),
        -t61 - t105 + t108,
        t141, 0.0
    ]

    mt5 = [
        -t23 * t144 - deltax3 * t20 * t26 * (t70 / 2.0 + t134 + t140 + t153) +
        deltay3 * t20 * t26 * (t79 - t123 +
                            (t8 * (t70 - t128 + deltax2 * deltay2 * t13 * t28 * (t45 - t65))) / 2.0 +
                            (t48 * (t45 - t65)) / 2.0),
        -t23 * t145 + deltay3 * t20 * t26 * t160 -
        deltax3 * t20 * t26 * (t72 / 2.0 + t126 - t136 - t155),
        t23 * t141 + deltax3 * t20 * t26 * t158 -
        deltay3 * t20 * t26 * t157,
        0.0, t35, t36, t33, 1.0,
        t35 + t77 + t112 + t121,
        t36 + t78 + t115 + t124,
        t33 + t64 + t95 + t100, 1.0
    ]

    mt6 = [
        t35 + t77 + t112 + t121 - s3 * t20 * t26 * t144 -
        deltay3 * s3 * t14 * t29 * (t79 - t123 +
                                    (t8 * (t70 - t128 + deltax2 * deltay2 * t13 * t28 * (t45 - t65))) / 2.0 +
                                    (t48 * (t45 - t65)) / 2.0) +
        deltax3 * s3 * t14 * t29 * (t70 / 2.0 + t134 + t140 + t153),
        t36 + t78 + t115 + t124 - s3 * t20 * t26 * t145 +
        deltax3 * s3 * t14 * t29 * (t72 / 2.0 + t126 - t136 - t155) -
        deltay3 * s3 * t14 * t29 * t160,
        t33 + t64 + t95 + t100 + s3 * t20 * t26 * t141 -
        deltax3 * s3 * t14 * t29 * t158 +
        deltay3 * s3 * t14 * t29 * t157,
        1.0
    ]

    # Combine all mt arrays into T_full
    return np.reshape([*mt1, *mt2, *mt3, *mt4, *mt5, *mt6], (12, 4),order='F')