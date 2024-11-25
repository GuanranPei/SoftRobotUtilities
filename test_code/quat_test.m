clear 
clc

q1 = eul2quat(deg2rad([30.1, 20.4, 50.3]))
q2 = eul2quat(deg2rad([20.3, 30.4, 60.3]))

q1inv = quat_inv(q1)
q1inv1 = quatinv(q1)

q1q2 = quat_multiply(q1,q2)
q1q21 = quatmultiply(q1,q2)

% quat = eul2quat(deg2rad([23.1, 34.2, 45.3]))
% 
% quat = robotics.internal.normalizeRows(quat)
% 
% % eul = quaternion_to_euler(quat,'ZYX')
% % 
% % eul1 = quat2eul(quat)
% 
% rotm = quat_to_rot_matrix(quat)
% rotm1 = quat2rotm(quat)
% 
% eul = rad2deg(rot_matrix_to_euler_ZYX(rotm1))
% eul1 = rad2deg(rotm2eul(rotm1))
% 
% eul = rad2deg(rot_matrix_to_euler_XYZ(rotm1))
% eul1 = rad2deg(rotm2eul(rotm1,"XYZ"))

% eul = deg2rad([30.2, 53.4, 45.6])
% % R = euler_to_rot_matrix_ZYX(eul)
% R1 = eul2rotm(eul,"ZYX")
% % R = euler_to_rot_matrix_XYZ(eul)
% R1 = eul2rotm(eul,"XYZ")

function R = quat_to_rot_matrix(q)
    % 输入 q: 四元数 [w, x, y, z]
    % 输出 R: 3x3旋转矩阵
    
    % 确保四元数是单位四元数（归一化）
    q = q / norm(q);
    
    % 提取四元数分量
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);
    
    % 计算旋转矩阵
    R = [
        1 - 2*y^2 - 2*z^2, 2*x*y - 2*w*z, 2*x*z + 2*w*y;
        2*x*y + 2*w*z, 1 - 2*x^2 - 2*z^2, 2*y*z - 2*w*x;
        2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x^2 - 2*y^2
    ];
end

function q = rot_matrix_to_quat(R)
    % 输入 R: 3x3旋转矩阵
    % 输出 q: 四元数 [w, x, y, z]
    
    % 计算四元数的分量
    w = sqrt(1 + R(1,1) + R(2,2) + R(3,3)) / 2;
    x = (R(3,2) - R(2,3)) / (4 * w);
    y = (R(1,3) - R(3,1)) / (4 * w);
    z = (R(2,1) - R(1,2)) / (4 * w);
    
    % 组合成四元数
    q = [w, x, y, z];
end

function euler_angles = rot_matrix_to_euler_ZYX(R)
    % 输入 R: 3x3旋转矩阵
    % 输出 euler_angles: [yaw, pitch, roll] (ZYX顺序)
    
    if R(3,1) < 1
        if R(3,1) > -1
            yaw = atan2(R(2,1), R(1,1));
            pitch = asin(-R(3,1));
            roll = atan2(R(3,2), R(3,3));
        else
            % R(3,1) == -1, 特殊情况
            yaw = -atan2(-R(2,3), R(2,2));
            pitch = pi/2;
            roll = 0;
        end
    else
        % R(3,1) == 1, 特殊情况
        yaw = atan2(-R(2,3), R(2,2));
        pitch = -pi/2;
        roll = 0;
    end
    
    euler_angles = [yaw, pitch, roll];
end

function euler_angles = rot_matrix_to_euler_XYZ(R)
    % 输入 R: 3x3旋转矩阵
    % 输出 euler_angles: [roll, pitch, yaw] (XYZ顺序)
    
    if R(3,3) < 1
        if R(3,3) > -1
            roll = atan2(-R(2,3), R(3,3));
            pitch = asin(R(1,3));
            yaw = atan2(-R(1,2), R(1,1));
        else
            % R(3,3) == -1, 特殊情况
            roll = -atan2(R(1,2), R(1,1));
            pitch = -pi/2;
            yaw = 0;
        end
    else
        % R(3,3) == 1, 特殊情况
        roll = atan2(R(1,2), R(1,1));
        pitch = pi/2;
        yaw = 0;
    end
    
    euler_angles = [roll, pitch, yaw];
end

function R = euler_to_rot_matrix_ZYX(eul)
    % 输入: yaw, pitch, roll (ZYX顺序的欧拉角)
    % 输出: R - 3x3旋转矩阵

    yaw = eul(1); pitch = eul(2); roll = eul(3);

    % 计算各旋转矩阵
    Rz = [cos(yaw), -sin(yaw), 0;
          sin(yaw),  cos(yaw), 0;
          0,         0,        1];
      
    Ry = [cos(pitch), 0, sin(pitch);
          0,          1, 0;
         -sin(pitch), 0, cos(pitch)];
     
    Rx = [1, 0,          0;
          0, cos(roll), -sin(roll);
          0, sin(roll),  cos(roll)];

    % 组合旋转矩阵
    R = Rz * Ry * Rx;
end

function R = euler_to_rot_matrix_XYZ(eul)
    % 输入: roll, pitch, yaw (XYZ顺序的欧拉角)
    % 输出: R - 3x3旋转矩阵
    yaw = eul(3); pitch = eul(2); roll = eul(1);

    % 计算各旋转矩阵
    Rx = [1, 0,          0;
          0, cos(roll), -sin(roll);
          0, sin(roll),  cos(roll)];
      
    Ry = [cos(pitch), 0, sin(pitch);
          0,          1, 0;
         -sin(pitch), 0, cos(pitch)];
     
    Rz = [cos(yaw), -sin(yaw), 0;
          sin(yaw),  cos(yaw), 0;
          0,         0,        1];

    % 组合旋转矩阵
    R = Rx * Ry * Rz;
end

function q_inv = quat_inv(quat)
    % Convert the quaternion to an array if necessary and negate the vector part
    quat = quat(:); % Ensure quat is a column vector
    q_inv = quat .* [1; -1; -1; -1];
end

function qm = quat_multiply(q1, q2)  
    % Extract quaternion components
    w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
    w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);
    
    % Quaternion multiplication formula
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
    
    % Return the quaternion in [w, x, y, z] format
    qm = [w; x; y; z];
end