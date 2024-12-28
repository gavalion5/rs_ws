# import numpy as np
# from scipy.spatial.transform import Rotation as R

# def dh_transform(a, alpha, d, theta):
#     """Compute the transformation matrix using DH parameters."""
#     ct = np.cos(theta)
#     st = np.sin(theta)
#     ca = np.cos(alpha)
#     sa = np.sin(alpha)
#     return np.array([
#         [ct, -st * ca,  st * sa, a * ct],
#         [st,  ct * ca, -ct * sa, a * st],
#         [0,       sa,      ca,      d],
#         [0,        0,       0,      1]
#     ])

# def forward_kinematics(joint_angles, dh_params):
#     """
#     Compute the forward kinematics of the robot.
#     :param joint_angles: List of joint angles (theta1, theta2, ..., theta6)
#     :param dh_params: List of DH parameters [a, alpha, d, theta_offset]
#     :return: 4x4 transformation matrix of the end-effector
#     """
#     T = np.eye(4)  # Start with identity matrix
#     for i, (a, alpha, d, theta_offset) in enumerate(dh_params):
#         theta = joint_angles[i] + theta_offset
#         T = np.dot(T, dh_transform(a, alpha, d, theta))
#         print(T)
#     return T
# def compute_jacobian(joint_angles, dh_params):
#     """
#     Compute the Jacobian for the robot.
#     :param joint_angles: List of joint angles
#     :param dh_params: List of DH parameters
#     :return: 6xN Jacobian matrix
#     """
#     n = len(joint_angles)
#     T = np.eye(4)
#     z = np.array([0, 0, 1])
#     o = np.array([0, 0, 0])
#     J = np.zeros((6, n))

#     transformations = []  # Store intermediate transforms
#     for i, (a, alpha, d, theta_offset) in enumerate(dh_params):
#         theta = joint_angles[i] + theta_offset
#         T = np.dot(T, dh_transform(a, alpha, d, theta))
#         transformations.append(T)

#     o_end = T[:3, 3]  # End-effector position

#     for i in range(n):
#         T_prev = transformations[i - 1] if i > 0 else np.eye(4)
#         z_i = T_prev[:3, 2]
#         o_i = T_prev[:3, 3]

#         # Linear velocity
#         J[:3, i] = np.cross(z_i, (o_end - o_i))

#         # Angular velocity
#         J[3:, i] = z_i

#     return J
# def numerical_ik(target_pose, initial_guess, dh_params, max_iters=1000, tol=1e-6):
#     """
#     Numerical IK solver using Jacobian pseudoinverse.
#     :param target_pose: Desired 4x4 end-effector pose (position + orientation)
#     :param initial_guess: Initial guess for joint angles
#     :param dh_params: DH parameters
#     :param max_iters: Maximum number of iterations
#     :param tol: Convergence tolerance
#     :return: Joint angles that achieve the desired pose
#     """
#     joint_angles = np.array(initial_guess)

#     for _ in range(max_iters):
#         # Compute current end-effector pose
#         current_pose = forward_kinematics(joint_angles, dh_params)
        
#         # Compute position and orientation error
#         position_error = target_pose[:3, 3] - current_pose[:3, 3]
#         R_current = current_pose[:3, :3]
#         R_target = target_pose[:3, :3]
#         orientation_error = 0.5 * (np.cross(R_current[:, 0], R_target[:, 0]) +
#                                    np.cross(R_current[:, 1], R_target[:, 1]) +
#                                    np.cross(R_current[:, 2], R_target[:, 2]))

#         # Combine position and orientation error
#         error = np.concatenate((position_error, orientation_error))
#         if np.linalg.norm(error) < tol:
#             break

#         # Compute Jacobian and update joint angles
#         J = compute_jacobian(joint_angles, dh_params)
#         J_pseudo = np.linalg.pinv(J)
#         delta_theta = np.dot(J_pseudo, error)
#         joint_angles += delta_theta
#         print(joint_angles)
#         print(error)

#     return joint_angles

# # Define DH parameters for UR5
# dh_params = [
#     [0.0, np.pi/2, 0.08946, 0.0],
#     [-0.425, 0.0, 0.0, 0],
#     [-0.3922, 0.0, 0.0, 0.0],
#     [0.0, np.pi/2, 0.1091, 0.0],
#     [0.0, -np.pi/2, 0.09465, 0.0],
#     [0.0, 0.0, 0.0823, 0.0]
# ]

# import numpy as np

# def rpy_to_matrix(roll, pitch, yaw, translation=(0, 0, 0)):
#     """
#     Converts Roll-Pitch-Yaw (RPY) angles to a 4x4 transformation matrix.

#     Args:
#         roll (float): Roll angle in radians.
#         pitch (float): Pitch angle in radians.
#         yaw (float): Yaw angle in radians.
#         translation (tuple): Translation (x, y, z) as a tuple. Default is (0, 0, 0).

#     Returns:
#         np.ndarray: A 4x4 homogeneous transformation matrix.
#     """
#     # Compute individual rotation matrices
#     cr = np.cos(roll)
#     sr = np.sin(roll)
#     cp = np.cos(pitch)
#     sp = np.sin(pitch)
#     cy = np.cos(yaw)
#     sy = np.sin(yaw)

#     # Rotation matrix for Roll (X-axis)
#     R_x = np.array([
#         [1, 0, 0],
#         [0, cr, -sr],
#         [0, sr, cr]
#     ])

#     # Rotation matrix for Pitch (Y-axis)
#     R_y = np.array([
#         [cp, 0, sp],
#         [0, 1, 0],
#         [-sp, 0, cp]
#     ])

#     # Rotation matrix for Yaw (Z-axis)
#     R_z = np.array([
#         [cy, -sy, 0],
#         [sy, cy, 0],
#         [0, 0, 1]
#     ])

#     # Combined rotation matrix
#     R = R_z @ R_y @ R_x  # Note the order: Z * Y * X

#     # Create the 4x4 transformation matrix
#     T = np.eye(4)  # Start with an identity matrix
#     T[:3, :3] = R  # Set the top-left 3x3 submatrix to the rotation matrix
#     T[:3, 3] = translation  # Set the top-right 3x1 submatrix to the translation vector

#     return T

# # Example usage
# roll = np.pi/2   # in radians
# pitch = 0  # in radians
# yaw = np.pi/2    # in radians
# translation = (0.0953, 0.409-0.3, 0.956-0.05)  # Translation vector

# target_pose = rpy_to_matrix(roll, pitch, yaw, translation)
# print("4x4 Transformation Matrix:")
# print(target_pose)

# # Desired target pose (4x4 transformation matrix)
# # Initial guess for joint angles
# initial_guess = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# # Solve IK
# solution = numerical_ik(target_pose, initial_guess, dh_params)
# print("Joint Angles:", solution)

# # Verify using FK
# end_effector_pose = forward_kinematics(solution, dh_params)
# print("End Effector Pose:")
# print(end_effector_pose)

# solution = np.array([0,-np.pi/2,0,0,np.pi/2,0])
# end_effector_pose = forward_kinematics(initial_guess, dh_params)
# print("End Effector Pose:")
# world_to_robot = np.array([
#     [-1,0,0,0],
#     [0,-1,0,0],
#     [0,0,1,0],
#     [0,0,0,1]
# ])

# print(np.dot(world_to_robot, end_effector_pose))



# # 11
# # 1
# # # 1
# # # 1

import numpy as np
from pytransform3d import rotations as pr

def forwardKinematics(theta,tcp=None):  
    # theta are the joint angles in radians
    # tcp is the tcp offset as a pose (x,y,z,rx,ry,rz)

    #values for UR-10e (https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/):
    a = np.array([0.0000,-0.425,-0.39225,0.0000,0.0000,0.0000])
    d = np.array([0.089159,0.0000,0.0000,0.10915,0.09465,0.0823])
    alpha = np.array([np.pi/2,0.,0.,np.pi/2,-np.pi/2,0.])
    
    #values from calibration.conf:
    # delta_a = np.array([ 3.1576640107943976e-05, 0.298634925475782076, 0.227031257526500829, -8.27068507303316573e-05, 3.6195435783833642e-05, 0])
    # delta_d = np.array([ 5.82932048768247668e-05, 362.998939868892023, -614.839459588742898, 251.84113332747981, 0.000164511802564715204, -0.000899906496469232708])
    # delta_alpha = np.array([ -0.000774756642435869836, 0.00144883356002286951, -0.00181081418698111852, 0.00068792563586761446, 0.000450856239573305118, 0])
    # delta_theta = np.array([ 1.09391516130152855e-07, 1.03245736607748673, 6.17452995676434124, -0.92380698472218048, 6.42771759845617296e-07, -3.18941184192234051e-08])

    # a += delta_a
    # d += delta_d
    # alpha+=delta_alpha
    # theta=theta.copy()+delta_theta

    ot = np.eye(4)
    for i in range(6):
        ot = ot @ np.array([[np.cos(theta[i]), -(np.sin(theta[i]))*np.cos(alpha[i]), np.sin(theta[i])*np.sin(alpha[i]), a[i]*np.cos(theta[i])],[np.sin(theta[i]),np.cos(theta[i])*np.cos(alpha[i]),-(np.cos(theta[i]))*np.sin(alpha[i]),a[i]*np.sin(theta[i])], [0.0,np.sin(alpha[i]),np.cos(alpha[i]),d[i]],[0.0,0.0,0.0,1.0]])
    world_to_robot = np.array([
        [-1,0,0,0],
        [0,-1,0,0],
        [0,0,1,0],
        [0,0,0,1]
    ])
    ot = world_to_robot @ ot
    if not tcp is None:
        offset = np.array([ot[0,3],ot[1,3],ot[2,3]])  + (ot[:3,:3] @ tcp[:3])
        newAngle = pr.compact_axis_angle_from_matrix( ot[:3,:3] @ pr.matrix_from_compact_axis_angle(tcp[3:]))
        return np.array([offset[0],offset[1],offset[2],newAngle[0],newAngle[1],newAngle[2]])
    else:
        axisAngle = pr.compact_axis_angle_from_matrix(ot[:3,:3])
        return np.array([ot[0,3],ot[1,3],ot[2,3],axisAngle[0],axisAngle[1],axisAngle[2]])
    
# world_to_robot = np.array([
#     [-1,0,0,0],
#     [0,-1,0,0],
#     [0,0,1,0],
#     [0,0,0,1]
# ])
# end_effector_pose = forwardKinematics([0,-1.57,0,0,1.57,0],[0,0,0,0,0,0])
# print(np.dot(world_to_robot, end_effector_pose))
print(forwardKinematics([0,-1.57,0,-1.57,0,0],[0,0,0,0,0,0]))