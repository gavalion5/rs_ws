#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64
import time
import roboticstoolbox as rtb
from math import cos,sin
import numpy as np


def dh_transform(a, alpha, d, theta):
    """Compute the transformation matrix using DH parameters."""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,       sa,      ca,      d],
        [0,        0,       0,      1]
    ])

def forward_kinematics(joint_angles, dh_params):
    """
    Compute the forward kinematics of the robot.
    :param joint_angles: List of joint angles (theta1, theta2, ..., theta6)
    :param dh_params: List of DH parameters [a, alpha, d, theta_offset]
    :return: 4x4 transformation matrix of the end-effector
    """
    T = np.eye(4)  # Start with identity matrix
    for i, (a, alpha, d, theta_offset) in enumerate(dh_params):
        theta = joint_angles[i] + theta_offset
        T = np.dot(T, dh_transform(a, alpha, d, theta))
        # print(T)
    return T

def position_jacobian(joint_angles, dh_params):
    """
    Compute the Jacobian for the robot.
    :param joint_angles: List of joint angles
    :param dh_params: List of DH parameters
    :return: 6xN Jacobian matrix
    """
    n = len(joint_angles)
    T = np.eye(4)
    z = np.array([0, 0, 1])
    o = np.array([0, 0, 0])
    J = np.zeros((3, n))

    transformations = []  # Store intermediate transforms
    for i, (a, alpha, d, theta_offset) in enumerate(dh_params):
        theta = joint_angles[i] + theta_offset
        T = np.dot(T, dh_transform(a, alpha, d, theta))
        transformations.append(T)

    o_end = T[:3, 3]  # wrist_3_link or ee

    for i in range(n):
        T_prev = transformations[i - 1] if i > 0 else np.eye(4)
        z_i = T_prev[:3, 2]
        o_i = T_prev[:3, 3]
        # Linear velocity
        J[:3, i] = np.cross(z_i, (o_end - o_i))
    return J

def rcm_jacobian(joint_angles,dh_params, dh_params_ee, p_trocar): #compute rcm jacobian and return the stacked/modified jacobian
    """
    Compute the Jacobian for the robot.
    :param joint_angles: List of joint angles
    :param dh_params: List of DH parameters
    :return: 6xN Jacobian matrix
    """
    J_wrist_3 = position_jacobian(joint_angles,dh_params)
    J_ee = position_jacobian(joint_angles,dh_params_with_ee)
    p_wrist_3 = forward_kinematics(joint_angles, dh_params)[:3,3]
    p_ee = forward_kinematics(joint_angles, dh_params_with_ee)[:3,3]
    n = len(joint_angles)
    depth = np.linalg.norm(np.dot((p_trocar - p_wrist_3),(p_ee-p_wrist_3)/ np.linalg.norm(p_ee-p_wrist_3)))
    p_rcm = p_wrist_3 + (p_ee- p_wrist_3)*depth
    T = np.eye(4)
    z = np.array([0, 0, 1])
    o = np.array([0, 0, 0])
    J_rcm = np.zeros((3, n+1))
    J = np.zeros((6, n+1))
    J_rcm[:3,:6] = J_wrist_3+ depth*(J_ee-J_wrist_3) 
    J_rcm[:3,6] = p_ee - p_wrist_3
    J[:3,:6] = J_ee
    J[3:,:] = J_rcm
    return [J,p_ee,p_rcm,depth]

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











# def numerical_ik(target_pose, initial_guess, dh_params, max_iters=1000, tol=1e-1):
    # """
    # Numerical IK solver using Jacobian pseudoinverse.
    # :param target_pose: Desired 4x4 end-effector pose (position + orientation)
    # :param initial_guess: Initial guess for joint angles
    # :param dh_params: DH parameters
    # :param max_iters: Maximum number of iterations
    # :param tol: Convergence tolerance
    # :return: Joint angles that achieve the desired pose
    # """
    # joint_angles = np.array(initial_guess)

    # for _ in range(max_iters):print(3, 3] - current_pose[:3, 3]
    #     R_current = current_pose[:3, :3]
    #     R_target = target_pose[:3, :3]
        
    #     orientation_error = 0.5 * (np.cross(R_current[:, 0], R_target[:, 0]) +
    #                                np.cross(R_current[:, 1], R_target[:, 1]) +
    #                                np.cross(R_current[:, 2], R_target[:, 2]))

    #     # Combine position and orientation error
    #     error = np.concatenate((position_error, orientation_error))
    #     if np.linalg.norm(error) < tol:
    #         break

    #     # Compute Jacobian and update joint angles
    #     J = compute_jacobian(joint_angles, dh_params)
    #     J_pseudo = np.linalg.pinv(J)
    #     delta_theta = np.dot(J_pseudo, error)
    #     joint_angles += learning_rate * delta_theta
    #     # print(joint_angles)
    #     print(delta_theta)
    # print(_)
    # return joint_angles

# def control_pos(joint_angles, p_goal, p_trocar,dh_params_ee,dh_params,w): # w is [0,0,0,0,0,0, lambda0 - lambda]
#     L = rcm_jacobian(joint_angles,dh_params, dh_params_ee, p_trocar)
#     J,p_ee,p_rcm = L[0], L[1], L[2]
#     J_ee = J[:3,:6]
#     J_rcm = J[3:,:]
#     damping_multiplier = 0.01
#     damping = np.eye(6) * damping_multiplier
#     J_dls = J.T @ (J@J.T + damping) # damped least square method
#     error_p = np.concatenate((p_goal - p_ee), (p_trocar - p_rcm))
#     #continue here
#     error_v = np.concatenate((v_desired - v_ee), (-v_rcm))
#     joint_velocity = J_dls @ gain_p @ error_p + J_dls @ gain_v @ error_v +(np.eye(7) - J_dls@J)@w #7*1 vector take the 7th for the next iteration 
#     return joint_velocity

def control_speed(joint_angles, p_goal, p_trocar,dh_params_ee,dh_params): # w is [0,0,0,0,0,0, lambda0 - lambda]
    global v_prev
    start_time = time.time()
    alpha = 0.2 #low - accurate and responsive; high - smoother
    L = rcm_jacobian(joint_angles,dh_params, dh_params_ee, p_trocar)
    J,p_ee,p_rcm, depth = L[0], L[1], L[2], L[3]
    p_wrist_3 = forward_kinematics(joint_angles, dh_params)
    J_ee = J[:3,:6]
    J_rcm = J[3:,:]
    damping_multiplier = 0.01
    damping = np.eye(6) * damping_multiplier
    J_dls = J.T @ np.linalg.pinv((J@J.T + damping)) # damped least square method, dont forget to inverse
    v_repulse = vpf_repulse_link(p_wrist_3, p_ee, p_obstacles, p_rcm)
    v_attract = vpf_attract(p_ee, p_goal)
    v_desired = v_repulse+v_attract
    p_desired = p_ee + v_desired * elapsed_time
    depth_desired = 0.3 - np.linalg.norm(p_desired - p_trocar)
    w = np.array([0,0,0,0,0,0, depth_desired - depth])
    gain_p = np.array([[0.3,0,0,0,0,0],[0,0.3,0,0,0,0],[0,0,0.15,0,0,0],[0,0,0,2.5,0,0],[0,0,0,0,2.5,0],[0,0,0,0,0,2.5]])
    error_p = np.zeros(6)
    error_p[:3] = p_desired - p_ee
    error_p[3:] = p_trocar - p_rcm
    print(v_attract == v_desired)
    print(v_attract)
    print(error_p)
    joint_velocity_command = J_dls @ gain_p @ error_p +(np.eye(7) - J_dls@J)@w
    # v_rcm = (p_trocar - p_rcm)/elapsed_time # time from timestamp msgs (subscriber)
    # v_combined = np.zeros(6)
    # v_combined[:3] = v_desired
    # v_combined[3:] = v_rcm
    # v_final = (1-alpha) * v_combined + alpha * v_prev # damping for smoother transition 
    # v_final = v_smooth - kdamp * v_prev
    # v_prev = v_final # we can change the v_prev as actual speed
    # joint_velocity_output = J_dls@v_final # +(np.eye(7) - J_dls@J)@w #uncomment for 
    # print(time.time() - start_time)
    print(joint_velocity_command)
    return joint_velocity_command

def rcm_jacobian1(joint_angles,dh_params, dh_params_ee, p_ee,p_wrist_3,depth): #compute rcm jacobian and return the stacked/modified jacobian
    """
    Compute the Jacobian for the robot.
    :param joint_angles: List of joint angles
    :param dh_params: List of DH parameters
    :return: 6xN Jacobian matrix
    """
    laparascope_length = 0.3
    J_wrist_3 = position_jacobian(joint_angles,dh_params)
    J_ee = position_jacobian(joint_angles,dh_params_ee)
    n = len(joint_angles)
    p_rcm = p_wrist_3 + (p_ee- p_wrist_3)*depth/laparascope_length
    # print(depth)
    # print(p_rcm)
    J_rcm = np.zeros((3, n+1))
    J = np.zeros((6, n+1))
    J_rcm[:3,:6] = J_wrist_3+ depth*(J_ee-J_wrist_3) 
    J_rcm[:3,6] = p_ee - p_wrist_3
    J[:3,:6] = J_ee
    J[3:,:] = J_rcm
    return [J,p_rcm]


def control_speed1(joint_angles, p_goal, p_trocar,dh_params_ee,dh_params,depth_velocity): # w is [0,0,0,0,0,0, lambda0 - lambda]
    global v_prev
    global depth
    elapsed_time = 0.01
    # start_time = time.time()
    alpha = 0.2 #low - accurate and responsive; high - smoother
    depth = depth + depth_velocity*elapsed_time
    p_wrist_3 = forward_kinematics(joint_angles, dh_params)[:3,3]
    p_ee = forward_kinematics(joint_angles, dh_params_with_ee)[:3,3]
    L = rcm_jacobian1(joint_angles,dh_params, dh_params_ee, p_ee, p_wrist_3,depth)
    J,p_rcm = L[0], L[1]
    
    # p_wrist_3 = forward_kinematics(joint_angles, dh_params)
    damping_multiplier = 0.01
    damping = np.eye(6) * damping_multiplier
    J_dls = np.linalg.pinv(J) # J.T @ np.linalg.pinv((J@J.T + damping)) # damped least square method, dont forget to inverse
    v_repulse = vpf_repulse_link(p_wrist_3, p_ee, p_obstacles, p_rcm)
    v_attract = vpf_attract(p_ee, p_goal)
    v_desired = v_repulse+v_attract
    p_desired = p_ee + v_desired * elapsed_time
    depth_desired = 0.3 - np.linalg.norm(p_desired - p_trocar)
    w = np.array([0,0,0,0,0,0, depth_desired - depth])
    # gain_p = np.array([[0.3,0,0,0,0,0],[0,0.3,0,0,0,0],[0,0,0.15,0,0,0],[0,0,0,2.5,0,0],[0,0,0,0,2.5,0],[0,0,0,0,0,2.5]])
    gain_p = np.array([[0.7,0,0,0,0,0],[0,0.7,0,0,0,0],[0,0,0.7,0,0,0],[0,0,0,2.5,0,0],[0,0,0,0,2.5,0],[0,0,0,0,0,2.5]])
    error_p = np.zeros(6)
    error_p[:3] = p_desired - p_ee
    error_p[3:] = p_trocar - p_rcm
    # print(v_attract == v_desired)
    # print(p_trocar - p_rcm)
    # print(error_p)
    # L_prcm.append(p_rcm)
    # L_pee.append(p_ee)
    if np.linalg.norm(p_ee-p_goal)<0.01:
        return np.zeros(7)
    joint_velocity_command = J_dls @ gain_p @ error_p +(np.eye(7) - J_dls@J)@w
    return joint_velocity_command

def control_speed2(joint_angles, p_goal, p_trocar,dh_params_ee,dh_params,depth_velocity): # w is [0,0,0,0,0,0, lambda0 - lambda]
    global v_prev
    global depth
    global counter_path
    elapsed_time = 0.01
    # start_time = time.time()
    alpha = 0.2 #low - accurate and responsive; high - smoother
    depth = depth + depth_velocity*elapsed_time
    p_wrist_3 = forward_kinematics(joint_angles, dh_params)[:3,3]
    p_ee = forward_kinematics(joint_angles, dh_params_with_ee)[:3,3]
    L = rcm_jacobian1(joint_angles,dh_params, dh_params_ee, p_ee, p_wrist_3,depth)
    J,p_rcm = L[0], L[1]
    # p_wrist_3 = forward_kinematics(joint_angles, dh_params)
    damping_multiplier = 0.01
    damping = np.eye(6) * damping_multiplier
    J_dls = np.linalg.pinv(J) # J.T @ np.linalg.pinv((J@J.T + damping)) # damped least square method, dont forget to inverse
    # v_repulse = vpf_repulse_link(p_wrist_3, p_ee, p_obstacles, p_rcm)
    # v_attract = vpf_attract(p_ee, p_goal)
    # v_desired = v_repulse+v_attract
    p_desired = path_generated[-1]
    counter_path +=1
    depth_desired = 0.3 - np.linalg.norm(p_desired - p_trocar)
    w = np.array([0,0,0,0,0,0, depth_desired - depth])
    # gain_p = np.array([[0.3,0,0,0,0,0],[0,0.3,0,0,0,0],[0,0,0.15,0,0,0],[0,0,0,2.5,0,0],[0,0,0,0,2.5,0],[0,0,0,0,0,2.5]])
    gain_p = np.array([[0.7,0,0,0,0,0],[0,0.7,0,0,0,0],[0,0,0.35,0,0,0],[0,0,0,2.5,0,0],[0,0,0,0,2.5,0],[0,0,0,0,0,2.5]])
    error_p = np.zeros(6)
    error_p[:3] = p_desired - p_ee
    error_p[3:] = p_trocar - p_rcm
    print(counter_path)
    # print(v_attract == v_desired)
    # print(p_trocar - p_rcm)
    # print(error_p)
    # L_prcm.append(p_rcm)
    # L_pee.append(p_ee)
    if np.linalg.norm(p_ee-p_goal)<0.01:
        return np.zeros(7)
    joint_velocity_command = J_dls @ gain_p @ error_p +(np.eye(7) - J_dls@J)@w
    return joint_velocity_command

def control_speed3(joint_angles, p_goal, p_trocar,dh_params_ee,dh_params,depth_velocity,p_obstacles): # w is [0,0,0,0,0,0, lambda0 - lambda]
    global v_prev
    global depth
    global counter_path
    global path_generated
    elapsed_time = 0.01
    n=4
    # start_time = time.time()
    alpha = 0.2 #low - accurate and responsive; high - smoother
    depth = depth + depth_velocity*elapsed_time
    p_wrist_3 = forward_kinematics(joint_angles, dh_params)[:3,3]
    p_ee = forward_kinematics(joint_angles, dh_params_with_ee)[:3,3]
    L = rcm_jacobian1(joint_angles,dh_params, dh_params_ee, p_ee, p_wrist_3,depth)
    J,p_rcm = L[0], L[1]
    counter_path = counter_path%n
    if not counter_path:
        path_generated = generate_path_steps(p_wrist_3, p_ee,p_goal, n,p_obstacles)
    # p_wrist_3 = forward_kinematics(joint_angles, dh_params)
    damping_multiplier = 0.01
    damping = np.eye(6) * damping_multiplier
    J_dls = np.linalg.pinv(J) # J.T @ np.linalg.pinv((J@J.T + damping)) # damped least square method, dont forget to inverse
    # v_repulse = vpf_repulse_link(p_wrist_3, p_ee, p_obstacles, p_rcm)
    # v_attract = vpf_attract(p_ee, p_goal)
    # v_desired = v_repulse+v_attract
    p_desired = path_generated[counter_path]
    counter_path +=1
    counter_path = counter_path%n

    # print(path_generated[-1])
    depth_desired = 0.3 - np.linalg.norm(p_desired - p_trocar)
    w = np.array([0,0,0,0,0,0, depth_desired - depth])
    # gain_p = np.array([[0.3,0,0,0,0,0],[0,0.3,0,0,0,0],[0,0,0.15,0,0,0],[0,0,0,2.5,0,0],[0,0,0,0,2.5,0],[0,0,0,0,0,2.5]])
    gain_p = np.array([[0.7,0,0,0,0,0],[0,0.7,0,0,0,0],[0,0,0.35,0,0,0],[0,0,0,5.5,0,0],[0,0,0,0,5.5,0],[0,0,0,0,0,5.5]])
    error_p = np.zeros(6)
    error_p[:3] = p_desired - p_ee
    l1 = p_trocar - p_wrist_3 
    l2 = p_ee - p_wrist_3
    l3 = np.dot(l1,l2)/(np.linalg.norm(l2)**2)*l2
    rcm_error = np.linalg.norm(-l1+l3)
    # cos_theta = np.dot(l1,l2)/np.linalg.norm(l1)/np.linalg.norm(l2)
    # rcm_error = (np.linalg.norm(l2)**2+np.linalg.norm(l1)**2-2*np.linalg.norm(l1)*np.linalg.norm(l2)*cos_theta)**0.5
    
    rcm_error_list.append(rcm_error)
    
    time_stamp_list.append(time.time())
    error_p[3:] = p_trocar - p_rcm
    if np.linalg.norm(p_ee-p_goal)<0.005:
        return np.zeros(7)
    joint_velocity_command = J_dls @ gain_p @ error_p +(np.eye(7) - J_dls@J)@w
    return joint_velocity_command

def generate_path(p_ee, p_goal):
    global path_finish
    path_generated.append(np.copy(p_ee))
    while np.linalg.norm(p_ee - p_goal)>0.01:
        print(p_ee)
        v_repulse =  np.array([0,0,0])#vpf_repulse_link(p_wrist_3, p_ee, p_obstacles, p_trocar)
        v_attract = vpf_attract(p_ee, p_goal)
        v_desired = v_repulse+v_attract
        p_ee += v_desired * 0.01
        path_generated.append(np.copy(p_ee))
    path_finish = True

def generate_path_steps(p_wrist_3,p_ee1, p_goal,n,p_obstacles):
    # global path_finish
    # path_generated.append(np.copy(p_ee))
    global local_minima
    path_generated = []
    step = 0
    p_ee = np.copy(p_ee1)
    while step <n:
        v_repulse =  vpf_repulse_link(p_wrist_3,p_ee, p_obstacles,10) #np.array([0,0,0])#vpf_repulse_link(p_wrist_3, p_ee, p_obstacles, p_trocar)
        v_repulse = np.zeros(3) #for test 1
        v_attract = vpf_attract(p_ee, p_goal,np.linalg.norm(v_repulse))
        closest_dist = float("inf")
        closest_obs = p_ee
        for i in p_obstacles:
            temp = np.array([i[0],i[1],p_ee[2]])
            if np.linalg.norm(temp - p_ee)<closest_dist:
                closest_obs = temp
        v_tan = np.cross(np.array([0,0,1]), v_repulse) * 0.6
        # v_tan = np.array([0,0,0])

        print("v attract ", v_attract )
        print("v repulse ", v_repulse )
        print("v tan ", v_tan )
        v_desired = v_repulse+v_attract +v_tan
        print("v desired " ,np.linalg.norm(v_desired))
        #speed limit
        normal = (p_ee-p_wrist_3)/np.linalg.norm(p_ee-p_wrist_3)
        lateral_vel = v_desired - np.dot(v_desired, normal)*normal
        normal_vel  = np.dot(v_desired, normal)*normal

        print("lateral vel: ", np.linalg.norm(lateral_vel)) 
        lat_vel_list.append(np.linalg.norm(lateral_vel))
        if np.linalg.norm(v_desired) > 0.5 and not np.linalg.norm(v_repulse):
            v_desired = v_desired * 0.5/np.linalg.norm(v_desired)
        print("distance ", np.linalg.norm(p_ee - p_goal))        
        # if :
            # v_desired = v_repulse+v_attract*3 +v_tan
        if np.linalg.norm(v_desired) < 0.015: #and np.linalg.norm(p_ee - p_goal) < 0.013:
            print("flag 1")
            v_desired = v_repulse/ np.linalg.norm(v_repulse) * np.linalg.norm(v_attract)*0.5 + v_attract*1.5 + v_tan
        if np.linalg.norm(p_ee - p_goal) < 0.013:
            print("flag 4")
            v_desired = v_attract
        p_ee += v_desired * 0.01
        path_generated.append(np.copy(p_ee))
        step +=1
        print("v desired recalculated" ,np.linalg.norm(v_desired))
    # path_finish = True
    return path_generated
        

def vpf_attract(p_ee, p_goal,is_obs_near):
    """
    :param p_ee: current 3*1 vector end-effector pose (position)
    :param p_goal: goal 3*1 vector
    """ 
    k_att = 25
    # if np.linalg.norm(p_ee-p_goal)<0.04:
    #     k_att = k_att* 3
    if is_obs_near:
        print("flag 3")
        k_att = 25
    if np.linalg.norm(p_ee - p_goal) > 0.030 or is_obs_near:
        print("flag 2")
        return - (p_ee - p_goal)/np.linalg.norm(p_ee - p_goal) * .15
    return -k_att * (p_ee - p_goal)

def check_apf (p_desired, p_goal):
    res = []
    while(np.linalg.norm(p_desired-p_goal)>0.001):
        alpha = 0.2 #low - accurate and responsive; high - smoother
        # L = rcm_jacobian(joint_angles,dh_params, dh_params_ee, p_trocar)
        # J,p_ee,p_rcm, depth = L[0], L[1], L[2], L[3]
        # p_wrist_3 = forward_kinematics(joint_angles, dh_params)
        # damping_multiplier = 0.01
        # damping = np.eye(6) * damping_multiplier
        # J_dls = J.T @ np.linalg.pinv((J@J.T + damping)) # damped least square method, dont forget to inverse
        # v_repulse = vpf_repulse_link(p_wrist_3, p_ee, p_obstacles, p_rcm)
        v_attract = vpf_attract(p_desired, p_goal)
        
        v_desired = v_attract
        p_desired = p_desired+v_desired * 0.01
        res.append(p_desired)
        # print(p_desired)
    return res


def vpf_repulse_link(p_wrist_3,p_ee, p_obstacles,n): #EE link vs obstacle, for 3D
    """
    :param p_ee: current 3*1 vector end-effector pose (position)
    :param p_obstacle: list of obstacle (3*1 vector)
    """ 
    d_thres = 0.05
    k_rep = 0.000003
    v_rep = np.zeros(3)
    for i in p_obstacles:
        temp = np.zeros(3)
        min_distance = float("inf")
        for j in range(n):
            x = np.copy(i)
            x[2] = x[2]+j*0.0525/n
            distance_projected = np.dot((x-p_wrist_3), (p_ee-p_wrist_3))/np.linalg.norm(p_ee - p_wrist_3) #scalar
            obj_to_link = -(x-p_wrist_3) + distance_projected * (p_ee-p_wrist_3)/np.linalg.norm(p_ee - p_wrist_3) #cartesian displacement from obs to link 
            distance = np.linalg.norm(obj_to_link)
            if  distance_projected < 0.3 and distance < min_distance and distance < d_thres and distance > 0:
                repulsion_magnitude = k_rep*(1/distance - 1/ d_thres) * (1/distance)**2
                repulsion_direction = obj_to_link/distance
                # proportion_rcm_to_obs_proj = (np.linalg.norm(p_ee-p_trocar) / np.linalg.norm(p_obs_proj-p_trocar))
                temp = repulsion_magnitude * repulsion_direction #* proportion_rcm_to_obs_proj #(np.linalg.norm(p_ee-p_rcm) / np.linalg.norm(p_obs_proj-p_rcm))
                # repulsion_magnitude = k_rep * (1.0 / (distance**2) - 1.0 / (d_thres**2))
                # repulsion_direction = diff / (distance**3)  # Normalize and scale

                # k_rep * (ee_pos - obstacle) / (dist**2 + epsilon)
        v_rep+=np.copy(temp)
                # v_rep += repulsion_magnitude * repulsion_direction
    return v_rep


    #error_p = np.concatenate((p_goal - p_ee), (p_trocar - p_rcm))
    #continue here
    #error_v = np.concatenate((v_desired - v_ee), (-v_rcm))
    #joint_velocity = J_dls @ gain_p @ error_p + J_dls @ gain_v @ error_v +(np.eye(7) - J_dls@J)@w #7*1 vector take the 7th for the next iteration 
    # return joint_velocity

def cmd_vel(joint_velocity):
    return None

def get_joint_angle(joint_state :JointState):
    global prev_time
    global elapsed_time
    counter = 0
    temp = joint_state.name
    for i in temp:
        if (i.replace("_joint","") not in joint_names):
            joint_names.append(i.replace("_joint",""))
    for i in joint_names:
        joint_velocity_data[i] = joint_state.velocity[counter]
        joint_position_data[i] = joint_state.position[counter]
        counter+=1
    current_time = joint_state.header.stamp.to_sec()
    if prev_time is not None:
        elapsed_time = current_time - prev_time
        # rospy.loginfo("Time elapsed between messages: %.6f seconds", elapsed_time)
    prev_time = current_time
    

# Define DH parameters for UR5
dh_params = [
    [0.0, np.pi/2, 0.089159, 0.0],
    [-0.425, 0.0, 0.0, 0],
    [-0.3922, 0.0, 0.0, 0.0],
    [0.0, np.pi/2, 0.10915, 0.0],
    [0.0, -np.pi/2, 0.09465, 0.0],
    [0.0, 0.0, 0.0823, 0.0]
]
dh_params_with_ee = [
    [0.0, np.pi/2, 0.089159, 0.0],
    [-0.425, 0.0, 0.0, 0],
    [-0.3922, 0.0, 0.0, 0.0],
    [0.0, np.pi/2, 0.10915, 0.0],
    [0.0, -np.pi/2, 0.09465, 0.0],
    [0.0, 0.0, 0.0823+0.3, 0.0]
]

# p_trocar = [1.34,-1.515, 1.557, -2.166, -1.315, 0]
# print(forward_kinematics(p_trocar, dh_params_with_ee)[:3,3])
# p_goal = [1.473,-1.219, 1.604, -2.593, -1.508, 0]
# print(forward_kinematics(p_goal, dh_params_with_ee)[:3,3])
# p_obs = [-1.387,-2.847, 0.298, -2.157, 1.580, 0]
# # print(forward_kinematics(p_obs, dh_params_with_ee)[:3,3])
# x = [["0.04867848 -0.83445059 0.04993548"],["0.04867848 -0.79070059 0.04993548"],["0.00492848 -0.81257559 0.04993548"],["0.00492848 -0.76882559 0.04993548"], ["0.00492848 -0.85632559 0.04993548"]]  
# p_obstacles =[]
# for i in x:
#     p_obstacles.append((list(map(lambda x : float(x),i[0].split(" ")))))
# res = []
# # print(np.array([0.04867848, -0.79070059, 0.04993548]))
# for i in range(len(p_obstacles)):
#     res.append([round(p_obstacles[i][0]-0.0875, 8) , p_obstacles[i][1] , p_obstacles[i][2]])
# # p_obstacles = [map(lambda x : float(x),"0.04867848 -0.79070059 0.04993548".split(" "))]
# p_obstacles.extend(res)
# print(p_obstacles)


# '''
# apf with steps
lat_vel_list =[]
test_type = input("test type (1 or 2):")
test_no= input("test_no (0-19) and (0-39):")
delta_target = 0.013
target_heigth = 0.08
rcm_error_list = []
time_stamp_list = []
goal_test_1_points = [np.array([0.05312177, -0.82213129,  0.11547495]), np.array([-0.035, -0.970, 0.120]), np.array([0.051, -0.722,  0.110]), np.array([0.054, -0.869,  0.130]), np.array([-0.164, -0.729,  0.200]),np.array([-0.134, -0.799,  0.200]),np.array([-0.114, -0.739,  0.150]),np.array([-0.08257152, -0.83632559, 0.080]), np.array([-0.074, -0.870, 0.075]), np.array([-0.074, -0.900, 0.065]), np.array([-0.174, -0.800, 0.115]), np.array([-0.134, -0.810, 0.145]), np.array([-0.104, -0.810, 0.215]), np.array([0.04767848, -0.72070059, 0.07993548]), np.array([-0.124, -0.810, 0.19]), np.array([0.04767848, -0.82070059, 0.1903548]), np.array([0.04967848, -0.80070059, 0.1703548]), np.array([0.05167848, -0.86070059, 0.1503548]), np.array([0.05367848, -0.88070059, 0.1103548]), np.array([-0.10367848, -0.68070059, 0.1303548])] 
p_obstacles = [[0.04867848, -0.83445059, 0.04993548], [0.04867848, -0.79070059, 0.04993548], [0.00492848, -0.81257559, 0.04993548], [0.00492848, -0.76882559, 0.04993548], [0.00492848, -0.85632559, 0.04993548], [-0.03882152, -0.83445059, 0.04993548], [-0.03882152, -0.79070059, 0.04993548], [-0.08257152, -0.81257559, 0.04993548], [-0.08257152, -0.76882559, 0.04993548], [-0.08257152, -0.85632559, 0.04993548]]
goal_test_2_points = []
for i in p_obstacles:
    goal_test_2_points.append([i[0] + delta_target, i[1], target_heigth])
    goal_test_2_points.append([i[0] - delta_target, i[1], target_heigth])
    goal_test_2_points.append([i[0] , i[1] + delta_target, target_heigth])
    goal_test_2_points.append([i[0] , i[1] - delta_target, target_heigth])
    # goal_test_2_points.append(i[], i[], 0.01)
if __name__ == "__main__":
    global counter_path
    counter_path =0
    # path_finish = False
    # p_trocar = np.array([0.04590928, -0.99686627, 0.22717402]) # setup 1
    
    
    p_trocar = np.array([0.04248387, -0.71920688 , 0.23206623])
    # p_goal = np.array([0.04325266, -1.03956855, 0.18399166]) # setup 1 test 1
    # p_goal = np.array([0.03325266, -1.13956855, 0.14399166]) # setup 1 test 2
    # p_goal = np.array([0.05312177, -0.82213129,  0.09047495])
    # p_goal = np.array([0.05312177, -0.88213129,  0.09047495])
    # p_goal = np.array([0.05312177 - 0.04375 *2, -0.82213129,  0.09047495])
    # p_goal = np.array([0.04867848 + 0, -0.83445059 + 0.012, 0.07])
    if test_type == "1":
        p_goal = np.array(goal_test_1_points[int(test_no)])
    elif test_type == "2":
        p_goal = np.array(goal_test_2_points[int(test_no)])
    else:
        print("invalid num")
        exit()
    
    depth_velocity = 0
    depth = 0.3
    elapsed_time = 0
    v_prev = np.array([0,0,0,0,0,0])
    prev_time = None
    joint_T = {} 
    rospy.init_node("VPF_controller")
    joint_names = []
    joint_names_in_order = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
    joint_angles_in_order=[0,0,0,0,0,0]
    pub_dic = {}
    joint_velocity_data={}
    joint_position_data={}
    sub = rospy.Subscriber(f"/joint_states", JointState, callback=get_joint_angle)
    for i in joint_names_in_order:
        pub_dic[i] = [rospy.Publisher(f"/{i}_joint_velocity_controller/command", Float64, queue_size=10),f"/{i}_joint_velocity_controller/command"]
    x = rospy.Publisher(f"/shoulder_pan_joint_velocity_controller/command",Float64, queue_size=10)
    rospy.loginfo("Node is runnng")
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        try :
            # rospy.loginfo(joint_position_data)
            # print(len(pub_dic))
            if len (joint_position_data) == 6: #subscriber need some time to load
                
                for i in range(len(joint_names_in_order)):
                    joint_angles_in_order[i] = joint_position_data[joint_names_in_order[i]]

                # do some calculation
                joint_vel = control_speed3(joint_angles_in_order, p_goal, p_trocar,dh_params_with_ee,dh_params,depth_velocity,p_obstacles)
                depth_velocity = joint_vel[6]
                # joint_vel = control_speed1(joint_angles_in_order, p_goal, p_trocar,dh_params_with_ee, dh_params)
                # depth = depth + joint_vel[6]*0.01 # update depth after each iteration
                # for i in range(len(joint_names_in_order)):
                #     joint_angles_in_order[i] = joint_position_data[joint_names_in_order[i]]
                for i in range(len(joint_names_in_order)):
                    pub_dic[joint_names_in_order[i]][0].publish(joint_vel[i])
                if not np.linalg.norm(joint_vel):
                    rospy.loginfo("goal reached")
                    break

                # control()
                # control(joint_velocity_data, p_goal, p_trocar,dh_params_ee,dh_params,w)
                # pub_dic["shoulder_pan"][0].pub
                
        except RuntimeError:
            rospy.logwarn("error")
            pass
        rate.sleep()
# '''
start_time = time_stamp_list[0]
for i in range(len(time_stamp_list)):
    time_stamp_list[i] = time_stamp_list[i]-start_time
res = np.array([rcm_error_list,time_stamp_list]).T
np.savetxt("rcm_error type"+ test_type +" no "+ test_no +  ".csv" , res, delimiter=",")
print("max deviation ", max(rcm_error_list[1:]))
print("avg deviation ", sum(rcm_error_list[1:])/(len(rcm_error_list)-1))
print("max lat vel", max(lat_vel_list))
print("avg lat_vel ", sum(lat_vel_list[1:])/(len(lat_vel_list)-1))
# print(len(goal_test_1_points))
# for i in goal_test_1_points:
#     print(str(list(i)).replace(",","").replace("[","").replace("]",""))
# np.savetxt("time_stamp.csv", tiime_stamp_list, delimiter=",")


# z = time.time()
# elapsed_time = 1
# p_obstacles = []
# v_prev = np.array([0,0,0,0,0,0])
# T_world_to_base = np.array([[-1,0,0,0],[0,-1,0,0],[0,0,1,0.05],[0,0,0,1]])
# x = np.array([1.483,-0.326,-0.406,-1.443, -1.508, -0.086])
# y = np.array([1.476,-0.749,0.695,4.021, 4.797, 6.202])
# p_trocar = np.array([0.04590928, -0.99686627, 0.22717402])
# p_goal = np.array([0.04325266, -1.03956855, 0.18399166])
# joint_angles = x
# print(control_speed(joint_angles, p_goal, p_trocar,dh_params_with_ee,dh_params))
# joint_angles += control_speed(joint_angles, p_goal, p_trocar,dh_params_with_ee,dh_params)[:6] * 1
# print(joint_angles)
# print(control_speed(joint_angles, p_goal, p_trocar,dh_params_with_ee,dh_params))
# print(v_prev)
# x = np.array([0,0,0,0,0,0])
# print(forward_kinematics(x,dh_params_with_ee))
# print(forward_kinematics(y,dh_params_with_ee))
# # print(forward_kinematics(y,dh_params_with_ee))
# print (1/(time.time()-z))


# p_desired = np.array([0.04590928, -0.99686627, 0.22717402])
# p_trocar = np.array([0.04590928, -0.99686627, 0.22717402])
# p_goal = np.array([0.04325266, -1.03956855, 0.18399166])
# depth = 0.3
# joint_angles = [1.483, -0.326, -0.406, -1.443, -1.508, -0.086]
# depth_velocity = 0
# p_obstacles =[]

# L =[]
# L_prcm = []
# L_pee = []
# res = control_speed1(joint_angles, p_goal, p_trocar,dh_params_with_ee,dh_params,depth_velocity)
# while np.linalg.norm(res) > 0.0000000000001:
#     depth = depth + res[6]*0.01
#     joint_angles += res[:6]*0.01
#     L.append(res)
#     res = control_speed1(joint_angles, p_goal, p_trocar,dh_params_with_ee,dh_params,depth_velocity)
#     print(np.linalg.norm(res))
# print(L)

# L_pee =L_prcm [len(L_prcm)//2:]

    

# print(check_apf(p_desired, p_goal)[0])
# print(control_speed1(joint_angles, p_goal, p_trocar,dh_params_with_ee,dh_params,depth_velocity))

'''
# control algo with reactive apf
if __name__ == "__main__":
    p_trocar = np.array([0.04590928, -0.99686627, 0.22717402])
    # p_goal = np.array([0.04325266, -1.03956855, 0.18399166]) # test 1
    p_goal = np.array([0.03325266, -1.13956855, 0.14399166]) #test 2
    p_obstacles =[]
    depth_velocity = 0e
    depth = 0.3
    elapsed_time = 0
    v_prev = np.array([0,0,0,0,0,0])
    prev_time = None
    joint_T = {} 
    rospy.init_node("VPF_controller")
    joint_names = []
    joint_names_in_order = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
    joint_angles_in_order=[0,0,0,0,0,0]
    pub_dic = {}
    joint_velocity_data={}
    joint_position_data={}
    sub = rospy.Subscriber(f"/joint_states", JointState, callback=get_joint_angle)
    for i in joint_names_in_order:
        pub_dic[i] = [rospy.Publisher(f"/{i}_joint_velocity_controller/command", Float64, queue_size=10),f"/{i}_joint_velocity_controller/command"]
    x = rospy.Publisher(f"/shoulder_pan_joint_velocity_controller/command",Float64, queue_size=10)
    rospy.loginfo("Node is runnng")
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        try :
            rospy.loginfo(joint_position_data)
            # print(len(pub_dic))
            if len (joint_position_data) == 6: #subscriber need some time to load
                
                for i in range(len(joint_names_in_order)):
                    joint_angles_in_order[i] = joint_position_data[joint_names_in_order[i]]

                # do some calculation
                joint_vel = control_speed1(joint_angles_in_order, p_goal, p_trocar,dh_params_with_ee,dh_params,depth_velocity)
                depth_velocity = joint_vel[6]
                # joint_vel = control_speed1(joint_angles_in_order, p_goal, p_trocar,dh_params_with_ee, dh_params)
                # depth = depth + joint_vel[6]*0.01 # update depth after each iteration
                # for i in range(len(joint_names_in_order)):
                #     joint_angles_in_order[i] = joint_position_data[joint_names_in_order[i]]
                for i in range(len(joint_names_in_order)):
                    pub_dic[joint_names_in_order[i]][0].publish(joint_vel[i])
                if not np.linalg.norm(joint_vel):
                    rospy.loginfo("goal reached")
                    break

                # control()
                # control(joint_velocity_data, p_goal, p_trocar,dh_params_ee,dh_params,w)
                # pub_dic["shoulder_pan"][0].pub
                
        except:
            rospy.logwarn("error")
            pass
        rate.sleep()
# '''

'''
if __name__ == "__main__":
    path_finish = False
    counter_path = 0
    p_trocar = np.array([0.04590928, -0.99686627, 0.22717402])
    # p_goal = np.array([0.04325266, -1.03956855, 0.18399166]) # test 1
    p_goal = np.array([0.03325266, -1.13956855, 0.14399166]) #test 2
    p_obstacles =[]
    depth_velocity = 0
    depth = 0.3
    elapsed_time = 0
    v_prev = np.array([0,0,0,0,0,0])
    prev_time = None
    joint_T = {} 
    rospy.init_node("VPF_controller")
    joint_names = []
    joint_names_in_order = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
    joint_angles_in_order=[0,0,0,0,0,0]
    pub_dic = {}
    joint_velocity_data={}
    joint_position_data={}
    sub = rospy.Subscriber(f"/joint_states", JointState, callback=get_joint_angle)
    for i in joint_names_in_order:
        pub_dic[i] = [rospy.Publisher(f"/{i}_joint_velocity_controller/command", Float64, queue_size=10),f"/{i}_joint_velocity_controller/command"]
    x = rospy.Publisher(f"/shoulder_pan_joint_velocity_controller/command",Float64, queue_size=10)
    rospy.loginfo("Node is runnng")
    rate = rospy.Rate(100)
    path_generated = []
    generate_path(p_trocar, p_goal)
    print(path_generated)
    while not rospy.is_shutdown():
        try :
            rospy.loginfo(joint_position_data)
            # print(len(pub_dic))
            if len (joint_position_data) == 6 and path_finish:  #subscriber need some time to load
                # 
                for i in range(len(joint_names_in_order)):
                    joint_angles_in_order[i] = joint_position_data[joint_names_in_order[i]]
# 
                # do some calculation
                # 
                joint_vel = control_speed2(joint_angles_in_order, p_goal, p_trocar,dh_params_with_ee,dh_params,depth_velocity)
                depth_velocity = joint_vel[6]
                # joint_vel = control_speed1(joint_angles_in_order, p_goal, p_trocar,dh_params_with_ee, dh_params)
                # depth = depth + joint_vel[6]*0.01 # update depth after each iteration
                # for i in range(len(joint_names_in_order)):
                #     joint_angles_in_order[i] = joint_position_data[joint_names_in_order[i]]
                for i in range(len(joint_names_in_order)):
                    pub_dic[joint_names_in_order[i]][0].publish(joint_vel[i])
                if not np.linalg.norm(joint_vel):
                    rospy.loginfo("goal reached")
                    break
# 
                # control()
                # control(joint_velocity_data, p_goal, p_trocar,dh_params_ee,dh_params,w)
                # pub_dic["shoulder_pan"][0].pub
                # 
        except:
            rospy.logwarn("error")
            pass
        rate.sleep()
# '''



# target = np.array([[-1,0,0,-0.003],[0,0,-1,-1.915],[0,-1,0,1.001],[0,0,0,1]])
# x= numerical_ik(target,np.array([0.1,0.2,0.4,0.3,4.7500.2,0.1]),dh_params)
# print(x)
#print(position_jacobian([0,0,0,0,0,0],dh_params))

# print(forward_kinematics(x,dh_params))
# print(forward_kinematics([0,-1.57,0,-1.57,0,0],dh_params)) #@ np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.3],[0,0,0,1]]))
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

# import numpy as np
# from pytransform3d import rotations as pr

# def forwardKinematics(theta,tcp=None):  
#     # theta are the joint angles in radians
#     # tcp is the tcp offset as a pose (x,y,z,rx,ry,rz)import numpy as np
# from pytransform3d import rotations as pr

# r/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/):
#     a = np.array([0.0000,-0.425,-0.39225,0.0000,0.0000,0.0000])
#     d = np.array([0.089159,0.0000,0.0000,0.10915,0.09465,0.0823])
#     alpha = np.array([np.pi/2,0.,0.,np.pi/2,-np.pi/2,0.])
    
#     #values from calibration.conf:
#     # delta_a = np.array([ 3.1576640107943976e-05, 0.298634925475782076, 0.227031257526500829, -8.27068507303316573e-05, 3.6195435783833642e-05, 0])
#     # delta_d = np.array([ 5.82932048768247668e-05, 362.998939868892023, -614.839459588742898, 251.84113332747981, 0.000164511802564715204, -0.000899906496469232708])
#     # delta_alpha = np.array([ -0.000774756642435869836, 0.00144883356002286951, -0.00181081418698111852, 0.00068792563586761446, 0.000450856239573305118, 0])
#     # delta_theta = np.array([ 1.09391516130152855e-07, 1.03245736607748673, 6.17452995676434124, -0.92380698472218048, 6.42771759845617296e-07, -3.18941184192234051e-08])

#     # a += delta_a
#     # d += delta_d
#     # alpha+=delta_alpha
#     # theta=theta.copy()+delta_theta

#     ot = np.eye(4)
#     for i in range(6):
#         ot = ot @ np.array([[np.cos(theta[i]), -(np.sin(theta[i]))*np.cos(alpha[i]), np.sin(theta[i])*np.sin(alpha[i]), a[i]*np.cos(theta[i])],[np.sin(theta[i]),np.cos(theta[i])*np.cos(alpha[i]),-(np.cos(theta[i]))*np.sin(alpha[i]),a[i]*np.sin(theta[i])], [0.0,np.sin(alpha[i]),np.cos(alpha[i]),d[i]],[0.0,0.0,0.0,1.0]])
#     world_to_robot = np.array([
#         [-1,0,0,0],
#         [0,-1,0,0],
#         [0,0,1,0],
#         [0,0,0,1]
#     ])
#     ot = world_to_robot @ ot
#     if not tcp is None:
#         offset = np.array([ot[0,3],ot[1,3],ot[2,3]])  + (ot[:3,:3] @ tcp[:3])
#         newAngle = pr.compact_axis_angle_from_matrix( ot[:3,:3] @ pr.matrix_from_compact_axis_angle(tcp[3:]))
#         return np.array([offset[0],offset[1],offset[2],newAngle[0],newAngle[1],newAngle[2]])
#     else:
#         axisAngle = pr.compact_axis_angle_from_matrix(ot[:3,:3])
#         return np.array([ot[0,3],ot[1,3],ot[2,3],axisAngle[0],axisAngle[1],axisAngle[2]])
    
# world_to_robot = np.array([
#     [-1,0,0,0],
#     [0,-1,0,0],
#     [0,0,1,0],
#     [0,0,0,1]
# ])
# end_effector_pose = forwardKinematics([0,-1.57,0,0,1.57,0],[0,0,0,0,0,0])
# print(np.dot(world_to_robot, end_effector_pose))
# print(forwardKinematics([0,-1.57,0,-1.57,0,0],[0,0,0,0,0,0]))