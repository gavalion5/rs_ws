#!/usr/bin/env python3
import rospy
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64
import time
import roboticstoolbox as rtb
from math import cos,sin,pi
import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.setpoint = setpoint  # Desired target value

        # Internal variables for the PID calculation
        self.integral = 0
        self.previous_error = 0
        self.previous_time = None

    def reset(self):
        """Reset the PID controller's integral and previous error."""
        self.integral = 0
        self.previous_error = 0
        self.previous_time = None

    def compute(self, measurement):
        """Compute the PID control output."""
        # Calculate error
        error = self.setpoint - measurement

        # Get current time and calculate time difference
        current_time = time.time()
        delta_time = current_time - self.previous_time if self.previous_time else 0
        self.previous_time = current_time

        # Calculate integral
        self.integral += error * delta_time

        # Calculate derivative
        derivative = (error - self.previous_error) / delta_time if delta_time > 0 else 0
        self.previous_error = error

        # Compute PID output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output

def inverse_kinematics():
    rbt = rtb.models.DH.ur5()
    pass

def homogeneous_transform(q, d, a, alpha):
    
    T = np.array([[cos(q),            -sin(q),           0,             a          ],
                [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                 0,                 0,           0,             1]])
    return T

def get_joint_angle(joint_state :JointControllerState, joint_name):
    joint_data[joint_name] = joint_state.process_value
# def inverse_kinematics():

def controll_joint():
    for i in joint_names:
        controller_dic[i].setpoint = -3 # in radiant
        pub_dic[i][0].publish(controller_dic[i].compute(joint_data[i]))

if __name__ == "__main__":
    kp,ki,kd = 10,0,5
    dh_param = {"shoulder_pan" : [0.08946,0,pi/2],"shoulder_lift" :[0,-0.4250,0], "elbow" : [0,-0.3922 ,0], "wrist_1" : [ 0.1091,0,pi/2], "wrist_2" :[0.09465,0,-pi/2], "wrist_3" : [0.0823, 0, 0]}
    goal = [0,0,0,0,0,0] # x,y,z,r,p, y
    joint_T = {} 
    rospy.init_node("PID_controller")
    joint_names = ["elbow", "shoulder_lift", "shoulder_pan", "wrist_1", "wrist_2", "wrist_3"]
    #joint_names = ["elbow"]

    pub_dic = {}
    joint_data={}
    controller_dic = {}    
    
    for i in joint_names:
        pub_dic[i] = [rospy.Publisher(f"/{i}_joint_position_controller/command", Float64, queue_size=10),"/{i}_joint_position_controller/command"]
        sub = rospy.Subscriber(f"/{i}_joint_position_controller/state", JointControllerState, callback=get_joint_angle,callback_args=f"{i}")
        controller_dic[i] = PIDController(kp, ki, kd)
        # dh_param[i] = None
    rospy.loginfo("Node is running")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try :
            if len (joint_data) == 6: #subscriber need some time to load
                for i in joint_data:
                    #rospy.loginfo(f"{i} : {len(joint_data)}")
                    rospy.loginfo(f"{i} : {joint_data[i]}")
                    q = joint_data[i]
                    joint_T[i] = homogeneous_transform(q, dh_param[i][0], dh_param[i][1], dh_param[i][2])
                controll_joint()
        except :
            rospy.logwarn("error")
            pass
        rate.sleep()

