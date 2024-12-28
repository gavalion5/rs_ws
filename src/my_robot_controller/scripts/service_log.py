#!/usr/bin/env python3

import rospy
from turtlesim.srv import SetPen, SetPenResponse # Replace with your package and service type


def add_two_ints(req):
    return SetPenResponse(req.a + req.b)

# def add_two_ints_server():
    
if __name__ == "__main__":
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', SetPen, add_two_ints)
    print(1)
    s.spin()
# def echo_service_callback(request):
#     print(1)
#     rospy.loginfo(f"Received request: r={request.r}, g={request.g}, b={request.b}, width={request.width}, off={request.off}")
#     # Forward the request to the actual /turtle1/set_pen service
#     try:
#         response = set_pen_proxy(request.r, request.g, request.b, request.width, request.off)
#         rospy.loginfo(f"Response: {response}")
#         return response
#     except rospy.ServiceException as e:
#         rospy.logerr(f"Service call failed: {e}")
#         return SetPenResponse()

# if __name__ == "__main__":
#     rospy.init_node("set_pen_echo_server")

#     # Wait for the actual /turtle1/set_pen service to become available
#     rospy.wait_for_service('/turtle1/set_pen')
    
#     # Create a proxy to the actual service
#     set_pen_proxy = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
#     # Create a new service that calls the echo_service_callback
#     echo_service = rospy.Service('/set_pen_echo', SetPen, echo_service_callback)

#     rospy.loginfo("Echo service for /turtle1/set_pen is ready.")
#     rospy.spin()


# import rospy
# from turtlesim.srv import SetPen   # Import the service type

# def print_service_fields(service_class):
#     # Retrieve the request and response fields
#     request_fields = service_class._request_class.__slots__
#     response_fields = service_class._response_class.__slots__

#     print("Request fields:")
#     for field in request_fields:
#         print(f" - {field}")

#     print("Response fields:")
#     for field in response_fields:
#         print(f" - {field}")

# if __name__ == "__main__":
#     rospy.init_node("service_field_printer")

#     # Specify the service type (e.g., AddTwoInts)
#     print_service_fields(SetPen)
#     x = SetPen._request_class.
#     rospy.spin()

# import rospy
# from gazebo_msgs.srv import ApplyJointEffort

# def monitor_service():
#     rospy.init_node('service_monitor')

#     # Wait until the service is available
#     rospy.wait_for_service('/gazebo/apply_joint_effort')
#     try:
#         add_two_ints = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
#         while not rospy.is_shutdown():
#             # Call the service with sample data
#             response = add_two_ints(5, 3)
#             rospy.loginfo("Service response: %s", response)
#             rospy.sleep(1)
#     except rospy.ServiceException as e:
#         rospy.logwarn("Service call failed: %s", e)

# if __name__ == '__main__':
#     monitor_service()



