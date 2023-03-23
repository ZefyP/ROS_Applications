#!/usr/bin/env python3

import rospy 
from tuos_ros_msgs.srv import SetBool, SetBoolRequest 
import sys 

service_name = "move_service" 

# Initialise the client node
rospy.init_node(f"{service_name}_client") 

# Wait until the service that we want to call is actually running, execution of this node will
# not progress beyond this point until the service is detected on the ROS network 
# (launched by # the Server).
rospy.wait_for_service(service_name) 

# Wait until the service that we want to call is actually running, execution of this node will 
# not progress beyond this point until the service is detected on the ROS network 
# (launched by the Server).
service = rospy.ServiceProxy(service_name, SetBool) 

# Create an instance of the SetBoolResponse() part 
# of the service message, and populate this with the data that the server is expecting.
service_request =  SetBoolRequest() 
service_request.request_signal = True 


# Use the rospy.ServiceProxy instance that was created earlier (called service) to actually 
# send the service_request message to the service and obtain a response back from the Server 
# (once it's complete).
service_response = service(service_request) 
print(service_response) 
