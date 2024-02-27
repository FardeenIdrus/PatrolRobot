#!/usr/bin/env python3


import rospy
from second_coursework.srv import movetoroom, movetoroomRequest

rospy.init_node("client_node", anonymous=True)

rospy.wait_for_service("set_room")

move_room_proxy = rospy.ServiceProxy("set_room", movetoroom)

move_room_proxy("B")

