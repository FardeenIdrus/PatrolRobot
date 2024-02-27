#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from second_coursework.srv import movetoroom, movetoroomResponse, movetoroomRequest
import math
from actionlib_msgs.msg import GoalStatus



def move_to_room(request):
    room_name = request.roomName

    print("MOVING TO ROOM ", request.roomName)
    if room_name == "A":

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.header.frame_id = "map"

        goal.target_pose.pose.position.x = 1.7
        goal.target_pose.pose.position.y = 8.0
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = math.sin(math.pi / 4)
        goal.target_pose.pose.orientation.w = math.cos(math.pi / 4)


        move_base_client.send_goal(goal)
        move_base_client.wait_for_result()

    if room_name == "B":
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.header.frame_id = "map"

        goal.target_pose.pose.position.x = 6.0
        goal.target_pose.pose.position.y = 8.0
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = math.sin(math.pi / 4)
        goal.target_pose.pose.orientation.w = math.cos(math.pi / 4)

        move_base_client.send_goal(goal)
        move_base_client.wait_for_result()


    if room_name == "D":
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.header.frame_id = "map"

        goal.target_pose.pose.position.x = 1.7
        goal.target_pose.pose.position.y = 3.1
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = math.sin(math.pi / 4)
        goal.target_pose.pose.orientation.w = math.cos(math.pi / 4)

        move_base_client.send_goal(goal)
        move_base_client.wait_for_result()




    status = move_base_client.get_state()

    if status == GoalStatus.SUCCEEDED:
        return movetoroomResponse(True)
    return movetoroomResponse(False)

if __name__ == "__main__":
    rospy.init_node("room_node")

    move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base_client.wait_for_server()

    rospy.Service("/set_room", movetoroom, move_to_room)

    rospy.spin()


























