import rospy
import smach
from second_coursework.srv import movetoroom, movetoroomRequest

class movetoB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["success"], input_keys=["room_id"], output_keys=["room_id"])


    def execute(self, userdata):
        rospy.loginfo("moving to room B")

        rospy.wait_for_service("/set_room")

        move_room_proxy = rospy.ServiceProxy("/set_room", movetoroom)

        move_room_proxy("B")

        userdata.room_id = "B"

        return 'success'