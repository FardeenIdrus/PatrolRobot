import rospy
import smach
from second_coursework.srv import movetoroom, movetoroomRequest

class movetoA(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["success"], input_keys=["room_id"], output_keys=["room_id"])

    def execute(self, userdata):

        rospy.loginfo("Moving to Room A")

        move_room_proxy = rospy.ServiceProxy("/set_room", movetoroom)

        move_room_proxy("A")

        userdata.room_id = "A"

        return 'success'
