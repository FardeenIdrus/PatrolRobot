import rospy
import smach
from geometry_msgs.msg import Twist, Vector3
import time

class navroomB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["success"], input_keys=["room_id"], output_keys=["room_id"])
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("Navigating in room B")

        move_duration = 30
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time).to_sec() < move_duration:
            twist = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 3))
            self.pub.publish(twist)
            rospy.sleep(0.1)
        self.pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))

        return 'success'



