import rospy
import smach
from geometry_msgs.msg import Twist, Vector3
import time

class navroomD(smach.State):


    def __init__(self, visit_amount):
        smach.State.__init__(self, outcomes=["success","finished"],
                             input_keys=["room_id"],
                             output_keys=["room_id"])
        self.visit_amount = visit_amount
        self.visitCounter = 0
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)



    def execute(self, userdata):
        rospy.loginfo("Navigating in room D")

        move_duration = 30  # seconds
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time).to_sec() < move_duration:
            twist = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 3))
            self.pub.publish(twist)
            rospy.sleep(0.1)
        self.pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))

        self.visitCounter = self.visitCounter+1
        if (self.visitCounter == self.visit_amount) :
            return "finished"

        return "success"



