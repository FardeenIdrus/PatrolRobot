

import rospy
import smach
from geometry_msgs.msg import Twist, Vector3

@smach.cb_interface(input_keys=["room_id"], output_keys=["room_id"], outcomes=["success"])
def navigate(userdata, pub):
    start_time = rospy.Time.now()

    while (rospy.Time.now() - start_time).to_sec() < 30:
        pub.publish(Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 3)))
        rospy.sleep(0.1)

    return "success"















