#!/usr/bin/env python3

import rospy
import actionlib
from second_coursework.msg import checkroomFeedback, checkroomResult, checkroomAction
import roslib
from second_coursework.srv import movetoroom
import smach
import smach_ros
from smach import CBState, Concurrence
from geometry_msgs.msg import Twist, Vector3
from movetoA import movetoA
from movetoB import movetoB
from movetoD import movetoD
from navroomA import navigate
from navroomB import navroomB
from navroomD import navroomD
from yolo_state import YOLODetectionState


class RobotBehaviour:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("checkroom", checkroomAction, self.RobotCheckRoom, False)
        self.server.start()



    def RobotCheckRoom(self, goal):


        sm = smach.StateMachine(outcomes=["finish"])
        sm.userdata.rule_violations = [0, 0]

        with sm:


            smach.StateMachine.add("movetoA", movetoA(), transitions={"success": "CONA"})
            smach.StateMachine.add("movetoB", movetoB(), transitions={"success" : "CONB"})
            smach.StateMachine.add("movetoD", movetoD(), transitions={"success" : "COND"})

            smc_a = smach.Concurrence(outcomes= ["done"],
                                      default_outcome="done",
                                      input_keys=["room_id", "rule_violations"],
                                      output_keys=["room_id"],
                                      outcome_map={"done": {"navroomA": "success", "YOLODetectionState": "detected"}})


            smc_b = smach.Concurrence(outcomes= ["done"],
                                      default_outcome="done",
                                      input_keys=["room_id", "rule_violations"],
                                      output_keys=["room_id"],
                                      outcome_map={"done": {"navroomB": "success", "YOLODetectionState": "detected"}})


            smc_d = smach.Concurrence(outcomes= ["done", "finished"],
                                      default_outcome="done",
                                      input_keys=["room_id", "rule_violations"],
                                      output_keys=["room_id"],
                                      outcome_map={ "done": {"navroomD": "success", "YOLODetectionState": "detected"},
                                                    "finished": {"navroomD": "finished"}})

            pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

            with smc_a:
                smach.Concurrence.add("navroomA", CBState(navigate, cb_args = [pub]))
                smach.Concurrence.add("YOLODetectionState", YOLODetectionState(self.server))

            smach.StateMachine.add("CONA", smc_a,
                                       transitions = {"done": "movetoB"})

            with smc_b:
                smach.Concurrence.add("navroomB", navroomB())
                smach.Concurrence.add("YOLODetectionState", YOLODetectionState(self.server))

            smach.StateMachine.add("CONB", smc_b,
                                       transitions = {"done": "movetoD"})

            with smc_d:
                smach.Concurrence.add("navroomD", navroomD(goal.num_checks))
                smach.Concurrence.add("YOLODetectionState", YOLODetectionState(self.server))

            smach.StateMachine.add("COND", smc_d,
                                   transitions={"done": "movetoA", "finished": "finish"})


        outcome = sm.execute()

        if outcome == "finish":
            rules_broken = sm.userdata.rule_violations
            result = checkroomResult(rules_broken)
            self.server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("checkRoom", anonymous=True)
    rospy.wait_for_service("set_room")
    moveservice = rospy.ServiceProxy("movetoroom", movetoroom)
    server = RobotBehaviour()
    rospy.spin()






