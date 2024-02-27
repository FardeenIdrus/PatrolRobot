#!/usr/bin/env python3
import rospy
from second_coursework.msg import checkroomAction, checkroomGoal, checkroomFeedback
import actionlib


def main():
    rospy.init_node('main_node')

    client = actionlib.SimpleActionClient("checkroom", checkroomAction)
    client.wait_for_server()
    goal = checkroomGoal()
    goal.num_checks = rospy.get_param("~nchecks_param", default=2)

    client.send_goal(goal)
    client.wait_for_result()

    result = client.get_result()

    print("==========================================")
    rospy.loginfo(result)
    print(result)
    print("=========================================")

    return result


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


