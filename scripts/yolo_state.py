#!/usr/bin/env python3
import rospy
import smach
from second_coursework.srv import YOLOlastframe, YOLOlastframeRequest
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from second_coursework.msg import checkroomFeedback, checkroomResult, checkroomAction


class YOLODetectionState(smach.State):
    def __init__(self, server):
        smach.State.__init__(self, outcomes=['detected', 'not_detected'],
                             input_keys=["room_id", 'rule_violations'], output_keys=["room_id"])

        self.server = server

        self.current_pose = PoseWithCovarianceStamped()
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_pose)

        self.yolo_proxy = rospy.ServiceProxy('/detect_frame', YOLOlastframe)

        self.speech_pub = rospy.Publisher("/speech", String, queue_size =1 )

    def update_pose(self, msg):
        self.current_pose = msg.pose.pose.position

    def execute(self, userdata):
        rospy.loginfo("Executing YOLO Detection State")



        rospy.loginfo(userdata.room_id)

        start_time = rospy.Time.now().to_sec()

        duration = 30

        cat_dog_detected_before = False
        person_detected_before = False
        rospy.wait_for_service("/detect_frame")
        request = YOLOlastframeRequest()
        response = self.yolo_proxy(request)

        while rospy.Time.now().to_sec() - start_time < duration:

            cat_detected = False
            dog_detected = False
            person_in_D = False

            for detection in response.detections:
                print(detection)
                if detection.name == "cat":
                    print("cat detected")
                    cat_detected = True
                elif detection.name == "dog":
                    print("dog detected")
                    dog_detected = True
                elif userdata.room_id == "D" and detection.name == "person":
                    person_in_D = True

                if cat_detected and dog_detected and not cat_dog_detected_before:
                    cat_dog_detected_before = True
                    rospy.loginfo("Rule violated: Cat and dog detected in the same room.")
                    feedback = checkroomFeedback()
                    feedback.robot_position = self.current_pose
                    feedback.rule_broken = 1
                    userdata.rule_violations[0] += 1
                    print(feedback)
                    self.server.publish_feedback(feedback)
                    self.speech_pub.publish("Cat and dog leave!")

                elif person_in_D and not person_detected_before:
                    person_detected_before = True
                    rospy.loginfo("Person spotted in room D")
                    userdata.rule_violations[1] += 1
                    feedback = checkroomFeedback()
                    feedback.robot_position = self.current_pose
                    feedback.rule_broken = 2
                    print(feedback)
                    self.server.publish_feedback(feedback)
                    self.speech_pub.publish("Leave room D")

            response = self.yolo_proxy(request)


        return 'detected'


