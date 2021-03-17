# !/usr/bin/env python
# import roslib
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TwistWithCovarianceStamped

def listener():
    rospy.init_node('anyplan_listener')

    rospy.Subscriber("/state_estimator/twist", TwistWithCovarianceStamped, callback)

    rospy.spin()

    print("listener ends")

def callback(data):

    print(data.twist.twist.linear.x)


# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

