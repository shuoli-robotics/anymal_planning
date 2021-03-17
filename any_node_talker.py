# !/usr/bin/env python
# import roslib
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TwistWithCovarianceStamped


def talker():
    pub = rospy.Publisher('reference_trajectories', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.01) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

