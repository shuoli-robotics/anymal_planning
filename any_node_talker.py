# !/usr/bin/env python
# import roslib
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistWithCovarianceStamped


def talker():
    pub = rospy.Publisher('calc_zmp_cmd', Bool, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        calc_zmp_cmd = True
        pub.publish(calc_zmp_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

