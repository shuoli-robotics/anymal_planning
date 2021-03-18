# !/usr/bin/env python
# import roslib
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TwistWithCovarianceStamped
from signal_logger_msgs.msg import BoolStamped
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool
import astar_LIP

def listener():
    rospy.init_node('anyplan_listener')

    rospy.Subscriber("/state_estimator/twist", TwistWithCovarianceStamped, callback)
    rospy.Subscriber("/log/loco/leftFore/isGrounded", BoolStamped, astar.set_on_ground_LF_callback)
    rospy.Subscriber("/log/loco/rightFore/isGrounded", BoolStamped, astar.set_on_ground_RF_callback)
    rospy.Subscriber("/log/loco/leftHind/isGrounded", BoolStamped, astar.set_on_ground_LH_callback)
    rospy.Subscriber("/log/loco/rightHind/isGrounded", BoolStamped, astar.set_on_ground_RH_callback)
    rospy.Subscriber("/log/loco/leftFore/positionWorldToEEOriginInWorldFrame", Vector3Stamped, astar.set_EE_LF_callback)
    rospy.Subscriber("/log/loco/rightFore/positionWorldToEEOriginInWorldFrame", Vector3Stamped, astar.set_EE_RF_callback)
    rospy.Subscriber("/log/loco/leftHind/positionWorldToEEOriginInWorldFrame", Vector3Stamped, astar.set_EE_LH_callback)
    rospy.Subscriber("/log/loco/rightHind/positionWorldToEEOriginInWorldFrame", Vector3Stamped, astar.set_EE_RH_callback)
    rospy.Subscriber("calc_zmp_cmd", Bool,astar.calc_zmp_callback)

    rospy.spin()

    print("listener ends")

def callback(data):
    pass
    # print(data.twist.twist.linear.x)


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
    zmp_0 = (2.0,1.0,0.)
    zmp_f = (9.0,9,0)
    astar = astar_LIP.AnymalAStar(zmp_0,zmp_f)
    listener()

    # astar.run()

    # try:
    #     listener()
    # except rospy.ROSInterruptException:
    #     pass

