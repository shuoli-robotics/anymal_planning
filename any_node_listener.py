# !/usr/bin/env python
# import roslib
import rospy
import astar_ros

if __name__ == '__main__':
    zmp_0 = (2.0,1.0,0.)
    zmp_f = (5.0,5,0)
    astar = astar_ros.AnymalAStarRos(zmp_0,zmp_f)
    wait_time = 1
    r_wait_rate = rospy.Rate(1/wait_time)
    r_wait_rate.sleep()
    astar.test_calc_lf_ee_traj_callback()

    r = rospy.Rate(200)

    while not rospy.is_shutdown():
        astar.publish_whole_body_trajectory_reference()
        r.sleep()



