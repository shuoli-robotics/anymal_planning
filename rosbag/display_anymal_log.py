import rosbag
import rospy
import matplotlib.pyplot as plt

bag_file = 'subset.bag'
bag = rosbag.Bag(bag_file)

t_com = []
x_com = []
y_com = []
z_com = []
for topic, msg, t in bag.read_messages(topics='/log/loco/whole_body/positionWorldToComInWorldFrame'):
    t_com.append(rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs).to_sec())
    x_com.append(msg.vector.x)
    y_com.append(msg.vector.y)
    z_com.append(msg.vector.z)

t_desired = []
x_com_desired = []
y_com_desired = []
z_com_desired = []

x_lf_desired = []
y_lf_desired = []
z_lf_desired = []
lf_is_contact_desired = []

x_rf_desired = []
y_rf_desired = []
z_rf_desired = []
rf_is_contact_desired = []

x_lh_desired = []
y_lh_desired = []
z_lh_desired = []
lh_is_contact_desired = []

x_rh_desired = []
y_rh_desired = []
z_rh_desired = []
rh_is_contact_desired = []


for topic, msg, t in bag.read_messages(topics='/whole_body_control_reference'):
    t_desired.append(rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs).to_sec())
    x_com_desired.append(msg.com_position.x)
    y_com_desired.append(msg.com_position.y)
    z_com_desired.append(msg.com_position.z)

    x_lf_desired.append(msg.foot1_position.x)
    y_lf_desired.append(msg.foot1_position.y)
    z_lf_desired.append(msg.foot1_position.z)
    lf_is_contact_desired.append(msg.foot1_isContact)

    x_rf_desired.append(msg.foot2_position.x)
    y_rf_desired.append(msg.foot2_position.y)
    z_rf_desired.append(msg.foot2_position.z)
    rf_is_contact_desired.append(msg.foot2_isContact)

    x_lh_desired.append(msg.foot3_position.x)
    y_lh_desired.append(msg.foot3_position.y)
    z_lh_desired.append(msg.foot3_position.z)
    lh_is_contact_desired.append(msg.foot3_isContact)

    x_rh_desired.append(msg.foot4_position.x)
    y_rh_desired.append(msg.foot4_position.y)
    z_rh_desired.append(msg.foot4_position.z)
    rh_is_contact_desired.append(msg.foot4_isContact)


t_lf_measured = []
x_lf_measured = []
y_lf_measured = []
z_lf_measured = []



t_rf_measured = []
x_rf_measured = []
y_rf_measured = []
z_rf_measured = []

t_lh_measured = []
x_lh_measured = []
y_lh_measured = []
z_lh_measured = []

t_rh_measured = []
x_rh_measured = []
y_rh_measured = []
z_rh_measured = []

t_lf_is_contact = []
lf_is_contact = []

t_rf_is_contact = []
rf_is_contact = []

t_lh_is_contact = []
lh_is_contact = []

t_rh_is_contact = []
rh_is_contact = []

for topic, msg, t in bag.read_messages(topics='/log/loco/leftFore/positionWorldToEEOriginInWorldFrame'):
    t_lf_measured.append(rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs).to_sec())
    x_lf_measured.append(msg.vector.x)
    y_lf_measured.append(msg.vector.y)
    z_lf_measured.append(msg.vector.z)

for topic, msg, t in bag.read_messages(topics='/log/loco/rightFore/positionWorldToEEOriginInWorldFrame'):
    t_rf_measured.append(rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs).to_sec())
    x_rf_measured.append(msg.vector.x)
    y_rf_measured.append(msg.vector.y)
    z_rf_measured.append(msg.vector.z)

for topic, msg, t in bag.read_messages(topics='/log/loco/leftHind/positionWorldToEEOriginInWorldFrame'):
    t_lh_measured.append(rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs).to_sec())
    x_lh_measured.append(msg.vector.x)
    y_lh_measured.append(msg.vector.y)
    z_lh_measured.append(msg.vector.z)

for topic, msg, t in bag.read_messages(topics='/log/loco/rightHind/positionWorldToEEOriginInWorldFrame'):
    t_rh_measured.append(rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs).to_sec())
    x_rh_measured.append(msg.vector.x)
    y_rh_measured.append(msg.vector.y)
    z_rh_measured.append(msg.vector.z)

for topic, msg, t in bag.read_messages(topics='/log/loco/leftFore/isGrounded'):
    t_lf_is_contact.append(rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs).to_sec())
    lf_is_contact.append(msg.value)

for topic, msg, t in bag.read_messages(topics='/log/loco/rightFore/isGrounded'):
    t_rf_is_contact.append(rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs).to_sec())
    rf_is_contact.append(msg.value)

for topic, msg, t in bag.read_messages(topics='/log/loco/leftHind/isGrounded'):
    t_lh_is_contact.append(rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs).to_sec())
    lh_is_contact.append(msg.value)

for topic, msg, t in bag.read_messages(topics='/log/loco/rightHind/isGrounded'):
    t_rh_is_contact.append(rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs).to_sec())
    rh_is_contact.append(msg.value)

fig,ax_CoM = plt.subplots(3,1)
ax_CoM[0].plot(t_com,x_com,label='CoM real')
ax_CoM[0].plot(t_desired,x_com_desired,label='CoM ref')
ax_CoM[0].set_ylabel('x[m]')

ax_CoM[1].plot(t_com,y_com)
ax_CoM[1].plot(t_desired,y_com_desired)
ax_CoM[1].set_ylabel('y[m]')

ax_CoM[2].plot(t_com,z_com)
ax_CoM[2].plot(t_desired,z_com_desired)
ax_CoM[2].set_ylabel('z[m]')
ax_CoM[2].set_xlabel('time[s]')
ax_CoM[0].legend(loc='best')


fig1,ax_foot = plt.subplots(1,3)
ax_foot[0].plot(t_desired,x_lf_desired,'r:',label='lf_ref')
ax_foot[0].plot(t_lf_measured,x_lf_measured,'r-.',label='lf_measured')
ax_foot[0].plot(t_desired,x_rh_desired,'g:',label='rh_ref')
ax_foot[0].plot(t_lf_measured,x_rh_measured,'g-.',label='rh_measured')
ax_foot[0].plot(t_desired,lf_is_contact_desired,color=(0.3,0.3,0.3,0.3),label='contact')
ax_foot[0].set_ylabel('x[m]')
ax_foot[0].set_xlabel('time[s]')

ax_foot[1].plot(t_desired,y_lf_desired,'r:')
ax_foot[1].plot(t_lf_measured,y_lf_measured,'r-.')
ax_foot[1].plot(t_desired,y_rh_desired,'g:')
ax_foot[1].plot(t_lf_measured,y_rh_measured,'g-.')
ax_foot[1].plot(t_desired,lf_is_contact_desired,color=(0.3,0.3,0.3,0.3))
ax_foot[1].set_ylabel('y[m]')
ax_foot[1].set_xlabel('time[s]')

ax_foot[2].plot(t_desired,z_lf_desired,'r:')
ax_foot[2].plot(t_lf_measured,z_lf_measured,'r-.')
ax_foot[2].plot(t_desired,z_rh_desired,'g:')
ax_foot[2].plot(t_lf_measured,z_rh_measured,'g-.')
ax_foot[2].plot(t_desired,lf_is_contact_desired,color=(0.3,0.3,0.3,0.3))
ax_foot[2].set_ylabel('z[m]')
ax_foot[2].set_xlabel('time[s]')
ax_foot[0].legend(loc='best')

fig2,ax_foot_1 = plt.subplots(1,3)
ax_foot_1[0].plot(t_desired,x_rf_desired,'y:',label='rf_ref')
ax_foot_1[0].plot(t_rf_measured,x_rf_measured,'y-.',label='rf_measured')
ax_foot_1[0].plot(t_desired,x_lh_desired,'b:',label='lh_ref')
ax_foot_1[0].plot(t_lh_measured,x_lh_measured,'b-.',label='lh_measured')
ax_foot_1[0].plot(t_desired,rf_is_contact_desired,color=(0.3,0.3,0.3,0.3),label='contact')
ax_foot_1[0].set_ylabel('x[m]')
ax_foot_1[0].set_xlabel('time[s]')

ax_foot_1[1].plot(t_desired,y_rf_desired,'y:')
ax_foot_1[1].plot(t_rf_measured,y_rf_measured,'y-.')
ax_foot_1[1].plot(t_desired,y_lh_desired,'b:')
ax_foot_1[1].plot(t_lh_measured,y_lh_measured,'b-.')
ax_foot_1[1].plot(t_desired,rf_is_contact_desired,color=(0.3,0.3,0.3,0.3))
ax_foot_1[1].set_ylabel('y[m]')
ax_foot_1[1].set_xlabel('time[s]')

ax_foot_1[2].plot(t_desired,z_rf_desired,'y:')
ax_foot_1[2].plot(t_rf_measured,z_rf_measured,'y-.')
ax_foot_1[2].plot(t_desired,z_lh_desired,'b:')
ax_foot_1[2].plot(t_lh_measured,z_lh_measured,'b-.')
ax_foot_1[2].plot(t_desired,rf_is_contact_desired,color=(0.3,0.3,0.3,0.3))
ax_foot_1[2].set_ylabel('z[m]')
ax_foot_1[2].set_xlabel('time[s]')
ax_foot_1[0].legend(loc='best')

plt.show()

test = 1