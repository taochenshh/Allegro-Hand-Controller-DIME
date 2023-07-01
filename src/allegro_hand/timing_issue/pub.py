import rospy
from sensor_msgs.msg import JointState

def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('chatter', JointState, queue_size=10)

    rate = rospy.Rate(300) # 300hz
    js = JointState()
    js.header.stamp = rospy.Time.now()
    # when N<=14, it seems to be fine, when N>=15, the subscriber prints duplicated time stamps
    N = 16
    js.velocity = [1.5] * N
    js.effort = [1.5] * N
    js.position = [1.5] * N
    js.name = ['link_1'] * N
    while not rospy.is_shutdown():
        js.header.stamp = rospy.Time.now()
        # rospy.loginfo(hello_str)
        pub.publish(js)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
