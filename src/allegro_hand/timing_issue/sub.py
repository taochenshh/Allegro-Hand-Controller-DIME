import rospy
from sensor_msgs.msg import JointState
import time
import threading
from copy import deepcopy
class Subscriber:
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        # rospy.Subscriber("/allegroHand/joint_states", JointState, self.callback, queue_size=1)
        rospy.Subscriber("chatter", JointState, self.callback, queue_size=1)
        self.data = None
        self.lock = threading.RLock()
        time.sleep(1)

    def callback(self, data):
        with self.lock:
             self.data = deepcopy(data)
            #  print(self.data.header.stamp.to_sec())

    def get_time_stamp(self):
        with self.lock:
            ts = self.data.header.stamp.to_sec()
        return ts


if __name__ == '__main__':
    listener = Subscriber()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        print(listener.get_time_stamp())
        rate.sleep()

