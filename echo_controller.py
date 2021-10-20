import rospy
from geometry_msgs.msg import Twist

cmd_vel = Twist()

def callback(msg):
    global cmd_vel
    cmd_vel = msg

def echo_controller():
    pub = rospy.Publisher('vel', Twist, queue_size=0)
    sub = rospy.Subscriber('cmd_vel', Twist, callback)
    rospy.init_node('echo_controller', anonymous=False)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        pub.publish(cmd_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        echo_controller()
    except rospy.ROSInterruptException:
        pass
