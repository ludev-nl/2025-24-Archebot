from geometry_msgs.msg import Twist
import rospy

def drive(data) -> None:
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    twist = Twist()

    if data.data == "NOBJECT":
        print("NOBJECT")
        twist.linear.x = 0.2
    else:
        print("OBJECT")
        twist.linear.x = 0

    pub.publish(twist)
