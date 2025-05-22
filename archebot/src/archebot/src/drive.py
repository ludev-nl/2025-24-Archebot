from geometry_msgs.msg import Twist
import rospy

class Driver:
    def __init__(self):
        self.lat=0.0
        self.long=0.0
        self.object = False
        self.riding = False
        print("driver initialized")

    def update_object(self, data) -> None:
        print(data.data)
        if data.data == "NOBJECT":
            self.object = False
        else:
            self.object = True

    def update_gps(self, gps_data) -> None:
        #update long and lat
        self.long = gps_data.longitude  
        self.lat = gps_data.latitude


    def drive(self, _) -> None:
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        twist = Twist()
        if self.object:
            twist.linear.x = 0
        else:
            twist.linear.x = 0.2

        pub.publish(twist)