from geometry_msgs.msg import Twist
import rospy
import math

from tf.transformations import euler_from_quaternion

current_yaw = 0.0  # Global variable to store heading

# Target GPS location
GPS_LAT_TARGET = 52.165154
GPS_LONG_TARGET = 4.464589

def imu_callback(msg):
    global current_yaw
    orientation_q = msg.orientation
    quaternion = [
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w,
    ]
    (_, _, yaw) = euler_from_quaternion(quaternion)
    current_yaw = yaw  # in radians
    print(current_yaw)

def compute_distance_and_heading(current_lat, current_long, target_lat, target_long):
    """
    Computes approximate distance (in meters) and heading direction from current GPS to target.
    Assumes small distance and flat Earth for simplicity.
    """
    # Approximate conversions
    lat_to_m = 111320  # meters per degree latitude
    long_to_m = 40075000 * math.cos(math.radians(current_lat)) / 360  # meters per degree longitude

    delta_lat = target_lat - current_lat
    delta_long = target_long - current_long

    dx = delta_long * long_to_m
    dy = delta_lat * lat_to_m

    distance = math.hypot(dx, dy)
    heading_angle = math.atan2(dy, dx)

    return distance, heading_angle

def drive_towards_target(data):
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    twist = Twist()

    current_lat = data.latitude
    current_long = data.longitude

    distance, target_heading = compute_distance_and_heading(current_lat, current_long, GPS_LAT_TARGET, GPS_LONG_TARGET)

    heading_error = target_heading - current_yaw
    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))  # Normalize [-π, π]

    if distance < 0.5:
        twist.linear.x = 0
        twist.angular.z = 0
    else:
        # Angular control
        Kp_ang = 1.0
        max_ang_vel = 0.5
        angular_z = Kp_ang * heading_error
        angular_z = max(-max_ang_vel, min(max_ang_vel, angular_z))

        if abs(heading_error) < 0.05:
            angular_z = 0.0

        # Forward motion only if roughly facing the target
        if abs(heading_error) < 0.3:
            twist.linear.x = 0.2
        else:
            twist.linear.x = 0.0

        twist.angular.z = angular_z

    pub.publish(twist)

