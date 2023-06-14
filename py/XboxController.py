import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class XBoxController:
    def __init__(self):
        self.linear = 1
        self.angular = 2

        rospy.init_node("xbox_controller")

        self.l_scale = rospy.get_param("~scale_linear", 1.0)
        self.a_scale = rospy.get_param("~scale_angular", 1.0)

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joyCallback, queue_size=10)

    def joyCallback(self, joy):
        twist = Twist()
        twist.angular.z = self.a_scale * joy.axes[self.angular]
        twist.linear.x = self.l_scale * joy.axes[self.linear]
        self.vel_pub.publish(twist)


if __name__ == "__main__":
    xbox_controller = XBoxController()
    rospy.spin()
