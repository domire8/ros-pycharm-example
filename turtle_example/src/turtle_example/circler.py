import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, TeleportAbsoluteRequest


def main():
    rospy.init_node("circler")
    vel_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1000)
    client = rospy.ServiceProxy("/turtle1/teleport_absolute", TeleportAbsolute)

    srv = TeleportAbsoluteRequest()
    srv.x = 2
    srv.y = 3
    srv.theta = -45

    if client.call(srv):
        rospy.loginfo(f"Teleporting to {srv.x}, {srv.y}, {srv.theta}")
    else:
        rospy.logerr("Failed to call service")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        vel = Twist()
        vel.linear.x = 2
        vel.angular.z = 0.5
        vel_pub.publish(vel)
        rate.sleep()


if __name__ == '__main__':
    main()
