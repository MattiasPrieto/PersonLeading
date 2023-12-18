#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID  # Importa GoalID

class JoyControll:
    def __init__(self):
        self.subscriber = rospy.Subscriber('/joy', Joy, self._callback)
        self.publisher = rospy.Publisher('/p3dx/cmd_vel', Twist, queue_size=10)

    def _callback(self, msg):
        axes = msg.axes
        buttons = msg.buttons

        adelante = axes[1]
        girar = axes[3]

        button_a = buttons[0]
        button_b = buttons[1]
        button_x = buttons[2]
        button_y = buttons[3]

        if button_a == 1:
            self.cancel_navigation()

        output = Twist()

        output.linear.x = adelante * 0.3
        output.angular.z = girar * 1

        self.publisher.publish(output)

    def cancel_navigation(self):
        cancel_msg = GoalID(stamp=rospy.Time(secs=0, nsecs=0), id='')
        cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        cancel_pub.publish(cancel_msg)

def main():
    rospy.init_node('JoyControll')
    JoyControll()
    rospy.spin()

if __name__ == '__main__':
    main()
