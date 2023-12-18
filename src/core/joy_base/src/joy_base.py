#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyControll:
    def __init__(self):
        self.subscriber = rospy.Subscriber('/joy', Joy, self._callback)
        self.publisher = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)

    def _callback(self, msg):
        axes = msg.axes
        buttons = msg.buttons

        adelante = axes[1]
        girar = axes[3]

        button_a = buttons[0]
        button_b = buttons[1]
        button_x = buttons[2]
        button_y = buttons[3]

        output = Twist()

        output.linear.x = adelante * 0.3
        output.angular.z = girar * 1

        self.publisher.publish(output)

def main():
    rospy.init_node('JoyControll')
    JoyControll()
    rospy.spin()

if __name__ == '__main__':
    main()
