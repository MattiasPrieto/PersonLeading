#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID  # Importa GoalID
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion, Point
from std_msgs.msg import Header 

class JoyControll:
    def __init__(self):
        self.subscriber = rospy.Subscriber('/joy', Joy, self._callback)
        self.publisher = rospy.Publisher('/p3dx/cmd_vel', Twist, queue_size=10)
        self.subscriber_current_goal = rospy.Subscriber('/move_base/current_goal',PoseStamped, self.callback_current_goal)
        self.goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        self.camera_info =     rospy.Subscriber('/person_info', String, status)
        self.got_to =     rospy.Subscriber('/goal_str', String, goal)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_alfa = 0.0
        self.current_w = 00

        # Variables para guardar la posición
        self.saved_x = 0.0
        self.saved_y = 0.0
        self.saved_alfa = 0.0
        self.saved_w = 0.0

    def _callback(self, msg):
        axes = msg.axes
        buttons = msg.buttons

        adelante = axes[1]
        girar = axes[3]

        button_a = buttons[0]
        button_b = buttons[1]
        button_x = buttons[2]
        button_y = buttons[3]
        button_lb = buttons[4]
        button_rb = buttons[5]
        button_back = buttons[6]
        button_start = buttons[7]
        button_start = buttons[8]
        button_right = buttons[11]
        button_jeft = buttons[12]
        button_up = buttons[13]
        button_up = buttons[14]



        # when stop 
        if button_a == 1:
            #first save the  current goal

            self.save_goal()

            # now we cancel the nav
            self.cancel_navigation()



        #when press , the goal saved previesly is given again
        if button_b == 1:
            x = self.saved_x
            y = self.saved_y
            alfa = self.saved_alfa
            w = self.saved_w

            self.go_to_goal(x,y,alfa,w)


        # ir a la entrada
        if button_up     == 1:
            x_entrada = 0.12466902077675036
            y_entrada = -0.019672544114857332
            alfa_entrada = 0.015765711601958256
            w_entrada = 0.999875713445268
            self.go_to_goal(x_entrada,y_entrada,alfa_entrada,w_entrada)

        if button_x == 1:
            hall_x = 1.2697761124184956
            hall_y = 2.364388206728983
            hall_alfa = 0.07849068656127511  # Considerando que alfa es la rotación en z
            hall_w = 0.9969148469769821
            self.go_to_goal(hall_x, hall_y, hall_alfa, hall_w)

        

        #ir al living
        if button_y == 1:
            living_x = 0.605769076842279
            living_y = 4.427216976628204
            living_alfa = 0.47072591806072644  # Considerando que alfa es la rotación en z
            living_w = 0.8822794965689083
            self.go_to_goal(living_x, living_y, living_alfa, living_w)



        if button_lb == 1:
            cocina_x = 3.47005519266565
            cocina_y = 1.423050813329092
            cocina_alfa = 0.636320583866301  # Considerando que alfa es la rotación en z
            cocina_w = 0.7714247303191997
            self.go_to_goal(cocina_x, cocina_y, cocina_alfa, cocina_w)



        if button_rb == 1:
            pieza_x = 2.059899496019355
            pieza_y = 3.200316634677722
            pieza_alfa = 0.8426000135731917  # Considerando que alfa es la rotación en z
            pieza_w = 0.538539893718615
            self.go_to_goal(pieza_x, pieza_y, pieza_alfa, pieza_w)

        



        output = Twist()

        output.linear.x = adelante * 0.3
        output.angular.z = girar * 1

        self.publisher.publish(output)

    def cancel_navigation(self):
        cancel_msg = GoalID(stamp=rospy.Time(secs=0, nsecs=0), id='')
        cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        cancel_pub.publish(cancel_msg)

    def callback_current_goal(self,data):
        self.current_x = data.pose.position.x
        self.current_y = data.pose.position.x
        self.current_alfa = data.pose.orientation.z
        self.current_w = data.pose.orientation.w

    
    def status(self,data)
        
        if data == "cosa":
            #first save the  current goal

            self.save_goal()

            # now we cancel the nav
            self.cancel_navigation()

    def goal(self,data):

        if data == "living":
            living_x = 0.605769076842279
            living_y = 4.427216976628204
            living_alfa = 0.47072591806072644  # Considerando que alfa es la rotación en z
            living_w = 0.8822794965689083
            self.go_to_goal(living_x, living_y, living_alfa, living_w)

        if data == "hall":
            hall_x = 1.2697761124184956
            hall_y = 2.364388206728983
            hall_alfa = 0.07849068656127511  # Considerando que alfa es la rotación en z
            hall_w = 0.9969148469769821
            self.go_to_goal(hall_x, hall_y, hall_alfa, hall_w)

        if data == "cocina":
            cocina_x = 3.47005519266565
            cocina_y = 1.423050813329092
            cocina_alfa = 0.636320583866301  # Considerando que alfa es la rotación en z
            cocina_w = 0.7714247303191997
            self.go_to_goal(cocina_x, cocina_y, cocina_alfa, cocina_w)

        if data == "pieza":
            pieza_x = 2.059899496019355
            pieza_y = 3.200316634677722
            pieza_alfa = 0.8426000135731917  # Considerando que alfa es la rotación en z
            pieza_w = 0.538539893718615
            self.go_to_goal(pieza_x, pieza_y, pieza_alfa, pieza_w)






        #when press , the goal saved previesly is given again
        if data == "otra cosa":
            x = self.saved_x
            y = self.saved_y
            alfa = self.saved_alfa
            w = self.saved_w

            self.go_to_goal(x,y,alfa,w)

    def save_goal(self):
        self.saved_x = self.current_x
        self.saved_y = self.current_y
        self.saved_alfa = self.current_alfa
        self.saved_w = self.current_w
        #print(self.saved_x ,self.saved_y ,self.saved_alfa)

    def go_to_goal(self, x, y, alfa, w):
        goal_msg = MoveBaseActionGoal()
        goal_msg.goal.target_pose.header = Header(frame_id="map")
        goal_msg.goal.target_pose.pose = Pose(
            position=Point(x=x, y=y, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=alfa, w=w)
        )

        # Publica el mensaje en el tópico /move_base/goal
        self.goal_pub.publish(goal_msg)


       

    def resume_goal(self,x,y,alfa):

        goal = PoseStamped()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = alfa


        


def main():
    rospy.init_node('JoyControll')
    JoyControll()
    rospy.spin()

if __name__ == '__main__':
    main()
