#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from std_msgs.msg import Header

class NavigationControll:
    def __init__(self):
        rospy.init_node('NavigationControll')
        self.got_to = rospy.Subscriber('/destino', String, self.goal)
        self.sub_person_info = rospy.Subscriber("/person_movement", String, self.movement_callback)
        self.subscriber_current_goal = rospy.Subscriber('/move_base/current_goal', PoseStamped, self.callback_current_goal)
        self.goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_alfa = 0.0
        self.current_w = 0.0

        # Variables para guardar la posición
        self.saved_x = 0.0
        self.saved_y = 0.0
        self.saved_alfa = 0.0
        self.saved_w = 0.0

        # Variable para almacenar la dirección anterior
        self.prev_direction = None

    def movement_callback(self, data):
        movement_direction = str(data.data)

        # Determinar el mensaje basado en la dirección del movimiento
        if "aleja" in movement_direction.lower() and self.prev_direction != "aleja":
            self.cancel_navigation()
        elif "vuelto" in movement_direction.lower() or "acerca" in movement_direction.lower() and self.prev_direction != "acerca":
            self.resume_navigation(self.current_x, self.current_y, self.current_alfa)
        elif "fue" in movement_direction.lower() and self.prev_direction != "fue":
            self.cancel_navigation()
        else:
            pass

        # Actualizar la dirección anterior
        self.prev_direction = "vuelto" if "vuelto" in movement_direction.lower() else "fue" if "fue" in movement_direction.lower() else "acerca" if "acerca" in movement_direction.lower() else "aleja" if "aleja" in movement_direction.lower() else None

    def cancel_navigation(self):
        cancel_msg = GoalID(stamp=rospy.Time(secs=0, nsecs=0), id='')
        cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        cancel_pub.publish(cancel_msg)

    def callback_current_goal(self, data):
        self.current_x = data.pose.position.x
        self.current_y = data.pose.position.y  # Corregido: debería ser la coordenada y
        self.current_alfa = data.pose.orientation.z
        self.current_w = data.pose.orientation.w

    def goal(self, data):
        if data.data == "living":
            living_x = 0.605769076842279
            living_y = 4.427216976628204
            living_alfa = 0.47072591806072644  # Considerando que alfa es la rotación en z
            living_w = 0.8822794965689083
            self.go_to_goal(living_x, living_y, living_alfa, living_w)

        if data.data == "hall":
            hall_x = 1.2697761124184956
            hall_y = 2.364388206728983
            hall_alfa = 0.07849068656127511  # Considerando que alfa es la rotación en z
            hall_w = 0.9969148469769821
            self.go_to_goal(hall_x, hall_y, hall_alfa, hall_w)

        if data.data == "kitchen":
            cocina_x = 3.47005519266565
            cocina_y = 1.423050813329092
            cocina_alfa = 0.636320583866301  # Considerando que alfa es la rotación en z
            cocina_w = 0.7714247303191997
            self.go_to_goal(cocina_x, cocina_y, cocina_alfa, cocina_w)

        if data.data == "bedroom":
            pieza_x = 2.059899496019355
            pieza_y = 3.200316634677722
            pieza_alfa = 0.8426000135731917  # Considerando que alfa es la rotación en z
            pieza_w = 0.538539893718615
            self.go_to_goal(pieza_x, pieza_y, pieza_alfa, pieza_w)

        if data.data == "otra cosa":
            x = self.saved_x
            y = self.saved_y
            alfa = self.saved_alfa
            w = self.saved_w
            self.go_to_goal(x, y, alfa, w)

    def save_goal(self):
        self.saved_x = self.current_x
        self.saved_y = self.current_y
        self.saved_alfa = self.current_alfa
        self.saved_w = self.current_w

    def go_to_goal(self, x, y, alfa, w):
        goal_msg = MoveBaseActionGoal()
        goal_msg.goal.target_pose.header = Header(frame_id="map")
        goal_msg.goal.target_pose.pose = Pose(
            position=Point(x=x, y=y, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=alfa, w=w)
        )

        # Publica el mensaje en el tópico /move_base/goal
        self.goal_pub.publish(goal_msg)

    def resume_navigation(self, x, y, alfa):
        goal = PoseStamped()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = alfa

        # Publica el mensaje en el tópico /move_base/goal
        self.goal_pub.publish(goal)

        # Añade una pausa para dar tiempo al sistema para procesar la solicitud
        rospy.sleep(1)

def main():
    NavigationControll()
    rospy.spin()

if __name__ == '__main__':
    main()
