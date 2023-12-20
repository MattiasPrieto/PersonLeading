#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from std_msgs.msg import Header

class NavigationControll:
    def __init__(self):
        rospy.init_node('NavigationControll')
        self.got_to = rospy.Subscriber('/target_position', String, self.goal)
        self.sub_person_info = rospy.Subscriber("/person_movement", String, self.movement_callback)
        self.goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.finish_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.finish)
        
        # Variable para almacenar la ubicación objetivo

        self.destino = ""

        # Variable para almacenar la dirección anterior
        self.prev_direction = None

    def finish(self, msg):
        try:
            result = msg.status.status
            if result == 3:
                rospy.signal_shutdown("We have reach our destination")
                
        except:
            print("e")

    def movement_callback(self, data):
        movement_direction = str(data.data)
        # Determinar el mensaje basado en la dirección del movimiento
        if "aleja" in movement_direction.lower() and self.prev_direction != "aleja":
            self.cancel_navigation()
        elif "vuelto" in movement_direction.lower() or "acerca" in movement_direction.lower() and self.prev_direction != "acerca":
            try:
                self.goal(self.destino)
            except:
                rospy.sleep(1)
        elif "fue" in movement_direction.lower() and self.prev_direction != "fue":
            self.cancel_navigation()
        else:
            pass

        # Actualizar la dirección anterior
        self.prev_direction = "vuelto" if "vuelto" in movement_direction.lower() else "fue" if "fue" in movement_direction.lower() else "acerca" if "acerca" in movement_direction.lower() else "aleja" if "aleja" in movement_direction.lower() else None

    def cancel_navigation(self):
        cancel_msg = GoalID(stamp=rospy.Time(secs=0, nsecs=0), id='')   
        self.cancel_pub.publish(cancel_msg)

    def goal(self, data):

        self.destino = data

        if data.data == "living":
            living_x = 0.605769076842279
            living_y = 4.427216976628204
            living_alfa = 0.47072591806072644  # Considerando que alfa es la rotación en z
            living_w = 0.8822794965689083
            self.go_to_goal(living_x, living_y, living_alfa, living_w)

        if data.data == "center":
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

    def go_to_goal(self, x, y, alfa, w):
        goal_msg = MoveBaseActionGoal()
        goal_msg.goal.target_pose.header = Header(frame_id="map")
        goal_msg.goal.target_pose.pose = Pose(
            position=Point(x=x, y=y, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=alfa, w=w)
        )

        # Publica el mensaje en el tópico /move_base/goal
        self.goal_pub.publish(goal_msg)

def main():
    NavigationControll()
    rospy.spin()

if __name__ == '__main__':
    main()
