#!/usr/bin/env python3
import rospy
import subprocess
from std_msgs.msg import String

class DestinationPlanner:
    def __init__(self):
        rospy.init_node('destination_planner', anonymous=True)
        self.destination_pub = rospy.Publisher("/destino", String, queue_size=10)
        self.location_list = ["kitchen", "living_room", "bedroom", "hall"]
        rospy.Subscriber("/destination_request", String, self.destination_request_callback)

    def start_camera_node(self):
        rospy.loginfo("Starting camera node...")
        subprocess.Popen(['python3', 'b_eyes.py'])  # Reemplaza 'b_eyes.py' con tu nombre de archivo

    def start_voice_synthesis_node(self):
        rospy.loginfo("Starting voice synthesis node...")
        subprocess.Popen(['python3', 'speak.py'])  # Reemplaza 'speak.py' con tu nombre de archivo

    def start_navigation_node(self):
        rospy.loginfo("Starting navigation node...")
        subprocess.Popen(['python3', 'navigation.py'])  # Reemplaza 'joy_base.py' con tu nombre de archivo

    def destination_request_callback(self, data):
        destination_request = data.data

        if destination_request == "where are we going?":
            rospy.loginfo("Destination request received.")
            
            # Puedes agregar lógica para determinar la ubicación actual del robot
            rospy.loginfo("Please enter the destination: ")
            destination = input()

            if destination in self.location_list:
                rospy.loginfo(f"Destination set to {destination}.")
                self.destination_pub.publish(destination)
                rospy.loginfo("Starting nodes...")
                self.start_camera_node()
                self.start_voice_synthesis_node()
                self.start_navigation_node()
                rospy.loginfo("Nodes started.")
            else:
                rospy.loginfo("I don't know how to get there.")

if __name__ == '__main__':
    try:
        DestinationPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
