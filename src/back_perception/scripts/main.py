#!/usr/bin/env python3
import rospy
import os
import subprocess
from std_msgs.msg import String

path = os.path.dirname(os.path.abspath(__file__))
import speech_recognition as sr



class DestinationPlanner:
    def __init__(self):
        rospy.init_node('destination_planner', anonymous=True)
        self.destination_pub = rospy.Publisher("/target_position", String, queue_size=10)
        self.location_list = ["kitchen", "living_room", "bedroom", "hall"]

    def start_voice_synthesis_node(self):
        rospy.loginfo("Starting voice synthesis node...")
        subprocess.Popen(['python3', path + '/speak.py'])  

    def start_camera_node(self):
        rospy.loginfo("Starting camera node...")
        subprocess.Popen(['python3', path + '/b_eyes.py'])  

    def start_navigation_node(self):
        rospy.loginfo("Starting navigation node...")
        subprocess.Popen(['python3', path + '/navigation.py'])  

        
    def destination_request(self):
        recognizer = sr.Recognizer()

        try:
            with sr.Microphone() as source:
                print("Di algo...")
                recognizer.adjust_for_ambient_noise(source, duration=1)
                recognizer.energy_threshold = 4000  # Ajusta la sensibilidad según sea necesario
                audio = recognizer.listen(source)

            if audio:
                print("Reconociendo...")
                command = recognizer.recognize_google(audio, language="en-EN")
                print("Texto reconocido: {}".format(command))
            else:
                print("No se detectó audio.")

        except sr.UnknownValueError:
            print("No se pudo entender el audio")
        except sr.RequestError as e:
            print("Error en la solicitud al servicio de reconocimiento de voz: {}".format(e))
        except Exception as ex:
            print("Error inesperado: {}".format(ex))

   
        destination = command
        if destination in self.location_list:
            rospy.loginfo(f"Destination set to {destination}.")
            rospy.loginfo("Starting nodes...")
            self.start_camera_node()
            self.start_voice_synthesis_node()
            self.start_navigation_node()
            rospy.loginfo("Nodes started.")
            rospy.sleep(1)
            self.destination_pub.publish(destination)
            rospy.loginfo("Given destination.")
        else:
            rospy.loginfo("I don't know how to get there.")
            return DestinationPlanner.destination_request(self)

if __name__ == '__main__':
    try:
        planner = DestinationPlanner()
        planner.destination_request()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
