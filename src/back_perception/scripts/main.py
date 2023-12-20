#!/usr/bin/env python3
import rospy
import os
import subprocess
from std_msgs.msg import String

path = os.path.dirname(os.path.abspath(__file__))
import speech_recognition as sr
from sound_play.libsoundplay import SoundClient

class DestinationPlanner:
    def __init__(self):
        rospy.init_node('destination_planner', anonymous=True)
        self.destination_pub = rospy.Publisher("/target_position", String, queue_size=10)
        self.location_list = ["kitchen", "living_room", "bedroom", "center"]
        self.soundhandle = SoundClient()

    def start_voice_synthesis_node(self):
        rospy.loginfo("Starting voice synthesis node...")
        subprocess.Popen(['python3', path + '/speak.py'])  

    def start_camera_node(self):
        rospy.loginfo("Starting camera node...")
        subprocess.Popen(['python3', path + '/b_eyes.py'])  

    def start_navigation_node(self):
        rospy.loginfo("Starting navigation node...")
        subprocess.Popen(['python3', path + '/navigation.py'])  
    
    def speak_message(self, message, lang="en"):
        # Cambiar la voz predeterminada a la voz de Festival para inglés
        self.soundhandle.voice = 'english'
        
        # Agregar una pausa entre palabras para mejorar la pronunciación
        word_delay = 0.2  # Puedes ajustar este valor según sea necesario
        if message:
            self.soundhandle.say(message, lang=lang)
            rospy.sleep(word_delay)

        
    def destination_request(self):
        recognizer = sr.Recognizer()
        self.speak_message("Where are we going?")

        try:
            with sr.Microphone() as source:
                
                print("Where are we going?")
                recognizer.adjust_for_ambient_noise(source, duration=1)
                recognizer.energy_threshold = 4000  
                audio = recognizer.listen(source)
            if audio:
                print("Wait")
                command = recognizer.recognize_google(audio, language="en-EN")
                print("Destination: {}".format(command))
            else:
                print("Nothing.")

        except sr.UnknownValueError:
            print("Unable to process audio.")
        except sr.RequestError as e:
            print("Error: {}".format(e))
        except Exception as ex:
            print("Unexpected Error: {}".format(ex))

   
        destination = command.lower()

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
