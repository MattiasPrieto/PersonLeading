#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class VoiceSynthesizer:
    def __init__(self):
        rospy.init_node('voice_synthesizer', anonymous=True)
        self.soundhandle = SoundClient()
        self.prev_direction = None  # Variable para almacenar la dirección anterior
        self.is_speaking = False  # Bandera para rastrear si se está reproduciendo un mensaje
        rospy.Subscriber("/person_movement", String, self.movement_callback)
        rospy.spin()

    def movement_callback(self, data):
        movement_direction = str(data.data)
        
        # Determinar el mensaje basado en la dirección del movimiento
        if "aleja" in movement_direction.lower() and self.prev_direction != "aleja":
            message = "Please come closer"
        elif "vuelto" in movement_direction.lower() and self.prev_direction != "vuelto" or "acerca" in movement_direction.lower() and self.prev_direction != "acerca":
            message = "Let's continue"
        elif "fue" in movement_direction.lower() and self.prev_direction != "fue":
            message = "Please come back"
        else:
            message = None
        
        # Verificar si se está reproduciendo un mensaje antes de iniciar uno nuevo
        if not self.is_speaking and message:
            self.is_speaking = True  # Establecer la bandera a True
            self.speak_message(message, lang="en")

            # Actualizar la dirección anterior después de reproducir el mensaje
            self.prev_direction = "vuelto" if "vuelto" in movement_direction.lower() else "fue" if "fue" in movement_direction.lower() else "acerca" if "acerca" in movement_direction.lower() else "aleja" if "aleja" in movement_direction.lower() else None
            self.is_speaking = False  # Restablecer la bandera a False después de reproducir el mensaje

    def speak_message(self, message, lang="en"):
        # Cambiar la voz predeterminada a la voz de Festival para inglés
        self.soundhandle.voice = 'english'
        
        # Agregar una pausa entre palabras para mejorar la pronunciación
        word_delay = 0.2  # Puedes ajustar este valor según sea necesario
        if message:
            self.soundhandle.say(message, lang=lang)
            rospy.sleep(word_delay)

if __name__ == '__main__':
    try:
        VoiceSynthesizer()
    except rospy.ROSInterruptException:
        pass
