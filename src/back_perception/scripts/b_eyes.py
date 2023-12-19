#!/usr/bin/env python3
from __future__ import print_function

import os  # Agrega esta importación
import numpy as np
np.float = np.float64

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Agrega esta importación
from ultralytics import YOLO
import ros_numpy as rnp
import contextlib

class_list = [0]

class ImageConverter:
    def __init__(self, scale_factor=0.02, fov_frames_threshold=3):
        self.model = YOLO('yolov8n.pt')
        rospy.loginfo('YOLOv8 is now running...')
        
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.movement_pub = rospy.Publisher("/movement", String, queue_size=10)  # Crea el publicador para el tópico /movement
        
        self._image_data = None
        self.prev_bbox_x = None
        self.prev_bbox_y = None
        self.prev_bbox_size = None
        self.scale_factor = scale_factor
        self.frames_out_of_fov = 0
        self.fov_frames_threshold = fov_frames_threshold

    def image_callback(self, msg):
        seq = msg.header.seq
        self._image_data = rnp.numpify(msg)

        if self._image_data.shape[2] == 3 and self._image_data.dtype == np.uint8:
            self._image_data = cv2.cvtColor(self._image_data, cv2.COLOR_BGR2RGB)

        if self._image_data is not None:
            with open(os.devnull, 'w') as fnull:
                with contextlib.redirect_stdout(fnull):
                    detections = self.model.track(self._image_data, show=True, imgsz=320, persist=True, classes=class_list, conf=0.7)

            if len(detections) > 0:
                for i, d in enumerate(detections):
                    boxes_i = d.boxes.data
                    for b in boxes_i:
                        x_b, y_b, w_b, h_b = b[0:4]

                        # Calcular la posición horizontal y vertical del bounding box
                        bbox_x = x_b + w_b / 2
                        bbox_y = y_b + h_b / 2

                        # Calcular el tamaño del bounding box
                        bbox_size = w_b * h_b

                        # Determinar si la persona se acerca, se aleja, va a la derecha, a la izquierda, arriba o abajo
                        movement_direction = self.determine_movement(bbox_x, bbox_y, bbox_size)
                        if movement_direction is not None:
                            print('Detected object at x: {}, y: {}. BBox size: {:.2f}. Movement direction: {}'.format(x_b, y_b, bbox_size, movement_direction))
                            # Publicar la dirección del movimiento
                            self.movement_pub.publish(movement_direction)

                        # Actualizar la posición horizontal, vertical, el tamaño y la dirección anterior para la próxima iteración
                        self.prev_bbox_x = bbox_x
                        self.prev_bbox_y = bbox_y
                        self.prev_bbox_size = bbox_size
                        self.frames_out_of_fov = 0  # Reiniciar contador cuando se detecta a la persona

            else:
                # Incrementar el contador de frames fuera del FOV si no se detecta a la persona
                self.frames_out_of_fov += 1

                # Verificar si la persona ha estado fuera del FOV durante el umbral de frames
                if self.frames_out_of_fov >= self.fov_frames_threshold:
                    if self.prev_bbox_x is not None:
                        # Si la persona ha desaparecido, imprimir la última dirección conocida
                        if self.prev_bbox_x < self._image_data.shape[1] / 2:
                            direction = 'Se fue a la derecha'
                        else:
                            direction = 'Se fue a la izquierda'
                        print(direction)
                        # Publicar la dirección del movimiento
                        self.movement_pub.publish(direction)
                        
                    else:
                        print('Persona fuera del FOV')

    def determine_movement(self, current_bbox_x, current_bbox_y, current_bbox_size):
        # Compara la posición horizontal y vertical actual con la posición anterior y aplica un factor de escala
        if self.prev_bbox_x is not None and self.prev_bbox_y is not None and self.prev_bbox_size is not None:
            position_difference_x = current_bbox_x - self.prev_bbox_x
            position_difference_y = current_bbox_y - self.prev_bbox_y
            size_difference = current_bbox_size - self.prev_bbox_size

            if size_difference > self.scale_factor * current_bbox_size:
                return 'Se acerca'
            elif size_difference < -self.scale_factor * current_bbox_size:
                return 'Se aleja'
            elif abs(position_difference_y) > abs(position_difference_x):
                if position_difference_y > self.scale_factor * self._image_data.shape[0]:
                    return 'Se mueve hacia arriba'
                elif position_difference_y < -self.scale_factor * self._image_data.shape[0]:
                    return 'Se mueve hacia abajo'
            else:
                if position_difference_x > self.scale_factor * self._image_data.shape[1]:
                    return 'Se mueve a la izquierda'
                elif position_difference_x < -self.scale_factor * self._image_data.shape[1]:
                    return 'Se mueve a la derecha'

if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    
    # Puedes ajustar estos valores según tus necesidades
    scale_factor = 0.02
    fov_frames_threshold = 3
    image_converter = ImageConverter(scale_factor, fov_frames_threshold)
    
    rospy.spin()
