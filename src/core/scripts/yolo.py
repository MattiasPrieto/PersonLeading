#!/usr/bin/env python3

#Std Libs
import numpy as np
import math
np.float = np.float64

#Ros Python Libs
import rospy
import roslib
import ros_numpy as rnp

#ROS msgs
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

#Image Processing
import cv2
from ultralytics import YOLO

class_list = [0]

class Nodo_test():
    def __init__(self):
        self.model = YOLO('yolov8n.pt')
        rospy.loginfo('YOLOv8 is now running...')
        self.sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.callback)
        # self.sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback)
        self.pub = rospy.Publisher('object_info', String, queue_size=10)
        #self.person_poses = rospy.Publisher('person_pose', PoseStamped, queue_size=10)
        #self.marker_pub = rospy.Publisher("/visualization_markers", MarkerArray, queue_size = 2)
        self.lista_posiciones = []
        self._points_data = None
        self._image_data = None
        rospy.init_node('Nodo_test', anonymous=True)
        #self.marker_array_1 = MarkerArray()

    def callback(self,msg):
        seq = msg.header.seq
        try:
            self._points_data = rnp.numpify(msg)
            image_data = self._points_data['rgb'].view(
            (np.uint8, 4))[..., [0, 1, 2]]
            self._image_data = np.ascontiguousarray(image_data)
            # cv_image = self.cv.imgmsg_to_cv2(msg,'bgr8')
            # res = model(cv_image, show=True, conf=0.8)
            if not self._image_data is None:
                detections = self.model.track(self._image_data, show=True, persist=True, classes = class_list, conf=0.8)
            marker_array = MarkerArray()
            if len(detections)>0:
                for i, d in enumerate(detections):
                    boxes_i = d.boxes.data
                    for b in boxes_i:
                        x_b,y_b,w_b,h_b = b[0:4]
                        x = self._points_data['x'][int(y_b),int(x_b)]
                        print('x: '+str(x))
                        y = self._points_data['y'][int(y_b),int(x_b)]
                        z = self._points_data['z'][int(y_b),int(x_b)]
                        n_class = d.boxes.cls.cpu().numpy().astype(int)+1
                        
                        # ARREGLAR QUE ESTÁ TIRANDO LA MISMA UBICACIÓN PARA TODOS LOS OBJETOSOSSSS
                        #obj, cat = find_object_by_class(data_nom,n_class[0])
                        if n_class is not None:
                            info_objeto = {
                                "ID": i,
                                "class": int(n_class[0]),
                                "x" : x.item(),
                                "y" : y.item(),
                                "z" : z.item()
                            }
                        self.lista_posiciones.append(info_objeto)
                    
                self.pub.publish(json.dumps(self.lista_posiciones))     # se guarda como json.dumps, se lee como json.loads
                self.lista_posiciones = []

            #print(cv_image.shape)
        except cv_bridge.CvBridgeError as e:
            print(e)
        cv2.putText(self._image_data,text = str(seq),org = (50,50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color = (255,0,0), thickness=2)
        #cv2.imshow('hola',cv_image)
        cv2.waitKey(3)

if __name__=='__main__':
   # rospy.init_node('yolo')
    test = Nodo_test()
    rospy.spin()

# Json para escribir ubicaciones de objetos