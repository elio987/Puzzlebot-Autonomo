#!/usr/bin/env python
# coding=utf-8
#Leoanrdo Gracida Munoz A01379812
#Nancy L. Garcia Jimenez A01378043

#Importamos las librerias de numpy cv2 los mensajes y el bridge de ROS msg a cv2
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

#Creamos la clase
class Imagen():
    def __init__(self):
        #Iniciamos el nodo
        rospy.init_node("get_image")
        #Nos sucribimos al topico de la imagen dada por la camara
        rospy.Subscriber("/video_source/raw",Image,self.img_callback)
        #Creamos el publicador al topico del estado del semaforo
        self.pub_vel = rospy.Publisher("/estado_sem", Float32, queue_size = 1)
        #Creamos el publicador al topico de la imagen resultante procesada
        self.pub_img = rospy.Publisher("/img_sem", Image, queue_size = 1)
        #Iniciamos el mensaje de velocity
        self.vel = Twist()
        #Creamos la variable donde vamos a guardar la imagen obtenida de la camara
        self.frame = np.array([[]],dtype = "uint8")
        #Creamos el traductor de imagenes de ROS msg a cv2 y vicerversa
        self.bridge = CvBridge()
        #Declaramos los mensajes por segundo
        self.rate = rospy.Rate(60)

    #funciones callback para extraer los datos de los suscriptores
    def img_callback(self,data):
        #callback del punte entre ros y cv2
        frame = self.bridge.imgmsg_to_cv2(data,desired_encoding = "passthrough")
        self.frame = frame

    def main(self):
        #Declaramos el estado inicial para mostrarlo en consola
        estado = "avanza"
        estado_vel = Float32()
        estado_vel.data = 1
        while not rospy.is_shutdown():
            #rotar el frame si axis=1
            frame = np.flip(self.frame,axis=0)
            frame = self.frame
            frame_g = frame.copy()
            frame_r = frame.copy()
            frame_y = frame.copy()
            try:
                #El frame obtenido lo pasamos a HSV
                img_hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
                #umbral de valores para el filtrado de cada color

                #red
                #color_min_r=np.array([0,60,70])
                #color_max_r=np.array([70,255,255])
                color_min_r=np.array([0,70,200])
                color_max_r=np.array([30,116,255])
                #green
                color_min_g=np.array([47,88,80])
                color_max_g=np.array([90,164,255])

                #Creamos las mascaras filtrando los colores
                mask_g=cv.inRange(img_hsv,color_min_g,color_max_g)
                mask_r=cv.inRange(img_hsv,color_min_r,color_max_r)
                mask_y=cv.inRange(img_hsv,color_min_y,color_max_y)

                #Filtarmos color correspondiente en cada frame
                #green filter
                frame_g[mask_g<255]=(0,0,0)
                #red filter
                frame_r[mask_r<255]=(0,0,0)

                """Por cada color vamos a hacer el mismo procesamiento."""
                #green
                #Lo pasamos a escalas de grises la imagen filtrada solo del color verde del semaforo
                gray_g = cv.cvtColor(frame_g,cv.COLOR_BGR2GRAY)
                #Reducimos ruido
                gray_g = cv.GaussianBlur(gray_g,(5,5),0)
                #Se binariza con otsu
                _, binary_g = cv.threshold(gray_g,20,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)
                kernel = np.ones((3,3),np.uint8)
                #erosion de la imagen
                erosion_g = cv.erode(binary_g,kernel,iterations = 2)
                #Dilatacion de la imagen
                dilation_g = cv.dilate(erosion_g,kernel,iterations = 2)
                dilation_g_BGR = cv.cvtColor(dilation_g,cv.COLOR_GRAY2BGR)
                #Usamos el Blob Detector para que nos devuelva el tamano y localizacion del circulo del semaforo
                size_g,pos_g = self.filter_circle(dilation_g)
                #Obtenemos la posicion
                x_g,y_g = pos_g

                #red
                #Lo pasamos a escalas de grises la imagen filtrada solo del color rojo del semaforo
                gray_r = cv.cvtColor(frame_r,cv.COLOR_BGR2GRAY)
                #Reduccion de ruido
                gray_r = cv.GaussianBlur(gray_r,(5,5),0)
                #binarizacion con otsu
                _, binary_r = cv.threshold(gray_r,20,255,cv.THRESH_BINARY)
                #Erosion de la imagen
                erosion_r = cv.erode(binary_r,kernel,iterations = 1)
                #Dilatacion de la imagen
                dilation_r = cv.dilate(erosion_r,kernel,iterations = 5)
                dilation_r_BGR = cv.cvtColor(~dilation_r,cv.COLOR_GRAY2BGR)
                #Blob Detector para que nos devuelva el tamano y localizacion del circulo del semaforo
                size_r,pos_r = self.filter_circle(~dilation_r)
                print(size_r,pos_r)
                #Obtencion de la posicion
                x_r,y_r = pos_r

                #si el circulo es rojo y mayor a esa area cambia el estado a "detenido" y publica 0
                if size_r > 18:
                    estado = "detenido"
                    estado_vel.data = 0

                #si el circulo es verde y mayor a esa area cambia el estado a "avanza" y publica 1
                elif size_g > 8:
                    estado = "avanza"
                    estado_vel.data = 1

                #para la terminal
                print("modo: ",estado)

                #Establece el tamano de la ventana en 220x180
                smaller =cv.resize(dilation_r_BGR,(220,180),interpolation = cv.INTER_NEAREST)

                #El tamano de la ventana anterior sera la usada como puente entre ros y cv2
                img_back = self.bridge.cv2_to_imgmsg(smaller)
                img_back.encoding = "bgr8"

                #Publica en el topico /cmd_vel la velocidad
                self.pub_img.publish(img_back)

                #Publica en el topico /cmd_vel la velocidad
                self.pub_vel.publish(estado_vel)

            except:
                self.pub_vel.publish(estado_vel)

            self.rate.sleep()

    def filter_circle(self,bin_img):
        """Funcion que obteniendo un frame binarizado, detecta circulos, en base a las areas de pixeles negros conectados."""
        #Declaramos los parametros del Blob Detector
        params = cv.SimpleBlobDetector_Params()

        # Filtro por Area.
        params.filterByArea = True
        params.minArea = 50

        # Filtro por Circularidad
        params.filterByCircularity = True
        params.minCircularity = 0.4

        # Filtro por Convexividad
        params.filterByConvexity = False

        # Filter por Inercia
        params.filterByInertia = False

        # Creacion del detector de circulos utilizando los parametros anteriores
        detector = cv.SimpleBlobDetector_create(params)
        # lista de valores proporcionada por el detctor
        keypoints = list(detector.detect(bin_img))

        try:
            tam = []
            pos = []
            #Iteramos entre todos los ciruclos detectados
            for key in keypoints:
                #Extraemos su tamano y su posicion
                tam.append(key.size)
                pos.append((key.pt[0],key.pt[1]))
            #Obtenemos la posicion y tamano del mas grande
            max_size = np.max(tam)
            index_max_tam = tam.index(max_size)
            x,y = pos[index_max_tam]
            #Lo retornamos
            features = (max_size,(x,y))
        except:
            #En caso de no encontrar nada devolver una tupla de ceros
            features = (0,(0,0))
        return features

if __name__ == "__main__":
    img = Imagen()
    img.main()
