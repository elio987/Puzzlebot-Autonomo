#!/usr/bin/env python3
#Autores:
#Leonardo Gracida Munoz A01379812
#Nancy L. García Jiménez A01378043

#Importamos las librerias deseadas
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from tensorflow.keras.models import load_model
from tensorflow.keras import backend as bk
from tensorflow.config import set_visible_devices
import os, rospkg

class Imagen():
    def __init__(self):
        #Inicializamos el nodod
        rospy.init_node("signal_detector")
        #Nos suscribimos al topico de la imagen
        rospy.Subscriber("/video_source/raw",Image,self.img_callback)
        #El publisher del label de la senal deectada
        self.pub_class = rospy.Publisher("/class_det", Float32, queue_size = 1)
        #Libreria que permite ingresar al paquete de ROS
        self.rp = rospkg.RosPack()
        #Liberamos la memoria de la JETSON
        bk.clear_session()
        #Buscamos el GPU del sistema para usarlo
        set_visible_devices([],'GPU')
        #Variable en la que guardar la imagen de la camara
        self.frame = np.array([[]],dtype = "uint8")
        #Ingresamos el path del modelo a usar
        self.script_path = os.path.join(self.rp.get_path("pista_manchester"), "src", "modelos","signals_5_ligero")
        self.model = load_model(self.script_path)
        #Disccionario de las labels del modelo
        self.classDictionary = {0: "stop", 1: "fin_prob", 2: "derecha", 3: "izquierda", 4: "siga", 5: "rotonda"}
        #Declaramos los 60 mensajes por segundo.
        self.rate = rospy.Rate(60)
    def img_callback(self,msg):
        #callback del punte entre ros y cv2
        self.frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    def preprocess(self,img):
        """Funcion que preprocesa la magen para poder se ingresada a la red"""
        img = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
        img = cv.resize(img,(32,32))
        img = cv.equalizeHist(img)
        img = img/255
        img = np.expand_dims(img, axis=0)
        img = np.transpose(img, (1, 2, 0))
        img = np.expand_dims(img, axis=0)
        return img

    def main(self):
        #Clasificamos una imagen para poder iniciar el modelo
        imagen_inicial= os.path.join(self.rp.get_path("pista_manchester"), "src", "modelos","stop.png")
        imagen_inicial = cv.imread(imagen_inicial)
        imagen_inicial = self.preprocess(imagen_inicial)
        pred = self.model.predict(imagen_inicial)
        print("Listo para clasificar....")
        msg = Float32()
        while not rospy.is_shutdown():
            try:
                frame = self.frame
                #Creamos dos copias para poder filtrar rojo y verde
                frame_b = frame.copy()
                frame_r = frame.copy()
                #Pasamos la imagen a HSV
                img_hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
                #umbral de valores para el filtrado de cada color
                #Azul
                color_min_b=np.array([43,150,83])
                color_max_b=np.array([176,255,182])
                #color_min_b=np.array([47,73,73])
                #color_max_b=np.array([93,255,239])
                #Rojo
                color_min_r=np.array([140,61,135]) #tarde
                color_max_r=np.array([247,213,217])#tarde
                #color_min_r=np.array([0,90,106])
                #color_max_r=np.array([33,176,255])

                #Creamos las mascaras filtrando los colores
                mask_b=cv.inRange(img_hsv,color_min_b,color_max_b)
                mask_r=cv.inRange(img_hsv,color_min_r,color_max_r)
                #Filtarmos color correspondiente en cada frame

                frame_b[mask_b<255]=(0,0,0)
                frame_r[mask_r<255]=(0,0,0)
                """Aplicamos a la imagen filtrada una escala de grises, la binarizamos, la dilatamos
                y obtenemos los contornos, tanto para el rojo como para el azul."""
                #azul
                gray = cv.cvtColor(frame_b,cv.COLOR_BGR2GRAY)
                _,thresh = cv.threshold(gray,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
                kernel = np.ones((3,3),np.uint8)
                thresh = cv.dilate(thresh,kernel,iterations=5)
                contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
                #rojo
                gray_r = cv.cvtColor(frame_r,cv.COLOR_BGR2GRAY)
                _,thresh_r = cv.threshold(gray_r,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
                thresh_r = cv.dilate(thresh_r,kernel,iterations=7)
                contours_r, hierarchy_r = cv.findContours(thresh_r, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
                #azul
                if len(contours) > 0:
                    for cnt in contours:
                        area = cv.contourArea(cnt)
                        #Si la senal esta lo suficientemente cerca.
                        if (area > 1000)and(area < 3000):
                            #Lo ingresamos en un bounding rectangle
                            x,y,w,h = cv.boundingRect(cnt)
                            #Extraemos el ROI
                            img = frame[y:y+h,x:x+w]
                            #La preprocesamos
                            img = self.preprocess(img)
                            #Predecimos el ROI
                            predictions = self.model.predict(img)
                            #Obtenemos la maxima probabilidad
                            classIndex = predictions.argmax(axis=1)[0]
                            #Obtenemos el label de la prediccion
                            label = self.classDictionary[classIndex]
                            prob = predictions[0][classIndex]
                            #Si detectamos un stop devolvemos un 6 en lugar de un cero
                            if classIndex == 0:
                                classIndex = 6
                            #Si la prediccion tiene un mas del 90 por ciento de probabilidad, la publicamos
                            if (prob > 0.9):
                                msg.data = classIndex
                                self.pub_class.publish(msg)
                            else:
                                #Si no publicamos un cero
                                msg.data = 0
                                self.pub_class.publish(msg)
                        else:
                            #En caso de no detectar nada publicamos un cero
                            msg.data = 0
                            self.pub_class.publish(msg)
                #rojo
                if len(contours_r) > 0:
                    for cnt in contours_r:
                        area = cv.contourArea(cnt)
                        #Si la senal esta lo suficientemente cerca.
                        if (area > 1000):
                            #Lo ingresamos en un bounding rectangle
                            x,y,w,h = cv.boundingRect(cnt)
                            #Extraemos el ROI
                            img = frame[y:y+h,x:x+w]
                            #La preprocesamos
                            img = self.preprocess(img)
                            #Predecimos el ROI
                            predictions = self.model.predict(img)
                            #Obtenemos la maxima probabilidad
                            classIndex = predictions.argmax(axis=1)[0]
                            #Obtenemos el label de la prediccion
                            label = self.classDictionary[classIndex]
                            prob = predictions[0][classIndex]
                            #Si detectamos un stop devolvemos un 6 en lugar de un cero
                            if classIndex == 0:
                                classIndex = 6
                            #Si la prediccion tiene un mas del 90 por ciento de probabilidad, la publicamos
                            if (prob > 0.9):
                                msg.data = classIndex
                                self.pub_class.publish(msg)
                            else:
                                #Si no publicamos un cero
                                msg.data = 0
                                self.pub_class.publish(msg)
                        else:
                            #En caso de no detectar nada publicamos un cero
                            msg.data = 0
                            self.pub_class.publish(msg)

            except:
                #En caso de haber un fallo publicamos 0
                msg.data = 0
                self.pub_class.publish(msg)
                #print("vacio")

            self.rate.sleep()

if __name__ == "__main__":
    img = Imagen()
    img.main()
"""
                #color_min_b=np.array([59,165,84])
                #color_max_b=np.array([161,255,181])
                color_min_b=np.array([59,165,84])
                color_max_b=np.array([176,255,181])
                color_min_r=np.array([110,121,80])
                color_max_r=np.array([190,220,156])
"""
