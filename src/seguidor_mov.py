#!/usr/bin/env python
#Autores:
#Leonardo Gracida Munoz A01379812
#Nancy L. García Jiménez A01378043
#Importamos las librerias de rospy y numpy
import rospy
import numpy as np
#Importamos los mensajes de ROS necesarios
from std_msgs.msg import Float32,Bool
from geometry_msgs.msg import Pose2D,Twist

class SeguidorMov():
    def __init__(self):
        #Iniciamos el nodo
        rospy.init_node("seguidor_mov")
        self.err_line = 0
        self.edo_giro = 0
        self.edo_sem = 0
        #Creamos los subscribers
        rospy.Subscriber("/err_line",Float32,self.err_line_callback)
        rospy.Subscriber("/edo_giro",Float32,self.edo_giro_callback)
        rospy.Subscriber("/estado_sem",Float32,self.edo_sem_callback)
        #Creamos el publisher para poder mover el puzzlebot
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        #Declaramos los mensajes por segundo
        self.rate = rospy.Rate(100)
        #Iniciamos un mensaje tipo Twist
        self.robot_cmd = Twist()
        #Cuando se acabe el codigo o lo tiremos llamemos esa funcion
        rospy.on_shutdown(self.end_callback)
    #funciones callback para extraer los datos de los suscriptores
    def err_line_callback(self,data):
        self.err_line = data.data

    #Numero que controla la velocidad lineal y angular del robot dependiendo de la senal detectada.
    def edo_giro_callback(self,data):
        self.edo_giro = data.data
    #Funcion que va a parar el robot cuando sea llamada
    def end_callback(self):
    	self.robot_cmd.linear.x = 0.0
    	self.robot_cmd.angular.z = 0.0
    	self.pub.publish(self.robot_cmd)
    #Callback que guarda la senal de control del semaforo
    def edo_sem_callback(self,data):
        self.edo_sem = data.data
    def main(self):
        while not rospy.is_shutdown():
            #Declaramos la ganancia proporcional
            kp = 0.0012
            lim_vel_ang = 0.2
            vel_lin = 0.07
            #Error de angulo
            err_line = self.err_line
            #aplicacion de control proporcional al angulo
            proporcional = err_line * kp
            #Filtro de saturacion
            if proporcional > lim_vel_ang:
                proporcional = lim_vel_ang
            elif proporcional < -1*lim_vel_ang:
                proporcional = -1*lim_vel_ang
            #Si detectamos un go turn right giramos a la derecha
            if self.edo_giro == 2:
                proporcional = -0.05
                vel_lin = 0.08
            #Si obtenemos un stop para el robot
            if self.edo_giro == 3:
                proporcional = 0
                vel_lin = 0

            #Publicamos la velocida para mover el robot
            self.robot_cmd.angular.z = proporcional*self.edo_sem
            #Aplicamos la variables de control de velocidad lineal y el aumento de velocidad
            self.robot_cmd.linear.x = vel_lin*self.edo_sem
            #Publicamos la velocidad
            self.pub.publish(self.robot_cmd)
            #Declaramos el sleep para asegurar los mensaje por segundo.
            self.rate.sleep()
#Si el programa es corrido directamente
if __name__ == "__main__":
    #Creamos la clase
    seguidor_con = SeguidorMov()
    #Iniciamos el programa
    seguidor_con.main()
