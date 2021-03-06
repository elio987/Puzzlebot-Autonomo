# Puzzlebot-Autonomo
Codigos que controlan de manera autonoma, el robot Puzzlebot de la compañia manchester Robotics en una pista determinada que se muestra a continuación.
- La pista que vamos a completar con este codigo es la siguiente:
![alt text](pista.png)
No olvides calibrar las máscaras de filtradod e colores HSV, para poder detctar el semáforo a usar como las siguientes señales de trafico:
![alt text](Traffic_Signs.png)
- Para poder iniciar ingresa los siguiente comandos al Puzzlebot por medio de ssh a la Jetson Nano que contiene:
```
rosrun (tu paquete de ROS) signal_detector.py
```
- Espera hasta que la terminal diga "Listo para clasificar"
- Despues inicial el seguidor de linea:
```
rosrun (tu paquete de ROS) seguir_sum_ver.py
```
- Inicia el detector de semáforos:
```
rosrun (tu paquete de ROS) traffic_detector.py
```
- Inicia el nodo de movimiento:
```
rosrun (tu paquete de ROS) seguidor_mov.py
```
- Ademas no olvides revisar el notebook donde entrenamos el modelo que clasfica la senales se llama:<br>
```
train_senales_final
```
- Cambia el nombre del modelo donde llamamos el modelo CNN en el signal_detector no lo olvides y si es necesario calibrar otra vez las máscaras de HSV hazlo.
- En caso de que quieras detectar y hacer algo con las señales de fin de prohibición o la de giro a la izquierda modifica el codigo detector de señales (signal_detector.py) y el de seguidor de línea (seguir_sum_ver.py) para tomarlas en cuenta, como también el de movimiento (seguidor_mov.py) para hacer un giro a la izquerda o ir más rápido.
- No olvides ingresar los codigos al src de tu Puzzlebot !!!
- Link hacia video deostrativo del funcionamiento:<br>
https://drive.google.com/file/d/1_A2JwOrbDKLIEbJYOhIJmUWR3L3Wnr7b/view?usp=sharing
- Si deseas una descripción más de tallada del proyecto puedes ir al reporte de Leonardo Gracida Muñoz:<br>
https://drive.google.com/file/d/1pX_nELC5Xx-iCu0LSS3EcKA8bQNOAuXI/view?usp=sharing
- También en el de Nancy L. García Jiménez: 
https://drive.google.com/file/d/1eYCptTdOJJOhbiPmjKPLtjCVRL1iFCM8/view?usp=sharing
