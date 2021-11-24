import dronekit as dk
import dronekit_sitl as dk_sitl
import time
from checks import *


#Controlador encargado de centralizar las diversas funciones
# del dron.
#
#Cada función deberá realizar las comprobaciones necesarias previa a su ejecución
# Si alguna fallara, no se realizaría la acción
#
#Deberá ser posible, mediante una flag, controlar si estas comprobaciones se comprueban o no


class ControladorDron:
    def __init__(self):
        print("Iniciando el controlador del dron")
        #Declarar variables que guarden el estado del dron

    #Setup del dron
    def inicia():
        print("Iniciado")

    #Despega el dron a la altura especificada
    """
    Comprobaciones:
        - Altura es válida
        - is_armable
        - Modo
        - Armado
    """
    def despega(vehicle :dk.Vehicle, altura):

        #Comprobar si la altura es válida

        pre_arm()

        arming()

        print("Despega")
        vehicle.simple_takeoff(altura)

        esperaAltura()


    #https://dronekit-python.readthedocs.io/en/latest/guide/copter/guided_mode.html
    def mueve(**kwargs):
        tiempo = kwargs.get('tiempo', None) #Tiempo a moverse a la velocidad establecida
                                            
        distancia = kwargs.get('dist', None) #Distancia a recorrer

        vel = kwargs.get('vel', None) #Controlling vehicle movement using velocity is much smoother than using position when there are likely to be many updates (for example when tracking moving objects).

        dir = kwargs.get('dir', None) #Yaw Axis -> Vehicle.attitude

