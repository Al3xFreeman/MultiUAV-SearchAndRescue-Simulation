import dronekit as dk
import dronekit_sitl as dk_sitl
import argparse

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
        
        parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
        parser.add_argument('--connect',
                            help="Vehicle connection target string. If not specified, SITL automatically started and used.")
        args = parser.parse_args()

        connection_string = args.connect
        sitl = None

        # Start SITL if no connection string specified
        if not connection_string:
            import dronekit_sitl
            sitl = dronekit_sitl.start_default()
            connection_string = sitl.connection_string()

        # Connect to the Vehicle
        print('Connecting to vehicle on: %s' % connection_string)
        self.vehicle = dk.connect(connection_string, wait_ready=True)



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
    def despega(self, vehicle :dk.Vehicle, altura):

        if(altura < 0):
            print("Error: Altura ha de ser un valor positivo")
            return

        pre_arm(vehicle)

        arming(vehicle)

        print("Despega")
        vehicle.simple_takeoff(altura)

        esperaAltura(vehicle, altura)


    #https://dronekit-python.readthedocs.io/en/latest/guide/copter/guided_mode.html
    def mueve(**kwargs):
        tiempo = kwargs.get('tiempo', None) #Tiempo a moverse a la velocidad establecida
                                            
        distancia = kwargs.get('dist', None) #Distancia a recorrer

        vel = kwargs.get('vel', None) #Controlling vehicle movement using velocity is much smoother than using position when there are likely to be many updates (for example when tracking moving objects).

        dir = kwargs.get('dir', None) #Yaw Axis -> Vehicle.attitude

