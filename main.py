from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import numpy as np

# CONSTANTES
MAP_SIZE = 100
RESOLUTION = 0.1
MAP_CENTER_CELL = MAP_SIZE // 2

MAP_CELL_UNKNOWN = -1
MAP_CELL_FREE = 0
MAP_CELL_OBSTACLE = 10000

SPEED = 7

ESTADO_EXPLORACION = 1000
ESTADO_APROXIMACION = 2000
ESTADO_AGARRE = 3000
ESTADO_PLANIFICACION = 4000
ESTADO_RETORNO = 5000
ESTADO_SOLTADO = 6000
ESTADO_FINALIZADO = 8000

# MAPA MATRIZ
mapa_matriz = np.full((MAP_SIZE, MAP_SIZE), MAP_CELL_UNKNOWN, dtype=np.int32)
mapa_matriz[MAP_CENTER_CELL, MAP_CENTER_CELL] = MAP_CELL_FREE

# Representacion 10x10 para ejemplificar nada más

# -1 -1 -1 -1 -1 -1 -1 -1 -1 -1
# -1 -1 -1 -1 -1 -1 -1 -1 -1 -1
# -1 -1 -1 -1 -1 -1 -1 -1 -1 -1
# -1 -1 -1 -1 -1 -1 -1 -1 -1 -1
# -1 -1 -1 -1 -1 -1 -1 -1 -1 -1
# -1 -1 -1 -1 -1 -1 -1 -1 -1 -1
# -1 -1 -1 -1 -1 -1 -1 -1 -1 -1
# -1 -1 -1 -1 -1 -1 -1 -1 -1 -1
# -1 -1 -1 -1 -1 -1 -1 -1 -1 -1
# -1 -1 -1 -1 -1 -1 -1 -1 -1 -1

def posicion_real_a_matriz(x_real, y_real):
    col_relativa = int(round(x_real / RESOLUTION))
    fila_relativa = int(round(y_real / RESOLUTION))

    col_mapa = MAP_CENTER_CELL + col_relativa
    fila_mapa = MAP_CENTER_CELL + fila_relativa

    if 0 <= fila_mapa < MAP_SIZE and 0 <= col_mapa < MAP_SIZE:
        return fila_mapa, col_mapa
    else: 
        return None, None

# FUNCIONES
def main():
    client = RemoteAPIClient()
    sim = client.require('sim')

    sim.setStepping(True)

    sim.startSimulation()

    sim.addLog(sim.verbosity_scriptinfos, f"Iniciando busqueda y rescate")

    # obtener handles de los objetos a utilizar

    estado_actual = ESTADO_EXPLORACION

    while True:

        match estado_actual:

            case ESTADO_EXPLORACION:


            case ESTADO_APROXIMACION:


            case ESTADO_AGARRE:
                ejecutar_agarre(sim, handle_objetivo)

            case ESTADO_PLANIFICACION:
                posicion_real = [leer_odometria(sim, robot)]
                posicion_objetivo = [0,0]
                posicion_actual = posicion_real_a_matriz(posicion_real[0], posicion_real[1])
                comandos_de_retorno = calcular_ruta_mas_corta(mapa_matriz, posicion_actual, posicion_objetivo)
                estado_actual = ESTADO_RETORNO

            case ESTADO_RETORNO:
                ejecutar_paso_en_ruta(sim, comandos_de_retorno)

                estado_actual = ESTADO_SOLTADO

            case ESTADO_SOLTADO:
                ejecutar_soltado(sim, handle_objetivo)
                

            case ESTADO_FINALIZADO:
                sim.addLog(sim.verbosity_scriptinfos, f"Busqueda y rescate exitosa.")
                sim.stopSimulation()

            


def ajustar_movimiento_robot(leftSpeed, rightSpeed, sim, right_motor, left_motor):
    sim.setJointTargetVelocity(left_motor, leftSpeed)
    sim.setJointTargetVelocity(right_motor, rightSpeed)

def obtener_handles_objetos(sim): # obtiene todos los handles de los objetos que serán utilizados
    rm = sim.getObject("/PioneerP3DX/rightMotor")
    lm = sim.getObject("/PioneerP3DX/leftMotor")
    robot = sim.getObject("/PioneerP3DX")
    # falta conseguir mas handles de objetos


def leer_odometria(sim, robot, sensor_orientacion): # obtiene posicion x,y así como orientacion (theta) del robot
    # obtener posicion del robot y guardar en x e y
    # obtener orientacion del robot y guardar en theta

    return x, y, theta

def leer_sensores_proximidad(sim, sensores): # lee valores de sensores de proximidad para evitar colisiones y mapear rutas, sensores es un array de sensores


def buscar_objetivo_por_camara(sim, camara): # procesa imagen de la camara y devuelve true o false y posicion aproximada en la imagen


def actualizar_mapa(mapa_actual, pos_robot, datos_sensores): # Actualiza el mapa actual marcando las celdas frente a los sensores como ocupadas y propagando el costo de retorno


def evitar_colisiones(datos_sensores): # lógica de seguridad para verificar si es necesaria una parada o una maniobra de emergencia para evitar una colision


def transicion_estado(datos_estado): # funcion que recibe las lecturas y segun la logica devuelve el siguiente estado para el match


def ejecutar_exploracion(sim, datos_sensores): # implementa logica de navegacion (seguimiento de pared)


def ejecutar_aproximacion(sim, posicion_objetivo): # Dirige el robot al objetivo con alta precision hasta la distancia segura necesaria para el agarre


def ejecutar_agarre(sim, handle_objetivo): # oculta/elimina el objetivo de la simulacion y comenta accion por el log
    # ocultar el objetivo frente al robot
    sim.addLog(sim.verbosity_scriptinfos, f"Objetivo {handle_objetivo} encontrado. Siguiente tarea: devolverlo al origen.")

def ejecutar_soltado(sim, handle_objetivo): # omuestra/desoculta el objetivo de la simulacion y comenta accion por el log
    # desocultar el objetivo frente al robot
    sim.addLog(sim.verbosity_scriptinfos, f"Objetivo {handle_objetivo} recuperado y devuelto al origen.")

def calcular_ruta_mas_corta(mapa_actual, pos_origen, pos_destino): # Implementa algoritmo de busqueda (djikstra o a*) para generar una lista de movimientos optima (array de movimientos [avanzar, girarDerecha, avanzar, girarIzquierda, etc])
    

def ejecutar_paso_en_ruta(sim, ruta): # ejecuta el siguiente comando de movimiento de la ruta y consume el paso, verifica si se ha llegado hasta 0,0


def ejecutar_comando_movimiento(sim, comando, distance_angle): # traduce un comando abstracto en comandos de velocidad a los motores para avanzar o girar


