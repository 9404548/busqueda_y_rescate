from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import math, heapq, cv2

class C:
    # --- Simulación y Mapa ---
    DIMENSION_MAPA_METROS = 51
    TAMANO_CELDA = 1
    DIMENSION_MAPA_GRID = int(DIMENSION_MAPA_METROS / TAMANO_CELDA)
    CENTRO_MAPA = DIMENSION_MAPA_GRID // 2
    
    # --- Códigos del Grid ---
    GRID_UNKNOWN, GRID_OBSTACLE, GRID_FREE = 0, 1, 2

    # --- Cinemática y Controladores ---
    TOLERANCIA_ANGULO = 0.005
    TOLERANCIA_POSICION = 0.02
    KP_GIRO = 1.0
    KP_AVANCE = 2.5
    DISTANCIA_PASO_GRID = 1.0
    UMBRAL_PARED = 0.8
    
    # --- Sensores y Percepción ---
    MAX_DIST_SENSOR = 5.0
    RANGO_MAPEADO = 4.0
    ANGULOS_SENSORES_GRADOS = [90, 50, 30, 10, -10, -30, -50, -90, -90, -130, -150, -170, 170, 150, 130, 90]

    # --- Lógica de Estados y Visión ---
    UMBRAL_AGARRE = 0.4
    AREA_MINIMA_DETECCION = 300
    LOWER_GREEN_HSV = np.array([40, 50, 50])
    UPPER_GREEN_HSV = np.array([80, 255, 255])


# ==============================================================================
#  2. DEFINICIÓN DE LA MÁQUINA DE ESTADOS (FSM)
# ==============================================================================
class State:
    EXPLORACION, APROXIMACION, AGARRANDO, PLANIFICACION, RETORNO, SOLTANDO, FINALIZADO = range(7)

ESTADO_NOMBRES = { state: name for name, state in State.__dict__.items() if not name.startswith('__') }

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.setStepping(True)
    sim.startSimulation()

    robot, m_izq, m_der, cam, sen_us, h_obj = obtener_handles(sim)

    grid = np.full((C.DIMENSION_MAPA_GRID, C.DIMENSION_MAPA_GRID), C.GRID_UNKNOWN, dtype=np.int8)
    estado = State.EXPLORACION
    target_ori, target_pos, start_pos, ruta = None, None, None, []

    while estado != State.FINALIZADO:
        pos, ori, dists = percibir_y_mapear(sim, robot, sen_us, grid)
        obj_detectado, obj_pos = procesar_vision(sim, cam, h_obj)
        vl, vr = 0, 0
        if estado == State.EXPLORACION:
            
            estado = State.APROXIMACION
        elif estado == State.APROXIMACION:

            estado = State.AGARRANDO
        elif estado == State.AGARRANDO:

            estado = State.PLANIFICACION
        elif estado == State.PLANIFICACION:

            estado = State.RETORNO
        elif estado == State.RETORNO:

            estado = State.SOLTANDO
        elif estado == State.SOLTANDO:

            estado = State.FINALIZADO
        gestionar_movimientos(giro_necesario, sentido_giro, avance_necesario, target_cardinal, target_posicion)


def obtener_handles(sim):
    """Obtiene todos los handles de los objetos de la simulación."""
    robot = sim.getObject("/PioneerP3DX")
    return robot, sim.getObject("./leftMotor"), sim.getObject("./rightMotor"), \
           sim.getObject("./Vision_sensor"), [sim.getObject(f"./ultrasonicSensor[{i}]") for i in range(16)], \
           sim.getObject("/Objetivo")
    

def percibir_y_mapear():
    
    pass

def procesar_vision(sim, camara_handle, obj_handle):
    img, res = sim.getVisionSensorImg(camara_handle)
    if not img: return False, None
    
    image = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)
    image = cv2.flip(image, 0)
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, C.LOWER_GREEN_HSV, C.UPPER_GREEN_HSV)
    
    m = cv2.moments(mask)
    obj_pos = sim.getObjectPosition(obj_handle, -1) if m["m00"] > C.AREA_MINIMA_DETECCION else None
    
    cv2.imshow("VISTA ROBOT", image)
    cv2.imshow("MASCARA VERDE", mask)
    cv2.waitKey(1) 
    return obj_pos is not None, obj_pos
    

def gestionar_movimientos(giro_necesario, sentido_giro, avance_necesario, target_cardinal, target_posicion):