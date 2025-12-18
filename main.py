from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import math, heapq, cv2
from math import inf

# --- CONFIGURACIÓN GLOBAL ---
DIMENSION_MAPA = 100
TAMANO_CELDA = 0.2
CENTRO = DIMENSION_MAPA // 2

# --- DEFINICIÓN DE ESTADOS ---
ESTADO_EXPLORANDO = 0   
ESTADO_APROXIMAR  = 1   
ESTADO_AGARRAR    = 2   
ESTADO_PLANIFICAR = 3   
ESTADO_RETORNO    = 4   
ESTADO_SOLTAR     = 5   
ESTADO_FIN        = 6   

class djikstra_node:
    def __init__(self, coordinates, distance, previous):
        self.coordinates = coordinates
        self.distance = distance
        self.previous = previous


def main():
    print("--- INICIANDO SISTEMA DE CONTROL DOCUMENTADO ---")
    
    # 1. SETUP
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.setStepping(True)
    sim.startSimulation()

    # 2. HANDLES
    robot = sim.getObject("/PioneerP3DX")
    
    motor_izq = sim.getObject("/PioneerP3DX/leftMotor")
    motor_der = sim.getObject("/PioneerP3DX/rightMotor")
    
    camara = sim.getObject("/PioneerP3DX/Vision_sensor")    
    
    sensores_us = [sim.getObject(f"/PioneerP3DX/ultrasonicSensor[{i}]") for i in range(16)]
    
    h_objetivo = sim.getObject("/Objetivo")

    # 3. VARIABLES
    grid = np.zeros((DIMENSION_MAPA, DIMENSION_MAPA), dtype=np.int8) 
    ruta_retorno = [] 
    estado_actual = ESTADO_EXPLORANDO

    # --- BUCLE PRINCIPAL ---
    while estado_actual != ESTADO_FIN:
        
        # A. PERCEPCIÓN
        pos, ori, distancias_sensores = percibir_y_mapear(sim, robot, sensores_us, grid)
        objetivo_detectado, centro_x_objetivo = procesar_vision(sim, camara)

        vl, vr = 0, 0

        # B. MÁQUINA DE ESTADOS
        if estado_actual == ESTADO_EXPLORANDO:
            if objetivo_detectado:
                print(">>> Objetivo visto. Estado: APROXIMACIÓN.")
                estado_actual = ESTADO_APROXIMAR
            else:
                vl, vr = comportamiento_seguir_pared_izq(distancias_sensores)

        elif estado_actual == ESTADO_APROXIMAR:
            # Sensores 3 y 4 son los frontales centrales
            dist_frontal = min(distancias_sensores[3], distancias_sensores[4])
            if dist_frontal < 0.4:
                estado_actual = ESTADO_AGARRAR
            else:
                vl, vr = comportamiento_visual_aproximando(centro_x_objetivo)

        elif estado_actual == ESTADO_AGARRAR:
            ejecutar_agarre(sim, h_objetivo)
            estado_actual = ESTADO_PLANIFICAR

        elif estado_actual == ESTADO_PLANIFICAR:
            inicio = mundo_a_grid(pos)
            fin = (CENTRO, CENTRO) # Volver al origen (0,0)
            
            ruta_retorno = calcular_dijkstra(grid, inicio, fin)
            if not ruta_retorno: ruta_retorno = [fin] # Fallback
            
            estado_actual = ESTADO_RETORNO

        elif estado_actual == ESTADO_RETORNO:
            if not ruta_retorno:
                estado_actual = ESTADO_SOLTAR
            else:
                vl, vr = comportamiento_seguir_ruta(pos, ori, ruta_retorno)

        elif estado_actual == ESTADO_SOLTAR:
            ejecutar_soltar(sim, h_objetivo)
            estado_actual = ESTADO_FIN

        # C. ACTUACIÓN
        sim.setJointTargetVelocity(motor_izq, vl)
        sim.setJointTargetVelocity(motor_der, vr)
        sim.step()

    sim.stopSimulation()
    print("--- FIN DE MISIÓN ---")


# ==============================================================================
#                 PROTOTIPOS DE FUNCIONES A IMPLEMENTAR
# ==============================================================================

def percibir_y_mapear(sim, robot, sensores_handles, grid):
    """
    Lee la odometría y sensores, y actualiza la matriz de ocupación (grid).

    Args (Entrada):
        sim: Objeto cliente de la API de CoppeliaSim.
        robot: Handle (int) del robot Pioneer.
        sensores_handles: Lista de 16 enteros con los handles de los sensores US.
        grid: Matriz Numpy (DIMxDIM) global. Se modifica 'in-place'.

    Returns (Salida):
        pos: Lista [x, y, z] con la posición absoluta del robot.
        ori: Float con el ángulo Theta (yaw) en radianes.
        dists: Lista de 16 floats con las distancias (en metros) de cada sensor. 
               Si no detecta nada, usar 5.0 (máximo).
    """
    # Implementar lectura y mapeo trigonométrico
    return [x,y,z], theta, dists

def procesar_vision(sim, camara_handle):
    img_raw, res = sim.getVisionSensorImg(camara_handle, 0)
    if not img_raw:
        return False, 0
    # 2. Procesar buffer a formato OpenCV
    image = np.frombuffer(img_raw, dtype=np.uint8)
    image = image.reshape(res[1], res[0], 3)
    # 3. Corregir orientación
    image = cv2.flip(image, 0) 
    # 4. Cambio de espacio de color BGR/RGB a HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    # 5. Definir rango de color VERDE
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([80, 255, 255])
    # 6. Crear máscara binaria
    mask = cv2.inRange(hsv, lower_green, upper_green)
    # 7. Calcular centroide (Momentos)
    M = cv2.moments(mask)
    area = M["m00"] # Área total de píxeles blancos (detectados)
    detectado = False
    center_x = 0
    # Umbral de ruido: Solo consideramos detección si el área es > 300 px
    if area > 300:
        center_x = int(M["m10"] / area)
        center_y = int(M["m01"] / area)
        detectado = True
        # Opcional: Dibujar un punto en el centro para debug visual
        cv2.circle(image, (center_x, center_y), 5, (0, 0, 255), -1)
    # 8. Visualización (Opcional para debug)
    cv2.imshow("Vista Robot", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    cv2.imshow("Mascara Verde", mask)
    cv2.waitKey(1) 

    return detectado, center_x

def comportamiento_seguir_pared_izq(dists):
    """
    Calcula velocidades para seguir la pared izquierda usando fusión de sensores.
    Argumentos (Entrada):
        dists: Lista de 16 floats con las distancias actuales de los sensores.
    Returns (Salida):
        vl: Float (Velocidad motor izquierdo).
        vr: Float (Velocidad motor derecho).
    Lógica:
        - Usar min(dists[0], dists[1]) para medir distancia lateral.
        - Usar dists[2] para anticipar esquinas (campo potencial).
        - Usar min(dists[3], dists[4]) para evasión frontal.
    """
    # Implementar lógica reactiva para seguir pared continuamente
    return vl, vr

def comportamiento_visual_aproximando(centro_x):
    ANCHO_IMAGEN = 256      # Resolución horizontal de la cámara
    CENTRO_IDEAL = ANCHO_IMAGEN // 2
    VELOCIDAD_BASE = 2.0    # Velocidad de avance (m/s)
    KP = 0.02               # Ganancia Proporcional

    error = CENTRO_IDEAL - centro_x
    giro = error * KP

    vl = VELOCIDAD_BASE - giro
    vr = VELOCIDAD_BASE + giro

    return vl, vr

def calcular_dijkstra(grid, inicio, fin):
    rows, cols = grid.shape
    # dist(s) = 0. Usamos un diccionario para 'dist' y 'prev'.
    dist = {inicio: 0}
    prev = {inicio: None}
    # La cola de prioridad guarda tuplas: (distancia, (fila, col))
    pq = [(0, inicio)] 
    while pq:
        # u = deletemin(H)
        d_actual, u = heapq.heappop(pq)
        # Optimización: Si sacamos el nodo destino, terminamos antes.
        if u == fin:
            break
        # Si encontramos un camino más corto a este nodo antes, ignorar este obsoleto
        if d_actual > dist.get(u, float('inf')):
            continue
        # Definimos los vecinos (Arriba, Abajo, Izquierda, Derecha)
        r, c = u
        vecinos = [
            (r-1, c), (r+1, c), 
            (r, c-1), (r, c+1)
        ]
        for v in vecinos:
            vr, vc = v
            # Verificar límites del mapa y que no sea un obstáculo (Muro = 1)
            if 0 <= vr < rows and 0 <= vc < cols and grid[vr, vc] != 1:
                costo = 1 
                nueva_dist = d_actual + costo
                if nueva_dist < dist.get(v, float('inf')):
                    dist[v] = nueva_dist
                    prev[v] = u
                    heapq.heappush(pq, (nueva_dist, v))
    # El algoritmo llenó 'prev', ahora debemos ir de Fin a Inicio
    ruta = []
    curr = fin
    if curr not in prev:
        return [] # No se encontró camino
    while curr is not None:
        ruta.append(curr)
        curr = prev[curr]
    ruta.reverse() # Invertimos para tener Inicio -> Fin
    return ruta

def comportamiento_seguir_ruta(pos_actual, ori_actual, ruta):
    # 1. Si no hay ruta, parar
    if not ruta:
        return 0.0, 0.0
    target_r, target_c = ruta[0]
    # Conversión Grid -> Metros
    target_x = (target_c - CENTRO) * TAMANO_CELDA
    target_y = (target_r - CENTRO) * TAMANO_CELDA
    # 3. Calcular distancias
    dx = target_x - pos_actual[0]
    dy = target_y - pos_actual[1]
    distancia = math.hypot(dx, dy)
    # 4. Chequeo de llegada
    if distancia < 0.1: # Radio de aceptación de 10cm
        ruta.pop(0)     # Sacamos el punto de la lista
        return 0.0, 0.0 # Parada momentánea
    # 5. Calcular el ángulo necesario
    angulo_deseado = math.atan2(dy, dx)
    error = angulo_deseado - ori_actual
    # Normalizar error entre -PI y PI
    error = math.atan2(math.sin(error), math.cos(error))
    # UMBRAL DE GIRO: 0.1 radianes son aprox 6 grados.
    if abs(error) > 0.1:
        v_avance = 0.0
        v_giro = 1.0 * (1 if error > 0 else -1)
    else:
        # Ya estamos alineados, avanzamos recto.
        v_avance = 3.0
        # Pequeña corrección proporcional para no desviarnos mientras avanzamos
        v_giro = error * 2.0 
    # Asignar a motores
    vl = v_avance - v_giro
    vr = v_avance + v_giro

    return vl, vr

def ejecutar_agarre(sim, handle_obj):
    # Verificamos que el handle sea válido (que el objeto exista)
    if handle_obj != -1:
        # El parámetro 10 corresponde a la 'Visibility Layer'. 0 lo oculta.
        sim.setObjectInt32Param(handle_obj, 10, 0)
        sim.setObjectPosition(handle_obj, sim.handle_world, [0.0, 0.0, 3.35])
        
        print("Objeto capturado (invisible y sin físicas).")
    else:
        print("Error: No se encontró el handle del objeto para agarrar.")

def ejecutar_soltar(sim, handle_obj):
    if handle_obj != -1:
        # Activamos la capa de visibilidad (param 10) poniéndola en 1.
        sim.setObjectInt32Param(handle_obj, 10, 1)
        
        print("MISIÓN CUMPLIDA: Objeto entregado en la base.")
    else:
        print("Error: No hay objeto para soltar.")

def mundo_a_grid(pos_mundo):
    c = int(pos_mundo[0] / TAMANO_CELDA) + CENTRO
    r = int(pos_mundo[1] / TAMANO_CELDA) + CENTRO
    return r, c

if __name__ == "__main__":
    main()