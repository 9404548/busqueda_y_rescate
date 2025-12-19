from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import math, heapq, cv2

class C:
    # --- Mapa ---
    DIMENSION_MAPA_METROS = 51
    TAMANO_CELDA = 1.0 # Un metro por celda (ajustar segun tamaño real del laberinto)
    DIMENSION_MAPA_GRID = int(DIMENSION_MAPA_METROS / TAMANO_CELDA)
    CENTRO_MAPA = DIMENSION_MAPA_GRID // 2
    
    # --- Códigos del Grid ---
    GRID_UNKNOWN, GRID_OBSTACLE, GRID_FREE = 0, 1, 2

    # --- Cinemática LineTracer ---
    VEL_AVANZA = 4.0      # Velocidad base
    VEL_GIRO = 2.0         # Velocidad para maniobras
    KP_LINEA = 1.5         # Ganancia para seguir línea (ajustar si oscila)
    
    # ---- Visión Objetivo ----
    AREA_MINIMA_DETECCION = 200
    LOWER_GREEN_HSV = np.array([40, 50, 50])
    UPPER_GREEN_HSV = np.array([80, 255, 255])

class State:
    EXPLORACION, AGARRANDO, PLANIFICACION, RETORNO, SOLTANDO, FINALIZADO = range(6)

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.setStepping(True)
    sim.startSimulation()

    # 1. Obtener Handles (Limpio)
    handles = obtener_handles(sim)
    robot, m_izq, m_der, sens_linea, cam, h_obj = handles
    
    # 2. Inicialización
    grid = np.full((C.DIMENSION_MAPA_GRID, C.DIMENSION_MAPA_GRID), C.GRID_UNKNOWN, dtype=np.int8)
    estado = State.EXPLORACION
    ruta_retorno = []
    
    print(">>> INICIANDO LINETRACER: Modo Exploración")

    while estado != State.FINALIZADO:
        # A. PERCEPCIÓN
        # 1. Leer Línea: Devuelve booleans (True=Negro/Rojo, False=Blanco/Suelo)
        izq, cen, der = leer_sensores_linea(sim, sens_linea)
        
        # 2. Odometría y Mapeo Continuo (Pintamos por donde pasamos)
        pos_actual = sim.getObjectPosition(robot, sim.handle_world)
        percibir_y_mapear(pos_actual, grid)

        # 3. Visión Frontal (Busca el objetivo)
        obj_visto, obj_pos_img = procesar_vision(sim, cam, h_obj)
        
        # B. MÁQUINA DE ESTADOS
        vl, vr = 0, 0

        if estado == State.EXPLORACION:
            # CONDICIÓN DE ÉXITO: Ver el objetivo
            if obj_visto:
                print(">>> ¡OBJETIVO LOCALIZADO! Deteniendo para agarrar.")
                vl, vr = 0, 0
                sim.setJointTargetVelocity(m_izq, 0)
                sim.setJointTargetVelocity(m_der, 0)
                estado = State.AGARRANDO
                continue # Saltar al siguiente ciclo para cambiar lógica

            # LÓGICA DE NAVEGACIÓN (ÁRBOL)
            # Caso 1: INTERSECCIÓN (Izquierda + Derecha + Centro o variantes)
            # En un laberinto perfecto, T suele ser Izq+Cen+Der o Izq+Cen
            es_interseccion = (izq and der) or (izq and cen and der)
            
            # Caso 2: CALLEJÓN SIN SALIDA (Todo blanco)
            es_callejon = (not izq and not cen and not der)

            if es_interseccion:
                # ESTRATEGIA: "Mano Izquierda" (Prioridad: Izq -> Recto -> Der)
                # Avanzamos un poco para centrar el eje de ruedas en el cruce
                maniobra_avanzar_un_poco(sim, m_izq, m_der, 0.2)
                
                # Giramos 90 grados a la izquierda ciegamente
                maniobra_girar_90(sim, m_izq, m_der, 'izq')
                
            elif es_callejon:
                # Media vuelta
                maniobra_girar_180(sim, m_izq, m_der)
                
            else:
                # SEGUIMIENTO PID NORMAL
                # Error: -1 (izq), 0 (centro), 1 (der)
                error = 0
                if izq and not der: error = -1
                if der and not izq: error = 1
                
                vl = C.VEL_AVANZA + (error * C.KP_LINEA)
                vr = C.VEL_AVANZA - (error * C.KP_LINEA)

        elif estado == State.AGARRANDO:
            ejecutar_agarre(sim, h_obj)
            estado = State.PLANIFICACION

        elif estado == State.PLANIFICACION:
            # Dijkstra desde donde estamos hasta el centro (Inicio)
            inicio_grid = mundo_a_grid(pos_actual)
            fin_grid = (C.CENTRO_MAPA, C.CENTRO_MAPA)
            print(f"Calculando ruta de {inicio_grid} a {fin_grid}...")
            
            ruta_retorno = calcular_dijkstra(grid, inicio_grid, fin_grid)
            
            if not ruta_retorno:
                print("Error: No se encontró ruta de vuelta (¿Mapa desconectado?). Teletransportando...")
                estado = State.SOLTANDO # Fallback de emergencia
            else:
                estado = State.RETORNO

        elif estado == State.RETORNO:
            # Aquí ya NO usamos la línea roja obligatoriamente, usamos odometría pura
            # o una mezcla híbrida. Para simplificar, usaremos la función de ir a waypoint.
            if not ruta_retorno:
                estado = State.SOLTANDO
            else:
                # Lógica de ir al punto (extraída de tu código anterior o simplificada)
                target = ruta_retorno[0]
                # ... Lógica de Pure Pursuit ...
                # Si llegamos: ruta_retorno.pop(0)
                pass 

        elif estado == State.SOLTANDO:
            ejecutar_soltar(sim, h_obj)
            estado = State.FINALIZADO

        # Aplicar velocidades
        sim.setJointTargetVelocity(m_izq, vl)
        sim.setJointTargetVelocity(m_der, vr)
        sim.step()

    sim.stopSimulation()

# ==============================================================================
#  FUNCIONES DE APOYO (REVISADAS)
# ==============================================================================

def obtener_handles(sim):
    """Adaptado para LineTracer en CoppeliaSim."""
    robot = sim.getObject("/LineTracer") # Ajusta el nombre si es distinto
    m_izq = sim.getObject("./LeftMotor")
    m_der = sim.getObject("./RightMotor")
    # Sensores de visión mirando al suelo (Left, Middle, Right)
    s_izq = sim.getObject("./LeftSensor")
    s_cen = sim.getObject("./MiddleSensor")
    s_der = sim.getObject("./RightSensor")
    # Sensor frontal y objetivo
    cam = sim.getObject("./Vision_sensor")
    obj = sim.getObject("/CilindroObjetivo") # O como se llame tu objetivo
    
    return robot, m_izq, m_der, [s_izq, s_cen, s_der], cam, obj

def leer_sensores_linea(sim, sensores):
    """
    Lee los 3 sensores de suelo.
    Returns: (bool, bool, bool) -> True si detecta línea, False si es suelo.
    """
    lecturas = []
    for s in sensores:
        # readVisionSensor devuelve (res, auxData, ...). 
        # result=1 si detecta algo según sus filtros internos.
        # auxData[10] suele ser la intensidad media (0 a 1).
        res, aux, _ = sim.readVisionSensor(s)
        
        # Asumimos que el sensor está configurado para devolver res=1 sobre la línea
        # O podemos usar la intensidad: si intensidad < 0.5 es linea negra/roja
        detectado = (res == 1) 
        lecturas.append(detectado)
        
    return lecturas[0], lecturas[1], lecturas[2]

def percibir_y_mapear(pos_mundo, grid):
    """
    Marca la celda actual como LIBRE (2).
    Al usar Dijkstra luego, todo lo que sea 0 (Unknown) será tratado como muro.
    """
    r, c = mundo_a_grid(pos_mundo)
    # Protección de límites
    if 0 <= r < C.DIMENSION_MAPA_GRID and 0 <= c < C.DIMENSION_MAPA_GRID:
        grid[r, c] = C.GRID_FREE

def maniobra_girar_90(sim, mi, md, sentido):
    """Giro ciego de 90 grados (bloqueante o por tiempo)"""
    # Para simplificar en simulación step-by-step, usaremos un bucle rápido
    # En un robot real esto se hace con encoders o giroscopio
    vel = C.VEL_GIRO
    if sentido == 'izq': v_l, v_r = -vel, vel
    else: v_l, v_r = vel, -vel
    
    sim.setJointTargetVelocity(mi, v_l)
    sim.setJointTargetVelocity(md, v_r)
    
    # Esperar tiempo equivalente a 90 grados (Ajustar empíricamente '20' steps)
    for _ in range(25): 
        sim.step()

def maniobra_girar_180(sim, mi, md):
    """Media vuelta"""
    sim.setJointTargetVelocity(mi, -C.VEL_GIRO)
    sim.setJointTargetVelocity(md, C.VEL_GIRO)
    for _ in range(50): # Doble de tiempo que 90 grados
        sim.step()

def maniobra_avanzar_un_poco(sim, mi, md, distancia):
    """Avanzar ciego para cruzar la intersección"""
    sim.setJointTargetVelocity(mi, C.VEL_AVANZA)
    sim.setJointTargetVelocity(md, C.VEL_AVANZA)
    for _ in range(10): 
        sim.step()

# --- REUTILIZABLES DE TU CÓDIGO ---
def mundo_a_grid(pos_mundo):
    c = int(pos_mundo[0] / C.TAMANO_CELDA) + C.CENTRO_MAPA
    r = int(pos_mundo[1] / C.TAMANO_CELDA) + C.CENTRO_MAPA
    return r, c

def procesar_vision(sim, camara_handle, obj_handle):
    # (Tu función estaba bien, la mantengo igual o simplificada)
    img, res = sim.getVisionSensorImg(camara_handle)
    if not img: return False, None
    image = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)
    image = cv2.flip(image, 0)
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, C.LOWER_GREEN_HSV, C.UPPER_GREEN_HSV)
    if cv2.countNonZero(mask) > C.AREA_MINIMA_DETECCION:
        return True, 0 # Simplificado para detección
    return False, None

# (Funciones de Agarre, Soltar y Dijkstra se mantienen igual que las tenías)
# ... Pega aquí tu calcular_dijkstra, ejecutar_agarre, ejecutar_soltar ...

if __name__ == "__main__":
    main()