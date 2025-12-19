from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import math, heapq, cv2

class C:
    # --- Mapa ---
    DIMENSION_MAPA_METROS = 51
    TAMANO_CELDA = 1.0 # Un metro por celda
    DIMENSION_MAPA_GRID = int(DIMENSION_MAPA_METROS / TAMANO_CELDA)
    CENTRO_MAPA = DIMENSION_MAPA_GRID // 2
    
    # --- Códigos del Grid ---
    GRID_UNKNOWN, GRID_OBSTACLE, GRID_FREE = 0, 1, 2

    # --- Cinemática LineTracer ---
    WHEEL_RADIUS = 0.027
    NOMINAL_VEL_LINEAR = 0.3
    FACTOR_SLOW = 0.3

    # Factor de conversion de metros a radianes por segundo
    TO_ANGULAR = 1 / WHEEL_RADIUS

    VEL_GIRO_RAD = 2.0 # Velocidad angular para giros de 90 o 180 grados
    
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
    robot, m_izq, m_der, sens_linea, cam, h_obj, maze = handles
    
    # 2. Inicialización
    grid = np.full((C.DIMENSION_MAPA_GRID, C.DIMENSION_MAPA_GRID), C.GRID_UNKNOWN, dtype=np.int8)
    estado = State.EXPLORACION
    ruta_retorno = []
    
    print(">>> INICIANDO LINETRACER: Modo Exploración")

    while estado != State.FINALIZADO:
        # A. PERCEPCIÓN
        # 1. Leer Línea: Devuelve booleans (True=Negro/Rojo, False=Blanco/Suelo)
        izq, cen, der = leer_sensores_linea(sim, sens_linea)
        
        estado_colision, _ = sim.checkCollision(robot, maze)
        chocado = (estado_colision > 0)

        # 2. Odometría y Mapeo Continuo (Pintamos por donde pasamos)
        pos_actual = sim.getObjectPosition(robot, sim.handle_world)
        percibir_y_mapear(pos_actual, grid)

        # 3. Visión Frontal (Busca el objetivo)
        obj_visto, _ = procesar_vision(sim, cam, h_obj)
        
        # B. MÁQUINA DE ESTADOS
        vl_rad, vr_rad = 0, 0

        if estado == State.EXPLORACION:
            # CONDICIÓN DE ÉXITO: Ver el objetivo
            if obj_visto:
                print(">>> ¡OBJETIVO LOCALIZADO! Deteniendo para agarrar.")
                vl_rad, vr_rad = 0, 0
                sim.setJointTargetVelocity(m_izq, 0)
                sim.setJointTargetVelocity(m_der, 0)
                estado = State.AGARRANDO
                continue # Saltar al siguiente ciclo para cambiar lógica

            if chocado:
                print(">>> ¡CHOQUE DETECTADO! Es un callejón sin salida.")
                # Lógica de rebote: Retroceder y dar media vuelta
                maniobra_choque_y_vuelta(sim, m_izq, m_der)
            # LÓGICA DE NAVEGACIÓN (ÁRBOL)
            # Caso 1: INTERSECCIÓN (Izquierda + Derecha + Centro o variantes)
            # En un laberinto perfecto, T suele ser Izq+Cen+Der o Izq+Cen
            es_interseccion = (izq and der)
            
            # Caso 2: CALLEJÓN SIN SALIDA (Todo blanco)
            

            if es_interseccion:
                # ESTRATEGIA: "Mano Izquierda" (Prioridad: Izq -> Recto -> Der)
                # Avanzamos un poco para centrar el eje de ruedas en el cruce
                maniobra_avanzar_un_poco(sim, m_izq, m_der, 0.2)
                
                # Giramos 90 grados a la izquierda ciegamente
                maniobra_girar_90(sim, m_izq, m_der, 'izq')
                
            else:
                # 4. SEGUIMIENTO DE LÍNEA (Lógica exacta del script Lua)
                # Velocidad lineal base
                lin_izq = C.NOMINAL_VEL_LINEAR
                lin_der = C.NOMINAL_VEL_LINEAR
                
                # Si sensor izquierdo NO ve línea (false), frenar rueda izquierda
                # Esto causa un giro suave a la izquierda para recuperar la línea
                if not izq:
                    lin_izq = lin_izq * C.FACTOR_SLOW
                
                # Si sensor derecho NO ve línea (false), frenar rueda derecha
                if not der:
                    lin_der = lin_der * C.FACTOR_SLOW

                # Si el sensor central pierde la línea pero no los laterales (hueco o curva rara),
                # la lógica de arriba sigue funcionando bien.
                
                # Si NINGUNO ve línea (0-0-0), el script Lua frenaría ambos.
                # Nosotros añadimos seguridad: si perdemos línea por completo, es un final de camino lógico
                # (aunque preferimos usar el choque físico, esto es backup)
                if not izq and not der and not cen:
                    # Opcional: Activar maniobra de vuelta si no hay colisión pero se acabó la línea
                    pass 

                # Conversión final a radianes/segundo para los motores
                vl_rad = lin_izq * C.TO_ANGULAR
                vr_rad = lin_der * C.TO_ANGULAR

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
                vl_rad, vr_rad = comportamiento_seguir_ruta(pos_actual, sim.getObjectOrientation(robot, sim.handle_world)[2], ruta_retorno)
                # ... Lógica de Pure Pursuit ...
                # Si llegamos: ruta_retorno.pop(0)
                pass 

        elif estado == State.SOLTANDO:
            ejecutar_soltar(sim, h_obj)
            estado = State.FINALIZADO

        # Aplicar velocidades
        sim.setJointTargetVelocity(m_izq, vl_rad)
        sim.setJointTargetVelocity(m_der, vr_rad)
        sim.step()

    sim.stopSimulation()

# ==============================================================================
#  FUNCIONES DE APOYO (REVISADAS)
# ==============================================================================

def obtener_handles(sim):
    """Adaptado para LineTracer en CoppeliaSim."""
    robot = sim.getObject("/LineTracer") # Ajusta el nombre si es distinto
    m_izq = sim.getObject("/LineTracer/DynamicLeftJoint")
    m_der = sim.getObject("/LineTracer/DynamicRightJoint")
    # Sensores de visión mirando al suelo (Left, Middle, Right)
    s_izq = sim.getObject("/LineTracer/LeftSensor")
    s_cen = sim.getObject("/LineTracer/MiddleSensor")
    s_der = sim.getObject("/LineTracer/RightSensor")
    # Sensor frontal, objetivo y laberinto
    cam = sim.getObject("/LineTracer/Vision_sensor")
    obj = sim.getObject("/Objetivo")
    maze = sim.getObject("/Maze")
    
    return robot, m_izq, m_der, [s_izq, s_cen, s_der], cam, obj, maze

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
        datos = sim.readVisionSensor(s)
        # 2. Verificación de tipo para evitar el error de unpacking
        if isinstance(datos, int):
            # Caso A: La API devolvió solo el estado (0 o 1)
            res = datos
        elif isinstance(datos, (list, tuple)):
            # Caso B: La API devolvió [resultado, auxData, auxSizes]
            res = datos[0]
        else:
            # Caso C: Retorno inesperado (por seguridad)
            res = 0
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
    vel = C.VEL_GIRO_RAD
    if sentido == 'izq': v_l, v_r = -vel, vel
    else: v_l, v_r = vel, -vel
    
    sim.setJointTargetVelocity(mi, v_l)
    sim.setJointTargetVelocity(md, v_r)
    
    # Esperar tiempo equivalente a 90 grados (Ajustar empíricamente '20' steps)
    for _ in range(16): 
        sim.step()

def maniobra_girar_180(sim, mi, md):
    maniobra_choque_y_vuelta(sim, mi, md)

def maniobra_choque_y_vuelta(sim, mi, md):
    """
    Secuencia para salir de un callejón tras colisión:
    1. Parar.
    2. Retroceder (despegarse de la pared).
    3. Girar 180º.
    """
    # 1. Frenar en seco
    sim.setJointTargetVelocity(mi, 0)
    sim.setJointTargetVelocity(md, 0)
    sim.step() # Un paso para aplicar freno
    
    # 2. Retroceder (Velocidad negativa)
    sim.setJointTargetVelocity(mi, -2.0)
    sim.setJointTargetVelocity(md, -2.0)
    
    # Retrocedemos durante unos ciclos (ej. 15 steps)
    for _ in range(15):
        sim.step()
    sim.setJointTargetVelocity(mi, -C.VEL_GIRO_RAD)
    sim.setJointTargetVelocity(md, C.VEL_GIRO_RAD)
    for _ in range(32): sim.step()
        
    # 3. Media Vuelta (Giro 180)
    maniobra_girar_180(sim, mi, md)

def maniobra_avanzar_un_poco(sim, mi, md, tiempo_o_dist):
    """Avanzar ciego para cruzar la intersección"""
    vel_rad = C.NOMINAL_VEL_LINEAR * C.TO_ANGULAR
    sim.setJointTargetVelocity(mi, C.vel_rad)
    sim.setJointTargetVelocity(md, C.vel_rad)
    for _ in range(int(tiempo_o_dist * 20)): 
        sim.step()

# --- REUTILIZABLES DE TU CÓDIGO ---
def mundo_a_grid(pos_mundo):
    c = int(pos_mundo[0] / C.TAMANO_CELDA) + C.CENTRO_MAPA
    r = int(pos_mundo[1] / C.TAMANO_CELDA) + C.CENTRO_MAPA
    return r, c

def procesar_vision(sim, camara_handle, obj_handle):
    img, res = sim.getVisionSensorImg(camara_handle)
    if not img: return False, None
    image = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)
    image = cv2.flip(image, 0)
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, C.LOWER_GREEN_HSV, C.UPPER_GREEN_HSV)
    if cv2.countNonZero(mask) > C.AREA_MINIMA_DETECCION:
        return True, 0 # Simplificado para detección
    return False, None

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

def comportamiento_seguir_ruta(pos_actual, ori_actual, ruta):
    if not ruta: return 0.0, 0.0
    
    # Objetivo
    target_r, target_c = ruta[0]
    target_x = (target_c - C.CENTRO_MAPA) * C.TAMANO_CELDA
    target_y = (target_r - C.CENTRO_MAPA) * C.TAMANO_CELDA
    
    dx = target_x - pos_actual[0]
    dy = target_y - pos_actual[1]
    dist = math.hypot(dx, dy)
    
    if dist < 0.2:
        ruta.pop(0)
        return 0.0, 0.0
        
    target_ang = math.atan2(dy, dx)
    error = target_ang - ori_actual
    error = math.atan2(math.sin(error), math.cos(error))
    
    # Control simple para retorno
    kp = 2.0
    w = error * kp
    v = 0.2 # Velocidad retorno moderada
    
    # Convertir a velocidad ruedas (aprox para diferencial)
    # v = (vl + vr)/2 * r ... simplificamos
    vl = (v - w*0.1) * C.TO_ANGULAR
    vr = (v + w*0.1) * C.TO_ANGULAR
    
    return vl, vr

if __name__ == "__main__":
    main()