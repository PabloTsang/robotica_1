import robotica
import time

# --- VARIABLES GLOBALES ---
ultimo_error = 0.0
contador_convexo = 0 
lado_seguimiento = None  # Se fijará en "DER" o "IZQ"

def follow_wall_final(readings):
    global ultimo_error, contador_convexo, lado_seguimiento
    
    # --- CONFIGURACIÓN ---
    TICKS_RETRASO = 10     
    RADIO_GIRO = 0.1       
    VEL_MAX = 0.7          # Velocidad de crucero
    DIST_DESEADA = 0.40    # Distancia ideal a la pared
    KP = 4.0               
    KD = 12.0              
    UMBRAL_DETECCION = 0.40 # Distancia para considerar "contacto" con la pared

    # 1. BÚSQUEDA Y BLOQUEO DE LADO (Al inicio)
    if lado_seguimiento is None:
        dist_lat_izq = readings[0]
        dist_lat_der = readings[7]
        # Sensores 2 y 5 son los más laterales de los frontales
        dist_frontal_total = min(readings[2], readings[5])

        if dist_lat_izq < UMBRAL_DETECCION:
            lado_seguimiento = "IZQ"
            print(">>> Lado fijado: IZQUIERDA. Ignorando objetos a la derecha.")
        elif dist_lat_der < UMBRAL_DETECCION:
            lado_seguimiento = "DER"
            print(">>> Lado fijado: DERECHA. Ignorando objetos a la izquierda.")
        elif dist_frontal_total < UMBRAL_DETECCION:
            lado_seguimiento = "DER" # Por defecto tras choque frontal
            print(">>> Choque frontal en búsqueda. Fijando DERECHA.")
            return -0.2, 0.4 
        else:
            return VEL_MAX, VEL_MAX

    # 2. LÓGICA DE SEGUIMIENTO SELECTIVO (Mover colisión frontal aquí)
    
    if lado_seguimiento == "IZQ":
        # FILTRO DE SENSORES: Solo importan los del lado IZQ
        dist_frontal_relevante = min(readings[2], readings[3]) # Ignoramos 4 y 5
        d_diag = readings[1]
        d_lat = readings[0]

        # A. EVITAR COLISIÓN FRONTAL (Solo si el objeto está a la izquierda)
        # Si la pelota morada está a la DER, este valor será alto y el robot pasará de largo.
        if dist_frontal_relevante < 0.45:
            ultimo_error = 0 # Reset PD
            contador_convexo = 0
            # Giro sobre el eje a la derecha para alejarnos de la pared izquierda/obstáculo
            return VEL_MAX * 0.5, -VEL_MAX * 0.5

        # B. ESQUINA EXTERIOR (Convexo)
        if d_diag > UMBRAL_DETECCION and d_lat > UMBRAL_DETECCION:
            if contador_convexo < TICKS_RETRASO:
                contador_convexo += 1
                return VEL_MAX, VEL_MAX
            else:
                return (VEL_MAX * RADIO_GIRO), VEL_MAX 
        
        # C. PD NORMAL IZQUIERDA
        contador_convexo = 0
        error = d_diag - DIST_DESEADA
        ajuste = (error * KP) + ((error - ultimo_error) * KD)
        ultimo_error = error
        return (VEL_MAX - ajuste), (VEL_MAX + ajuste)

    elif lado_seguimiento == "DER":
        # FILTRO DE SENSORES: Solo importan los del lado DER
        dist_frontal_relevante = min(readings[4], readings[5]) # Ignoramos 2 y 3
        d_diag = readings[6]
        d_lat = readings[7]

        # A. EVITAR COLISIÓN FRONTAL (Solo si el objeto está a la derecha)
        if dist_frontal_relevante < 0.35:
            ultimo_error = 0 # Reset PD
            contador_convexo = 0
            # Giro sobre el eje a la izquierda para alejarnos de la pared derecha/obstáculo
            return -VEL_MAX * 0.5, VEL_MAX * 0.5

        # B. ESQUINA EXTERIOR
        if d_diag > UMBRAL_DETECCION and d_lat > UMBRAL_DETECCION:
            if contador_convexo < TICKS_RETRASO:
                contador_convexo += 1
                return VEL_MAX, VEL_MAX
            else:
                return VEL_MAX, (VEL_MAX * RADIO_GIRO)
        
        # C. PD NORMAL DERECHA
        contador_convexo = 0
        error = d_diag - DIST_DESEADA
        ajuste = (error * KP) + ((error - ultimo_error) * KD)
        ultimo_error = error
        return (VEL_MAX + ajuste), (VEL_MAX - ajuste)

def main():
    try:
        coppelia = robotica.Coppelia()
        robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX') 
        coppelia.start_simulation()
        
        while coppelia.is_running():
            readings = robot.get_sonar()
            if readings and len(readings) >= 16:
                lspeed, rspeed = follow_wall_final(readings)
                
                # Saturación final de motores
                lspeed = max(min(lspeed, 1.0), -0.3)
                rspeed = max(min(rspeed, 1.0), -0.3)
                
                robot.set_speed(lspeed, rspeed)
            time.sleep(0.05)

    except Exception as e:
        print(f"Error en ejecución: {e}")
    finally:
        if 'robot' in locals(): robot.set_speed(0, 0)
        coppelia.stop_simulation()

if __name__ == '__main__':
    main()