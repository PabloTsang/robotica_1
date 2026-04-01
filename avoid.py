import robotica
import time

# --- VARIABLES GLOBALES ---
ultimo_error = 0.0
contador_convexo = 0 
lado_seguimiento = None  # Se fijará en "DER" o "IZQ"

def follow_wall_final(readings):
    global ultimo_error, contador_convexo, lado_seguimiento
    
    # --- CONFIGURACIÓN PARA GIROS SUAVES ---
    TICKS_RETRASO = 8      
    RADIO_GIRO = 0.45      # Aumentado para que las curvas en esquinas sean más amplias y parabólicas
    VEL_MAX = 1.0          # Mantenemos tu velocidad
    DIST_DESEADA_BASE = 0.40 
    KP = 2.5               # Reducido un poco para evitar oscilaciones (zig-zag)
    KD = 7.0               # Reducido proporcionalmente
    UMBRAL_DETECCION = 0.55 

    # 1. BÚSQUEDA Y BLOQUEO DE LADO
    if lado_seguimiento is None:
        dist_lat_izq = readings[0]
        dist_lat_der = readings[7]
        dist_frontal = min(readings[3], readings[4])

        if dist_lat_izq < UMBRAL_DETECCION:
            lado_seguimiento = "IZQ"
            print(">>> Lado fijado: IZQUIERDA.")
        elif dist_lat_der < UMBRAL_DETECCION:
            lado_seguimiento = "DER"
            print(">>> Lado fijado: DERECHA.")
        elif dist_frontal < UMBRAL_DETECCION:
            lado_seguimiento = "DER" 
            return 0.1, 0.4 # Giro inicial suave
        else:
            return VEL_MAX, VEL_MAX

    dist_deseada_actual = DIST_DESEADA_BASE
    MARGEN_SEGURIDAD_OPUESTO = 0.35 

    # --- LÓGICA SI SIGUE PARED IZQUIERDA ---
    if lado_seguimiento == "IZQ":
        dist_opuesta = readings[7]
        if dist_opuesta < MARGEN_SEGURIDAD_OPUESTO:
            dist_deseada_actual = max(0.20, DIST_DESEADA_BASE - (MARGEN_SEGURIDAD_OPUESTO - dist_opuesta))
        
        # Ampliamos un poco el rango frontal para empezar a girar antes y más suave
        dist_frontal = min(readings[2], readings[3])
        d_diag = readings[1]
        d_lat = readings[0]

        # A. Obstáculo Frontal (Giro en arco, no sobre su eje)
        if dist_frontal < 0.45:
            ultimo_error = 0
            # Mantiene tracción hacia adelante en la rueda exterior para hacer una curva
            return VEL_MAX * 0.7, -VEL_MAX * 0.1

        # B. Esquina Convexa
        if d_diag > UMBRAL_DETECCION and d_lat > UMBRAL_DETECCION:
            if contador_convexo < TICKS_RETRASO:
                contador_convexo += 1
                return VEL_MAX, VEL_MAX
            else:
                return (VEL_MAX * RADIO_GIRO), VEL_MAX 
        
        # C. Seguimiento PD Normal
        contador_convexo = 0
        error = d_diag - dist_deseada_actual
        ajuste = (error * KP) + ((error - ultimo_error) * KD)
        ultimo_error = error
        
        # --- SUAVIZADO DEL CONTROLADOR ---
        # 1. Saturamos el ajuste para que no provoque cambios violentos
        ajuste = max(min(ajuste, VEL_MAX * 0.8), -VEL_MAX * 0.8)
        # 2. Si el robot tiene que corregir mucho, le bajamos la velocidad base temporalmente
        vel_base = VEL_MAX - (abs(ajuste) * 0.4) 
        
        return (vel_base - ajuste), (vel_base + ajuste)

    # --- LÓGICA SI SIGUE PARED DERECHA ---
    elif lado_seguimiento == "DER":
        dist_opuesta = readings[0]
        if dist_opuesta < MARGEN_SEGURIDAD_OPUESTO:
            dist_deseada_actual = max(0.20, DIST_DESEADA_BASE - (MARGEN_SEGURIDAD_OPUESTO - dist_opuesta))

        dist_frontal = min(readings[4], readings[5])
        d_diag = readings[6]
        d_lat = readings[7]

        # A. Obstáculo Frontal
        if dist_frontal < 0.45:
            ultimo_error = 0
            return -VEL_MAX * 0.1, VEL_MAX * 0.7

        # B. Esquina Convexa
        if d_diag > UMBRAL_DETECCION and d_lat > UMBRAL_DETECCION:
            if contador_convexo < TICKS_RETRASO:
                contador_convexo += 1
                return VEL_MAX, VEL_MAX
            else:
                return VEL_MAX, (VEL_MAX * RADIO_GIRO)
        
        # C. Seguimiento PD Normal
        contador_convexo = 0
        error = d_diag - dist_deseada_actual
        ajuste = (error * KP) + ((error - ultimo_error) * KD)
        ultimo_error = error
        
        # --- SUAVIZADO DEL CONTROLADOR ---
        ajuste = max(min(ajuste, VEL_MAX * 0.8), -VEL_MAX * 0.8)
        vel_base = VEL_MAX - (abs(ajuste) * 0.4) 
        
        return (vel_base + ajuste), (vel_base - ajuste)

def main():
    try:
        coppelia = robotica.Coppelia()
        robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX') 
        coppelia.start_simulation()
        
        print("Controlador Iniciado. Buscando paredes de forma fluida...")
        
        while coppelia.is_running():
            readings = robot.get_sonar()
            if readings and len(readings) >= 8:
                lspeed, rspeed = follow_wall_final(readings)
                
                lspeed = max(min(lspeed, 1.5), -1.0)
                rspeed = max(min(rspeed, 1.5), -1.0)
                
                robot.set_speed(lspeed, rspeed)
            time.sleep(0.05)

    except Exception as e:
        print(f"Error en ejecución: {e}")
    finally:
        if 'robot' in locals():
            robot.set_speed(0, 0)
        coppelia.stop_simulation()

if __name__ == '__main__':
    main()