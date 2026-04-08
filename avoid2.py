'''
avoid.py

Sample client for the Pioneer P3DX mobile robot that implements a
kind of heuristic, rule-based controller for collision avoidance.

Copyright (C) 2023 Javier de Lope

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import robotica

# Distribución de sensores ultrasónicos Pioneer P3DX (CoppeliaSim)
# Índices: 0 → 15 en sentido horario alrededor del robot

# FRONT (frontal)
# [0] frontal izquierdo extremo
# [1] frontal izquierdo
# [2] frontal centro-izquierda
# [3] frontal centro
# [4] frontal centro-derecha
# [5] frontal derecha
# [6] frontal derecho extremo

# RIGHT SIDE (lateral derecho)
# [7] lateral derecho

# BACK (trasero)
# [8]  trasero derecho
# [9]  trasero centro-derecha
# [10] trasero centro
# [11] trasero centro-izquierda
# [12] trasero izquierdo

# LEFT-BACK (entre trasero y lateral izquierdo)
# [13] trasero-izquierda

# LEFT SIDE (lateral izquierdo)
# [14] delantero-izquierda
# [15] lateral izquierdo

def avoid(readings, side):
    target_dist = 0.4
    max_dist = 0.8
    base_speed = 1.0
    
    # Si aún no hemos decidido un lado, vamos recto
    if side is None:
        return base_speed, base_speed

    front_center = readings[3]
    
    # Lógica dinámica según el lado elegido
    if side == 'right':
        front_side = readings[5]
        lateral = readings[7]
        if front_center < 0.4 or front_side < 0.3:
            return -0.3, 0.3 # Giro a la izquierda para seguir pared derecha
        if lateral < (target_dist - 0.05): 
            return base_speed * 0.5, base_speed * 1.0
        elif lateral >= max_dist:
            return base_speed, base_speed # Ir recto si pierde la pared
        elif lateral > (target_dist + 0.05):
            return base_speed * 0.8, base_speed * 0.5
        else:
            return base_speed, base_speed

    elif side == 'left':
        front_side = readings[2]
        lateral = readings[15]
        if front_center < 0.4 or front_side < 0.3:
            return 0.3, -0.3 # Giro a la derecha para seguir pared izquierda
        if lateral < (target_dist - 0.05):
            return base_speed * 1.0, base_speed * 0.5
        elif lateral >= max_dist:
            return base_speed, base_speed
        elif lateral > (target_dist + 0.05):
            return base_speed * 0.5, base_speed * 0.8
        else:
            return base_speed, base_speed
            
    return 0.0, 0.0

def main(args=None):
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')
    coppelia.start_simulation()
    
    # Empezamos sin un lado definido
    wall_side = None 
    
    while coppelia.is_running():
        readings = robot.get_sonar()
        
        # --- LÓGICA DE DECISIÓN DINÁMICA ---
        if wall_side is None:
            # Si detectamos algo de frente...
            if readings[3] < 0.6: 
                # Comparamos sensores diagonales: ¿está la pared más a la izquierda o derecha?
                if readings[2] < readings[5]:
                    wall_side = "left"
                    print("Pared detectada: Decido seguir por la IZQUIERDA")
                else:
                    wall_side = "right"
                    print("Pared detectada: Decido seguir por la DERECHA")
        # ----------------------------------

        lspeed, rspeed = avoid(readings, side=wall_side)
        robot.set_speed(lspeed, rspeed)
        
    coppelia.stop_simulation()

if __name__ == '__main__':
    main()