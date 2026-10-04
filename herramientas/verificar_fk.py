#!/usr/bin/env python3
""" Verificación de la FK contra el brazo real — Ítem 1 del Reto 2 """

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'arm_broker'))
from arm_broker import fk                                        # noqa: E402

# Se corre EN EL JETSON, con el puerto serie libre (sin otro programa conectado al brazo)
#   python3 verificar_fk.py                 # lleva el brazo a las poses por defecto
#   python3 verificar_fk.py --solo-leer     # no mueve: compara donde esté el brazo
# Criterio del reto: error ≤ 10 mm
CRITERIO_MM = 10.0

# 1. Poses de prueba
# Todas las columnas estan en radianes (q1..q6). Las tres primeras son las del ítem 1;
# 'baja' se probó también pero no se cuenta (ver docs/tabla_dh.md, sección 6)
POSES = {
    'cero':    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ready':   [0.0, -0.5, 0.5, 0.0, 0.5, 0.0],
    'girada':  [0.6, -0.4, 0.4, 0.0, 0.3, 0.0],
    'baja':    [0.0, -1.2, 1.2, 0.0, 0.0, 0.0],
}


# 2. Programa principal
def main():
    """Se lleva el brazo a cada pose, se lee lo que adoptó y se compara FK(q_real) con get_coords()"""
    """Se compara con q_real (get_angles) y no con el q comandado: el brazo no llega exacto al comandado"""
    ap = argparse.ArgumentParser()
    ap.add_argument('--puerto', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=1000000)
    ap.add_argument('--velocidad', type=int, default=30)     # Velocidad de send_angles, 1..100
    ap.add_argument('--espera', type=float, default=5.0)     # Segundos hasta que el brazo termina de moverse
    ap.add_argument('--solo-leer', action='store_true')      # No mueve: solo compara donde está
    args = ap.parse_args()

    try:
        from pymycobot.mycobot import MyCobot
    except ImportError:
        from pymycobot import MyCobot

    brazo = MyCobot(args.puerto, args.baud)
    print(f'{"pose":10} {"FK (x,y,z)":>26}  {"robot (x,y,z)":>26}  {"error":>8}')
    print('-' * 78)

    errores = []
    poses = {'donde este': None} if args.solo_leer else POSES

    for nombre, q_rad in poses.items():
        # 2.1 Mover (el robot espera grados) y esperar a que termine
        if q_rad is not None:
            brazo.send_angles([v * 180.0 / math.pi for v in q_rad], args.velocidad)
            time.sleep(args.espera)

        # 2.2 Leer lo que el brazo realmente adoptó
        leidos = brazo.get_angles()
        coords = brazo.get_coords()
        if not leidos or not coords or len(coords) < 3:
            print(f'{nombre:10}  el brazo no respondió; repite')
            continue

        # 2.3 FK propia sobre q_real (grados -> rad) frente a la posición que reporta el robot
        q_real = [v * math.pi / 180.0 for v in leidos]
        px, py, pz = fk.fk(q_real)
        rx, ry, rz = coords[0], coords[1], coords[2]
        err = math.sqrt((px - rx) ** 2 + (py - ry) ** 2 + (pz - rz) ** 2)
        errores.append(err)
        marca = 'OK' if err <= CRITERIO_MM else '>10mm'
        print(f'{nombre:10} ({px:7.1f},{py:7.1f},{pz:7.1f})  '
              f'({rx:7.1f},{ry:7.1f},{rz:7.1f})  {err:6.1f} {marca}')

    # 2.4 Resumen y orientación para interpretar el error
    if errores:
        print('-' * 78)
        print(f'error medio {sum(errores)/len(errores):.1f} mm   '
              f'máximo {max(errores):.1f} mm   criterio del reto: ≤ {CRITERIO_MM:.0f} mm')
        if max(errores) > CRITERIO_MM:
            print('\nSi el error es grande y constante, la tabla DH necesita ajuste.')
            print('Si crece con la distancia, revisen los parámetros a_i.')


if __name__ == '__main__':
    main()
