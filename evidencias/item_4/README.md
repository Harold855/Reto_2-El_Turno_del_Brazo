# Evidencias y explicación de la reconstruccion del Ítem 4 — Auditoría de cinemática inversa

## Contexto

Durante la sesión física con el JetCobot se ejecutó el Ítem 4 del Reto 2 utilizando `herramientas/auditar_ik.py`.

El procedimiento realizado fue:

1. Definir un objetivo cartesiano seguro.
2. Enviar el objetivo al JetCobot mediante `send_coords()`.
3. Permitir que el firmware resolviera la cinemática inversa.
4. Leer la configuración articular realmente ejecutada mediante `get_angles()`.
5. Convertir los ángulos de grados a radianes.
6. Evaluar dicha configuración con la función `fk(q)` desarrollada por el equipo.
7. Comparar `FK(q_real)` con el objetivo cartesiano solicitado.
8. Calcular el error cartesiano total.

## Resultado de la prueba física

Objetivo cartesiano:

- X = 129.3 mm
- Y = 11.4 mm
- Z = 398.8 mm

Orientación:

- Rx = -92.46°
- Ry = -22.42°
- Rz = -39.03°

Configuración articular ejecutada por el brazo:

- q [°] = (25.83, -19.42, 0.17, 13.79, 24.43, -23.29)

Configuración convertida a radianes:

- q [rad] = (0.4508, -0.3389, 0.0030, 0.2407, 0.4264, -0.4065)

Resultado de la FK propia:

- FK(q_real) = (127.2813, 14.1495, 399.9999) mm

Pose reportada por el firmware mediante `get_coords()`:

- (126.8, 11.9, 394.6) mm

Errores respecto al objetivo:

- Error X = -2.0187 mm
- Error Y = +2.7495 mm
- Error Z = +1.1999 mm
- Error cartesiano total = 3.6159 mm

Criterio cumplido del reto:

- Error requerido ≤ 10 mm

## Sobre `auditoria_ik_reconstruida.csv`

El archivo original `auditoria_ik.csv` fue generado durante las pruebas en el laboratorio mediante la opción `--guardar`. Sin embargo, no se llegaron a 
preservar posteriormente en el repositorio. Por esta razón y por falta de tiempo, se creo esta reconstrucción

Este archivo fue reconstruido utilizando:

- los valores registrados durante las pruebas;
- las fotos de evidencia de la sesión(solo se subio "auditoria_ik_3_62mm" como evidencia);
- la estructura exacta de columnas definida en `herramientas/auditar_ik.py`;
- la misma función `fk(q)` del repositorio para recalcular la posición cartesiana.

**Nota:** No se simuló ningún movimiento del robot, para la reconstrucción.

## Sobre `get_coords()`

`get_coords()` se conserva únicamente como dato de apoyo.

La evidencia principal es la comparación entre:

`FK(q_real)` y el objetivo cartesiano solicitado.

La diferencia entre `FK(q_real)` y `get_coords()` nos dio aproximadamente:

- 5.8695 mm

## Interpretación de la solución de IK

El firmware eligió una configuración articular relativamente cercana a la postura inicial del robot, manteniendo continuidad de movimiento.

La API de `pymycobot` no expone el criterio interno exacto utilizado para seleccionar entre distintas soluciones de cinemática inversa.
Por tanto, la continuidad articular, los límites articulares o criterios internos del firmware se consideran explicaciones posibles.
