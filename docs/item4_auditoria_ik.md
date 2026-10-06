# Ítem 4 — FK como instrumento que audita una IK ajena

| Parte | Estado |
|---|---|
| `herramientas/auditar_ik.py` (`error_cartesiano`, `grados_a_radianes`, flujo completo) | Completado, probado sin brazo y luego **usado con el brazo** |
| FK propia (`fk.py`) y su validación (ítem 1: 5.8 / 5.9 / 5.9 mm) | Completado |
| CSV de evidencia `evidencias/item_4/auditoria_ik.csv` | Completado: una fila con la auditoría oficial (reconstruida debido a que el original no se guardo por llamar mal el nombre de la carpeta) |
| Sesión con el brazo (diagnóstico, objetivo cartesiano, error y pregunta de la semana 5) | **Realizada el 06 de octubre**; error cartesiano **3.62 mm ≤ 10 mm** |

El equipo **no escribe un solver de cinemática inversa**, pues la IK la resuelve el firmware con `send_coords()`.

> **Nota sobre las fechas.** El ítem 4 había quedado cerrado sin la sesión con el robot por la fecha límite de
> entrega (05 de octubre). El lunes se logró tener un espacio para usar el brazo, y esta
> auditoría, es evidencia de ello.

## 1. Qué se demuestra

```
1) El firmware recibe un objetivo cartesiano (x, y, z)
        ↓
2) El firmware resuelve la cinemática inversa
        ↓
3) El brazo adopta una configuración articular q
        ↓
4) Se lee q          get_angles()  [°]  →  q_real [rad]
        ↓
5) Se aplica la FK propia   FK(q_real) = (X_FK, Y_FK, Z_FK)  [mm]
        ↓
6) Se comprueba si realmente llegó al objetivo pedido con
        e = √((X_FK − X_obj)² + (Y_FK − Y_obj)² + (Z_FK − Z_obj)²)
```

La evidencia principal es `FK(q_real)` frente al objetivo: se usa una FK propia para auditar una IK ajena.
`get_coords()` se guarda solo como dato de apoyo, porque es lo que el propio firmware dice que alcanzó, no una
medición independiente. Criterio: **e ≤ 10 mm**.

## 2. Herramienta

`herramientas/auditar_ik.py` (se usa en el Jetson, con el puerto serie libre):

```bash
python3 herramientas/auditar_ik.py --plantilla     # tabla vacía; no necesita el brazo
python3 herramientas/auditar_ik.py --solo-leer     # no mueve: diagnóstico FK(q) vs get_coords()
python3 herramientas/auditar_ik.py --x X --y Y --z Z --rx RX --ry RY --rz RZ \
    --velocidad 30 --confirmo-espacio-despejado --guardar
```

Pasos que ejecuta: define el objetivo → `send_coords([x, y, z, rx, ry, rz], velocidad, modo)` → espera →
`get_angles()` → grados a radianes → `fk.fk(q_real)` → `get_coords()` como apoyo → error → imprime la tabla y,
con `--guardar`, agrega una fila a `evidencias/item_4/auditoria_ik.csv`.

Pruebas sin brazo (`tests/test_auditar_ik.py`): `error_cartesiano((0,0,0),(3,4,0)) == 5.0`,
`error_cartesiano((200,50,180),(198,53,184)) = √29 ≈ 5.385`, conversión grados↔radianes, el registro de
evidencia y el flujo completo con un brazo simulado.

## 3. Datos que se guardan

`evidencias/item_4/auditoria_ik.csv` (las 16 primeras columnas son las acordadas, el resto es solo apoyo):

```
x_obj,y_obj,z_obj, q1_deg…q6_deg, x_fk,y_fk,z_fk, x_robot,y_robot,z_robot, error_mm,
rx_obj,ry_obj,rz_obj, velocidad, ex_mm,ey_mm,ez_mm, dif_fk_robot_mm
```

## 4. Sesión con el brazo (06 de octubre)

Se corrió en el Jetson (`172.51.1.20`), con el puerto serie libre, con espacio despejado y velocidad baja.

### 4.1 Plantilla y diagnóstico previo

1. Se recreó la carpeta de evidencia (`mkdir -p evidencias/item_4`) y se ejecutó
   `python3 herramientas/auditar_ik.py --plantilla`. Imprimió la tabla de evidencia vacía (objetivo,
   orientación, velocidad, `get_angles()`, `q_real`, `get_coords()`, `FK(q_real)`, errores por eje y total), lo
   que confirma que la herramienta estaba lista.
2. Se ejecutó `python3 herramientas/auditar_ik.py --solo-leer`, que **no mueve el brazo**: lee la pose actual y
   compara la FK propia con `get_coords()`.

| Dato del diagnóstico | Valor |
|---|---|
| `get_angles()` [°] | (33.48, −24.43, 23.11, −1.05, 16.52, −23.11) |
| `q_real` [rad] | (0.5843, −0.4264, 0.4033, −0.0183, 0.2883, −0.4033) |
| `get_coords()` [mm] | (109.3, 11.4, 398.8) |
| `FK(q_real)` [mm] | (109.5, 13.4, 404.4) |
| Error por eje (FK − robot) | X = +0.18 · Y = +2.04 · Z = +5.58 mm |
| Error total | **5.94 mm** (< 10 mm) |

Es una prueba diagnóstica previa, no la evidencia del ítem. Coincide con el desfase del ítem 1 (≈ 5.9 mm, casi
constante, ≈ +5.5 mm en z): la pose de partida quedó cerca de `girada`.

### 4.2 Objetivo cartesiano

Se leyó la pose actual con orientación, `[X, Y, Z, RX, RY, RZ] = [109.3, 11.4, 398.8, −92.46, −22.42, −39.03]`, y
se definió un objetivo cercano y seguro, **20 mm en X** y la misma orientación:

`[129.3, 11.4, 398.8, −92.46, −22.42, −39.03]` (ΔX = +20 mm, ΔY = 0, ΔZ = 0).

```bash
python3 herramientas/auditar_ik.py \
  --x 129.3 --y 11.4 --z 398.8 \
  --rx -92.46 --ry -22.42 --rz -39.03 \
  --velocidad 20 --modo 0 --espera 5 \
  --confirmo-espacio-despejado --guardar
```

## 5. Resultado oficial

El firmware resolvió la IK, movió el brazo y adoptó esta configuración:

| Variable | Valor |
|---|---|
| Objetivo solicitado X, Y, Z | (129.3, 11.4, 398.8) mm |
| Orientación utilizada | (−92.46, −22.42, −39.03) |
| Velocidad / modo | 20 / 0 |
| `get_angles()` [°] | (25.83, −19.42, 0.17, 13.79, 24.43, −23.29) |
| `q_real` [rad] | (0.4508, −0.3389, 0.0030, 0.2407, 0.4264, −0.4065) |
| `get_coords()` [mm] (apoyo) | (126.8, 11.9, 394.6) |
| `FK(q_real)` [mm] | (127.3, 14.1, 400.0) |
| Error X (FK − objetivo) | −2.02 mm |
| Error Y | +2.75 mm |
| Error Z | +1.20 mm |
| **Error cartesiano total** | **3.62 mm ≤ 10 mm → cumple** |
| Diferencia FK − `get_coords()` | 5.87 mm (≈ el desfase del ítem 1) |

La fila está en `evidencias/item_4/auditoria_ik.csv`. **Es una reconstrucción**: el CSV que generó `--guardar` en el
laboratorio no se preservó en el repositorio, así que se rehízo con los valores registrados, la estructura de
columnas de `auditar_ik.py` y la misma `fk(q)` del repositorio, sin simular ningún movimiento (copia idéntica en
`auditoria_ik_reconstruida.csv`). Respaldo: `evidencias/item_4/README.md`, `auditoria_final_reconstruida.txt` y la
captura `auditoria_ik_3_62mm.jpeg`. La evidencia principal es `FK(q_real)` frente al
objetivo; `get_coords()` es solo dato de apoyo. El error de 3.62 mm es menor que el desfase de 5.9 mm entre la FK
y `get_coords()` porque el objetivo se compara con la FK, no con lo que el firmware dice haber alcanzado.

## 6. Resultados esperados e interpretación

| Observación | Interpretación posible |
|---|---|
| `e ≤ 10 mm` | El firmware llegó al objetivo y la FK propia lo confirma |
| `e` grande, y `FK(q_real) ≈ get_coords()` | El firmware no alcanzó el objetivo (límites, objetivo inalcanzable con esa orientación) |
| `e` grande, y `FK(q_real)` difiere de `get_coords()` con una diferencia casi constante | Desfase de marco o de la tabla DH (p. ej. `d1`, `d5`, `d6` o el punto del efector), no un fallo de la IK |
| `FK(q_real) ≈ objetivo` pero `get_coords()` difiere | `get_coords()` usa otro marco o herramienta que la FK |

En la sesión: `e = 3.62 mm` (primera fila) y `dif_fk_robot_mm = 5.87 mm`, como se esperaba (~6 mm).

## 7. Pregunta: ¿por qué esa solución y no la del codo contrario?

Configuración inicial frente a la que eligió el firmware:

| Articulación | Inicial [°] | Seleccionada [°] | Cambio |
|---|---:|---:|---:|
| J1 | 33.48 | 25.83 | −7.65° |
| J2 | −24.43 | −19.42 | +5.01° |
| J3 | 23.11 | 0.17 | −22.94° |
| J4 | −1.05 | 13.79 | +14.84° |
| J5 | 16.52 | 24.43 | +7.91° |
| J6 | −23.11 | −23.29 | −0.18° |

**Explicación:** 

El firmware seleccionó una solución de IK cercana a la configuración articular inicial,
manteniendo continuidad de movimiento en lugar de realizar un cambio brusco hacia una solución alternativa de
codo contrario. Se ve en que los signos de `q2`, `q3` y `q5` no cambiaron (`q2 < 0`, `q3 ≥ 0`, `q5 > 0` antes y
después) y que el mayor cambio fue de unos 23° en J3. La API de `pymycobot` no expone el criterio interno exacto
con el que el firmware escoge la rama, por lo que esta conclusión se basa en los ángulos iniciales y finales
observados y no en el algoritmo interno del firmware. No se calculó la otra solución con un solver propio.

Nota Final: Parte de la estructura de esta documentación se realizó con IA, para poder organizar mejor los resultados y la descripción de la auditoria. Sin embargo, por nuestra parte, realizamos modificaciones en lo que estaba mal generado y calculado. Luego, se detallaron los resultados de las pruebas y se comentó para una mejor explicación.
