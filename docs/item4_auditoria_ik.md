# Ítem 4 — La FK como instrumento que audita una IK ajena

| Parte | Estado |
|---|---|
| `herramientas/auditar_ik.py` (`error_cartesiano`, `grados_a_radianes`, flujo completo) | Completado y probado sin el brazo |
| FK propia (`fk.py`) y su validación (ítem 1: 5.8 / 5.9 / 5.9 mm) | Completado |
| CSV de evidencia (encabezado de 24 columnas) | Completado; sin filas porque no se hizo la sesión con el robot |
| Sesión con el brazo (tabla de la §5 y explicación de la §7) | No realizada; se describe como se hubiera hecho |


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

`evidencias/item_4/auditoria_ik.csv` (las 16 primeras columnas son las acordadas; el resto es apoyo):

```
x_obj,y_obj,z_obj, q1_deg…q6_deg, x_fk,y_fk,z_fk, x_robot,y_robot,z_robot, error_mm,
rx_obj,ry_obj,rz_obj, velocidad, ex_mm,ey_mm,ez_mm, dif_fk_robot_mm
```

## 4. Objetivo y procedimiento que se hubieran usado

Se hubiera **reproducido un punto que el firmware ya alcanzó** en el ítem 1: la pose `ready` quedó en
`get_coords()` = (100.9, −40.5, 395.4) mm, dentro del espacio de trabajo y de los límites articulares.

1. Se hubiera llevado el brazo a `ready` (`verificar_fk.py`) y leído `get_coords()` con
   `auditar_ik.py --solo-leer`, para obtener `x, y, z` y la orientación `rx, ry, rz` a usar.
2. Después se hubiera vuelto a una pose distinta (por ejemplo `cero`) y se hubiera pedido ese objetivo con
   `send_coords`, con espacio despejado, aviso en voz alta, velocidad 30 y el puerto serie libre.
3. Finalmente se hubiera comparado `FK(q_real)` con el objetivo. Si el firmware llegó, `e` debería rondar el
   desfase conocido de la FK (≈ 5.9 mm), un valor mucho mayor apuntaría a un objetivo inalcanzable con esa
   orientación o a un problema de lectura, no a la tabla DH.

## 5. Tabla de evidencia que se hubiera completado

| Variable | Valor |
|---|---|
| Objetivo solicitado X, Y, Z | (100.9, −40.5, 395.4) mm previsto; el real, el medido en la sesión |
| Orientación utilizada | la leída con `--solo-leer` en `ready` |
| Velocidad | 30 |
| `get_angles()` [°], `q_real` [rad], `get_coords()`, `FK(q_real)` [mm] | los de la sesión (no medidos) |
| Error X, Y, Z y error cartesiano total | los de la sesión; esperado ≈ 6 mm, criterio ≤ 10 mm |

## 6. Resultados esperados e interpretación

| Observación | Interpretación posible |
|---|---|
| `e ≤ 10 mm` | El firmware llegó al objetivo y la FK propia lo confirma |
| `e` grande, y `FK(q_real) ≈ get_coords()` | El firmware no alcanzó el objetivo (límites, objetivo inalcanzable con esa orientación) |
| `e` grande, y `FK(q_real)` difiere de `get_coords()` con una diferencia casi constante | Desfase de marco o de la tabla DH (p. ej. `d1`, `d5`, `d6` o el punto del efector), no un fallo de la IK |
| `FK(q_real) ≈ objetivo` pero `get_coords()` difiere | `get_coords()` usa otro marco o herramienta que la FK |

Como en el ítem 1, la FK queda a unos 5.8–5.9 mm de `get_coords()` con un desfase casi constante (≈ +5.5 mm en
z). Por eso se espera que `dif_fk_robot_mm` ronde esos ~6 mm; si fuera mucho mayor, habría un problema de marco o
de lectura, no de la IK.

## 7. Pregunta de la semana 5

¿Por qué el brazo eligió esta solución y no la del codo contrario? 
Con los datos de la auditoría se hubiera descrito la configuración adoptada (signos de `q2`, `q3`, `q5`) y se hubiera contrastado con lo que documenta el
fabricante. Por lo tanto, no hay una configuración medida que explicar.
