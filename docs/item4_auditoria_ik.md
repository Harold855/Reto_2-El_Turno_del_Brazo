# Ítem 4 - FK como instrumento que audita una IK ajena

| Parte | Estado |
|---|---|
| `herramientas/auditar_ik.py` (`error_cartesiano`, `grados_a_radianes`, flujo completo) | Completado sin probar en el brazo |
| FK propia (`fk.py`) y su validación (ítem 1: 5.8 / 5.9 / 5.9 mm) | Completado |
| CSV de evidencia (encabezado de 24 columnas) | Completado |

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
5) Se aplica nuestro FK       FK(q_real) = (X_FK, Y_FK, Z_FK)  [mm]
        ↓
6) Se comprueba si realmente llegó al objetivo pedido con
        e = √((X_FK − X_obj)² + (Y_FK − Y_obj)² + (Z_FK − Z_obj)²)
```

## 2. Herramienta

`herramientas/auditar_ik.py` (funciona en el Jetson, con el puerto de serie libre):

```bash
python3 herramientas/auditar_ik.py --plantilla     # tabla vacía, completada sin el brazo
python3 herramientas/auditar_ik.py --solo-leer     # no se mueve: diagnóstico FK(q) vs get_coords()
```

Pasos que se ejecutan: primero se define el objetivo → usa `send_coords([x, y, z, rx, ry, rz], velocidad, modo)` →
luego espera → se usa`get_angles()` → se pasan los grados a radianes → se compara `fk.fk(q_real)` → `get_coords()` como apoyo → error →
despues imprime la tabla y con `--guardar`, agrega una fila a `evidencias/item_4/auditoria_ik.csv`.

Una prueba ficticia (`test_auditar_ik.py`): `error_cartesiano((0,0,0),(3,4,0)) == 5.0`,
`error_cartesiano((200,50,180),(198,53,184)) = √29 ≈ 5.385`, convirtiendo los grados a radianes, se obtiene el
registro de evidencia y el flujo completo con un brazo simulado.

## 3. Datos que se guardan

En `evidencias/item_4/auditoria_ik.csv` (las 16 primeras columnas son las acordadas; el resto solo se uso como apoyo):

```
x_obj,y_obj,z_obj, q1_deg…q6_deg, x_fk,y_fk,z_fk, x_robot,y_robot,z_robot, error_mm,
rx_obj,ry_obj,rz_obj, velocidad, ex_mm,ey_mm,ez_mm, dif_fk_robot_mm
```

### Objetivos que hubieramos implementado 

Se hubiera **reproducido un punto que el firmware ya alcanzó**, por ejemplo en nuestro ítem 1: la
pose `ready` quedó en `get_coords()` = (100.9, −40.5, 395.4) mm, dentro del espacio de trabajo y de
los límites articulares. 

**Procedimiento que se hubiera realizado**

1. Inicalmente se debia de llevar el brazo a `ready` (`verificar_fk.py`) y leer sus `get_coords()` con `auditar_ik.py --solo-leer`:
   dando con los `x, y, z` y la orientación `rx, ry, rz` a usar.
2. Después se huniera vuelto a una pose distinta (por ejemplo `cero`) y se pedía ese objetivo con `send_coords`.
3. Finalmente se compara `FK(q_real)` con el objetivo. Si el firmware llegó, `e` debe rondar el desfase conocido de
   FK (≈ 5.9 mm), con un valor mucho mayor apunta a objetivo inalcanzable con esa orientación o a un
   problema de lectura, no a la tabla DH.

## 4. Resultados esperados

| Observación | Interpretación posible |
|---|---|
| `e ≤ 10 mm` | El firmware llegó al objetivo y la FK propia lo confirma |
| `e` grande, y `FK(q_real) ≈ get_coords()` | El firmware no alcanzó el objetivo (límites, objetivo inalcanzable con esa orientación) |
| `e` grande, y `FK(q_real)` difiere de `get_coords()` con una diferencia casi constante | Desfase de marco o de la tabla DH (p. ej. `d1`, `d5`, `d6` o el punto del efector), no un fallo de la IK |
| `FK(q_real) ≈ objetivo` pero `get_coords()` difiere | `get_coords()` usa otro marco o herramienta que la FK |

Razon: Como en item 1, FK queda a unos 5.8-5.9 mm de `get_coords()` con un desfase casi constante
(≈ +5.5 mm en z). Por eso en el ítem 4 se espera que `dif_fk_robot_mm` ronde esos ~6 mm; si es mucho
mayor, hay un problema de marco o de lectura, no de la IK.

