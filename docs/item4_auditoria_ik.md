# Ítem 4 — La FK como instrumento que audita una IK ajena

> **Estado: herramienta, pruebas y documento listos; falta la sesión con el robot.** El equipo **no
> escribe un solver de cinemática inversa**: la IK la resuelve el firmware con `send_coords()`.
> `evidencias/item_4/auditoria_ik.csv` contiene solo el encabezado: **no se inventan filas**; se llena
> con `--guardar` durante la sesión (ver §7).

| Parte | Estado |
|---|---|
| `herramientas/auditar_ik.py` (`error_cartesiano`, `grados_a_radianes`, flujo completo) | Hecho y probado sin robot (4 pruebas en `tests/test_auditar_ik.py`) |
| FK propia (`fk.py`) y su validación (ítem 1: 5.8 / 5.9 / 5.9 mm) | Hecho |
| CSV de evidencia (encabezado de 24 columnas) | Hecho; sin filas hasta medir |
| Objetivo seguro sobre el tablero, medición con el robot, tabla de la §4, explicación de la §6 | **Pendiente (requiere el robot)** |

## 1. Qué se demuestra

```
El firmware recibe un objetivo cartesiano (x, y, z)
        ↓
El firmware resuelve la cinemática inversa
        ↓
El brazo adopta una configuración articular q
        ↓
Nosotros leemos q          get_angles()  [°]  →  q_real [rad]
        ↓
Aplicamos nuestra FK       FK(q_real) = (X_FK, Y_FK, Z_FK)  [mm]
        ↓
Comprobamos si realmente llegó al objetivo pedido
        e = √((X_FK − X_obj)² + (Y_FK − Y_obj)² + (Z_FK − Z_obj)²)
```

La **evidencia principal** es `FK(q_real)` frente al objetivo: demuestra que se usa una FK propia
para auditar una IK ajena. `get_coords()` se guarda solo como dato de apoyo, porque es lo que el
propio firmware dice que alcanzó, no una medición independiente. Criterio: **e ≤ 10 mm**.

## 2. Herramienta

`herramientas/auditar_ik.py` (corre en el Jetson, con el puerto serie libre):

```bash
python3 herramientas/auditar_ik.py --plantilla     # tabla vacía; no necesita el robot
python3 herramientas/auditar_ik.py --solo-leer     # no mueve: diagnóstico FK(q) vs get_coords()
python3 herramientas/auditar_ik.py --x X --y Y --z Z --rx RX --ry RY --rz RZ \
    --velocidad 30 --confirmo-espacio-despejado --guardar
```

Protecciones: **no hay objetivo por defecto** y el brazo no se mueve sin `--x --y --z --rx --ry --rz`
y sin `--confirmo-espacio-despejado`. El punto físico seguro sobre el tablero se define en la sesión
(se mide y se anota antes), no se fija aquí. `--solo-leer` sin objetivo es un diagnóstico y no se
guarda como evidencia.

Pasos que ejecuta: define el objetivo → `send_coords([x, y, z, rx, ry, rz], velocidad, modo)` →
espera → `get_angles()` → grados a radianes → `fk.fk(q_real)` → `get_coords()` como apoyo → error →
imprime la tabla y, con `--guardar`, agrega una fila a `evidencias/item_4/auditoria_ik.csv`.

Probado sin robot (`test_auditar_ik.py`): `error_cartesiano((0,0,0),(3,4,0)) == 5.0`,
`error_cartesiano((200,50,180),(198,53,184)) = √29 ≈ 5.385`, conversión grados↔radianes, el
registro de evidencia y el flujo completo con un brazo simulado.

## 3. Datos que se guardan

`evidencias/item_4/auditoria_ik.csv` (las 16 primeras columnas son las acordadas; el resto es apoyo):

```
x_obj,y_obj,z_obj, q1_deg…q6_deg, x_fk,y_fk,z_fk, x_robot,y_robot,z_robot, error_mm,
rx_obj,ry_obj,rz_obj, velocidad, ex_mm,ey_mm,ez_mm, dif_fk_robot_mm
```

### Objetivo recomendado para la primera corrida

En vez de inventar un punto, conviene **reproducir uno que el firmware ya alcanzó** en el ítem 1: la
pose `ready` quedó en `get_coords()` = (100.9, −40.5, 395.4) mm, dentro del espacio de trabajo y de
los límites articulares. Procedimiento:

1. Llevar el brazo a `ready` (`verificar_fk.py`) y leer `get_coords()` con `auditar_ik.py --solo-leer`:
   dan `x, y, z` **y la orientación** `rx, ry, rz` a usar.
2. Volver a una pose distinta (p. ej. `cero`) y pedir ese objetivo con `send_coords`.
3. Comparar `FK(q_real)` con el objetivo. Si el firmware llegó, `e` debe rondar el desfase conocido de
   la FK (≈ 5.9 mm); un valor mucho mayor apunta a objetivo inalcanzable con esa orientación o a un
   problema de lectura, no a la tabla DH.

Los valores finales se miden y se anotan en la sesión; los de arriba son una guía, no una medición.

## 4. Tabla de evidencia (la imprime el script; aquí, la plantilla)

| Variable | Valor |
|---|---|
| Objetivo solicitado X, Y, Z | pendiente |
| Orientación utilizada | pendiente |
| Velocidad | pendiente |
| `get_angles()` [°] | pendiente |
| `q_real` [rad] | pendiente |
| `get_coords()` | pendiente |
| `FK(q_real)` [mm] | pendiente |
| Error X (FK − objetivo) | pendiente |
| Error Y | pendiente |
| Error Z | pendiente |
| Error cartesiano total | pendiente |

## 5. Cómo leer el resultado

| Observación | Interpretación probable |
|---|---|
| `e ≤ 10 mm` | El firmware llegó al objetivo y la FK propia lo confirma |
| `e` grande, y `FK(q_real) ≈ get_coords()` | El firmware no alcanzó el objetivo (límites, objetivo inalcanzable con esa orientación) |
| `e` grande, y `FK(q_real)` difiere de `get_coords()` con una diferencia casi constante | Desfase de marco o de la tabla DH (p. ej. `d1`, `d5`, `d6` o el punto del efector), no un fallo de la IK |
| `FK(q_real) ≈ objetivo` pero `get_coords()` difiere | `get_coords()` usa otro marco o herramienta que la FK |

Referencia del ítem 1: la FK propia queda a unos 5.8-5.9 mm de `get_coords()` con un desfase casi constante
(≈ +5.5 mm en z). Por eso en el ítem 4 se espera que `dif_fk_robot_mm` ronde esos ~6 mm; si es mucho
mayor, hay un problema de marco o de lectura, no de la IK.

## 6. Pregunta abierta de la semana 5

¿Por qué el brazo eligió esta solución y no la del codo contrario? Con los datos de esta auditoría se
describe la configuración adoptada (signos de `q2`, `q3`, `q5`) y se contrasta con lo que documenta
el fabricante; no se calcula la otra solución con un solver propio.

- Explicación de la solución elegida por el brazo: `[completar tras medir]`

## 7. Lista de verificación para la sesión

- [ ] Fijar y **medir físicamente** un objetivo seguro sobre el tablero; anotar `x, y, z` y la orientación.
- [ ] Espacio despejado y aviso en voz alta; velocidad baja (30).
- [ ] Puerto serie libre (sin otro programa conectado al brazo).
- [ ] Correr con `--guardar` y conservar la tabla impresa (captura o log).
- [ ] Completar la tabla de la sección 4 y la explicación de la sección 6.
- [ ] Subir `evidencias/item_4/auditoria_ik.csv` con al menos una fila real y dejar la captura
      de la tabla impresa junto al CSV.
- [ ] Anotar en `docs/cierre_reflexivo.md` el error obtenido y la explicación de la §6.
