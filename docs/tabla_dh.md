# Documento de diseño previo — Tabla DH del JetCobot (Ítem 1)

> **Estado:** la tabla vigente es la **del manual oficial** (la de `main`). Las predicciones nuevas
> están en la sección 5; la validación de 5.8 / 5.9 / 5.9 mm de la sección 6 se hizo con la **tabla
> anterior** y hay que **repetirla** con la nueva. Quedan por completar los campos `[ ... ]` y la firma.
> La FK también sirve de instrumento en el ítem 4 (`docs/item4_auditoria_ik.md`).

| Campo | Valor |
|---|---|
| Reto | RB-2 · El Turno del Brazo |
| Equipo n.º | `[ ]` (ROS_DOMAIN_ID = 42 + n.º) |
| Integrantes | `[ ]`, `[ ]`, `[ ]`, `[ ]` |
| Fecha del borrador | `[ ]` |
| Fuente de la tabla | Manual oficial del myCobot 280 / JetCobot (Elephant Robotics, Yahboom). Reemplaza la deducción previa en pizarra |
| Implementación | `src/arm_broker/arm_broker/fk.py` (`DH`, `fk_matriz`, `fk`) |

## 1. Convención

Denavit-Hartenberg **estándar** (Craig/Siciliano-Spong). Cada eslabón:

```
A_i = Rot_z(θ_i) · Trans_z(d_i) · Trans_x(a_i) · Rot_x(α_i)
```

```
        | cosθ  -sinθ·cosα   sinθ·sinα   a·cosθ |
A_i  =  | sinθ   cosθ·cosα  -cosθ·sinα   a·sinθ |
        |  0        sinα         cosα        d    |
        |  0         0            0          1    |
```

Cadena completa: `T_0_6 = A1 · A2 · A3 · A4 · A5 · A6`.
La posición del efector es la última columna: `(x, y, z) = T_0_6[0:3, 3]`.

Unidades: longitudes en **mm**, ángulos en **rad** dentro del código (la tabla usa grados).
El ángulo DH es `θ_i = q_i + offset_i`, donde `q_i` es la lectura articular del robot.

## 2. Tabla DH (manual oficial del JetCobot)

| i | θ_i | d_i [mm] | a_i [mm] | α_i | offset |
|:-:|:---|---:|---:|---:|---:|
| 1 | q1 | 131.22 | 0 | +90° | 0° |
| 2 | q2 − 90° | 0 | −110.4 | 0° | −90° |
| 3 | q3 | 0 | −96 | 0° | 0° |
| 4 | q4 − 90° | 63.4 | 0 | +90° | −90° |
| 5 | q5 + 90° | 75.05 | 0 | −90° | +90° |
| 6 | q6 | 45.6 | 0 | 0° | 0° |

Correspondencia con el código (`fk.py`, columnas `alpha, a, d, offset_theta`):

```python
DH = [
    ( math.pi / 2,    0.0, 131.22,           0.0),   # J1
    (         0.0, -110.4,   0.00, -math.pi / 2),    # J2
    (         0.0,  -96.0,   0.00,           0.0),   # J3
    ( math.pi / 2,    0.0,  63.40, -math.pi / 2),    # J4
    (-math.pi / 2,    0.0,  75.05,  math.pi / 2),    # J5
    (         0.0,    0.0,  45.60,           0.0),   # J6
]
```

**Valores descartados (NO usar):** `d1 = 134.75`, `a2 = −110`, `d5 = 75.55`, `d6 = 50`. Salían de la
deducción previa en pizarra (medidas del manual más el cálculo de los vectores x, y, z de cada
articulación) y no coinciden con el manual oficial. Con ellos se hizo la primera validación (sección 6).

## 3. Esquema de marcos

Ayuda de lectura de la cadena, de la base al efector:

```
{0} base ── d1 = 131.22 ─▶ J1 (giro vertical)
   └─ a2 = −110.4 ─▶ J2 (hombro)
        └─ a3 = −96 ─▶ J3 (codo)
             └─ d4 = 63.4 ─▶ J4
                  └─ d5 = 75.05 ─▶ J5
                       └─ d6 = 45.6 ─▶ J6 → efector
```

Pose cero (q = 0): el brazo queda estirado hacia arriba; la FK da `(45.6, −63.4, 412.7) mm`, donde
`x = d6` y `y = −d4`. Comprobación a mano: `z = d1 + |a2| + |a3| + d5 = 131.22 + 110.4 + 96 + 75.05 = 412.67 mm`.

### Origen y punto del efector

- **Origen {0}:** centro del eje de J1 a nivel de la superficie de apoyo del robot; `z = 0` ahí.
- **Punto medido:** el origen del marco {6} (centro de la brida de J6), es decir, la última columna de
  `T_0_6` que devuelve `fk(q)`. No es la punta de la pinza ni de ningún accesorio.
- `get_coords()` del firmware puede usar otro origen o herramienta; un desfase casi constante entre
  `FK(q_real)` y `get_coords()` es compatible con eso.

### Puntos a confirmar antes de firmar

- [ ] **Medición independiente (opcional).** La validación es contra `get_coords()`; si el docente pide
      regla, medir el punto definido arriba en las tres poses.
- [ ] **Repetir la validación del ítem 1** con esta tabla (sección 6).

## 4. Límites y espacio de trabajo usados por la admisión (Ítem 2)

| Articulación | Mínimo [rad] | Máximo [rad] |
|---|---:|---:|
| J1 – J5 | −2.87979 (−165°) | +2.87979 (+165°) |
| J6 | −3.05433 (−175°) | +3.05433 (+175°) |

Fuente: límites publicados por Elephant Robotics para el myCobot 280.

- Alcance de admisión: `80 mm ≤ ‖(x, y, z)‖ ≤ 480 mm` y `z ≥ 0`, desde {0}. El alcance geométrico
  máximo de la cadena es `d1 + |a2| + |a3| + d5 + d6 = 131.22 + 110.4 + 96 + 75.05 + 45.6 ≈ 458.3 mm`, por
  lo que 480 mm deja margen.
- Radio de trabajo nominal del fabricante: 280 mm **desde J2** (zona de ±0.5 mm de repetibilidad y 250 g
  de carga). No es un límite cinemático: la pose cero está a ≈ 292 mm de J2 y a ≈ 415 mm de {0}.

## 5. Predicciones (declaradas ANTES de volver a medir)

Con la tabla del manual, `fk.fk(q)` y el `q` comandado
(`evidencias/item1/predicciones_dh_manual.txt`):

| Pose | q comandado [rad] | x_pred [mm] | y_pred [mm] | z_pred [mm] |
|---|---|---:|---:|---:|
| `cero` | [0, 0, 0, 0, 0, 0] | 45.60 | −63.40 | 412.67 |
| `ready` | [0, −0.5, 0.5, 0, 0.5, 0] | 92.95 | −41.54 | 399.16 |
| `girada` | [0.6, −0.4, 0.4, 0, 0.3, 0] | 99.63 | 7.67 | 403.96 |
| `baja` (adicional) | [0, −1.2, 1.2, 0, 0, 0] | 148.50 | −63.40 | 342.27 |

Criterio: error ≤ 10 mm contra el robot en las tres poses del ítem 1 (`cero`, `ready`, `girada`).

## 6. Resultados de la medición

### 6.1 Con la tabla vigente (manual): pendiente

| Pose | FK(q_real) [mm] | Robot, `get_coords()` [mm] | Error [mm] | ¿≤ 10 mm? |
|---|---|---|---:|:-:|
| `cero` | `[ ]` | `[ ]` | `[ ]` | `[ ]` |
| `ready` | `[ ]` | `[ ]` | `[ ]` | `[ ]` |
| `girada` | `[ ]` | `[ ]` | `[ ]` | `[ ]` |

Se obtiene con `python3 herramientas/verificar_fk.py` en el Jetson (el script lee `get_angles()`, calcula
`FK(q_real)` y compara con `get_coords()`).

### 6.2 Histórico, con la tabla anterior (pizarra)

Datos en `evidencias/item1/validacion_fk.txt`; predicciones en
`evidencias/item1/predicciones_antes_de_medir.txt` (commit `a0cbc35`, 2026-09-30 20:10, anterior a la
validación `84e9f1a`, 21:01).

| Pose | FK(q_real) [mm] | Robot, `get_coords()` [mm] | Error [mm] |
|---|---|---|---:|
| `cero` | (55.9, −62.6, 414.6) | (54.4, −63.2, 409.1) | 5.8 |
| `ready` | (102.1, −38.6, 400.9) | (100.9, −40.5, 395.4) | 5.9 |
| `girada` | (108.4, 14.6, 404.4) | (108.2, 11.9, 399.0) | 5.9 |

**Estos valores no son válidos para la tabla nueva**: la FK de `cero` pasa de (50.0, −63.4, 416.3) a
(45.6, −63.4, 412.7). El error era casi constante (≈ +5.5 mm en z), compatible con un desfase de marco o
de herramienta; sirve de referencia para interpretar la nueva medición. «Robot» es lo que reporta el
firmware con `get_coords()`, no una medición independiente con regla. El brazo tampoco llega exacto al
`q` comandado, por eso se compara con `q_real` (`get_angles()`).

## 7. Firma

Firmado antes de ejecutar las mediciones:
`[Nombre 1]` · `[Nombre 2]` · `[Nombre 3]` · `[Nombre 4]` — Fecha/hora: `[ ]`

---

*Este documento se completa luego con: diagrama de secuencia (Ítem 2) y la predicción del p95
por política (Ítem 3), que forman parte del mismo «documento de diseño previo».*
