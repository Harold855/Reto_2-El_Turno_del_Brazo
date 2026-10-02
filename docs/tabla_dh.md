# Documento de diseño previo — Tabla DH del JetCobot (Ítem 1) — v0.4

> **Estado: BORRADOR listo para firmar.** Los campos `[ ... ]` los rellena el
> equipo. Las predicciones de §5 ya están congeladas antes de medir.

| Campo | Valor |
|---|---|
| Reto | RB-2 · El Turno del Brazo |
| Equipo n.º | **06** (ROS_DOMAIN_ID = 92) |
| Integrantes | `[ ]`, `[ ]`, `[ ]`, `[ ]` |
| Fecha del borrador | `[ ]` |
| Fuente de la tabla | Manual oficial Elephant Robotics / Página de cinemática myCobot 280 |
| Implementación | `src/arm_broker/arm_broker/fk.py` (`DH`, `fk_matriz`, `fk`) |
| Versión de este documento | v0.4 (DH del manual, origen {0} y efector definidos, 3 poses congeladas) |

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

## 2. Tabla DH (Manual oficial myCobot 280)

| i | θ_i | d_i [mm] | a_i [mm] | α_i | offset |
|:-:|:---|---:|---:|---:|---:|
| 1 | q1 | 131.22 | 0 | +90° | 0° |
| 2 | q2 − 90° | 0 | −110.4 | 0° | −90° |
| 3 | q3 | 0 | −96 | 0° | 0° |
| 4 | q4 − 90° | 63.4 | 0 | +90° | −90° |
| 5 | q5 + 90° | 75.05 | 0 | −90° | +90° |
| 6 | q6 | 45.6 | 0 | 0° | 0° |

Correspondencia con `fk.py` (columnas `alpha, a, d, offset_theta`):

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

**Valores descartados del equipo (NO usar):** d1=134.75, a2=−110, d5=75.55, d6=50.
Provienen de una deducción previa en pizarra y no coinciden con el manual oficial.

## 3. Esquema de marcos

```
{0} base ── d1 = 131.22 ─▶ J1 (giro vertical)
   └─ a2 = −110.4 ─▶ J2 (hombro)
        └─ a3 = −96 ─▶ J3 (codo)
             └─ d4 = 63.4 ─▶ J4
                  └─ d5 = 75.05 ─▶ J5
                       └─ d6 = 45.6 ─▶ J6 → efector
```

Pose cero (q = 0): el brazo queda estirado hacia arriba. Como los offsets
colocan d6 sobre +x y d4 sobre −y, la FK da `(45.6, −63.4, 412.7) mm`.

### 3.bis Origen del marco {0} y punto del efector medido

- **Origen {0}**: centro del eje de J1, a nivel de la superficie de apoyo del
  robot. z = 0 en esa superficie.
- **Punto del efector medido**: centro geométrico de la brida de J6 — la cara
  frontal donde se monta el efector. Es el **origen del marco {6}**, es decir,
  la última columna de `T_0_6` que devuelve `fk(q)`.
- **NO se mide la punta del gripper** ni ningún accesorio montado.
- La pinza (a veces llamada "J7") no forma parte de la cadena DH ni de la
  medición del ítem 1.
- Procedimiento de medición sugerido: marcar con cinta o rotulador el centro
  de la cara frontal de J6, y proyectar su posición sobre la mesa para leer
  (x, y). La altura z se mide desde la superficie de apoyo.

## 4. Límites y espacio de trabajo usados por la admisión (Ítem 2)

| Articulación | Mínimo [rad] | Máximo [rad] |
|---|---:|---:|
| J1 – J5 | −2.87979 (−165°) | +2.87979 (+165°) |
| J6 | −3.05433 (−175°) | +3.05433 (+175°) |

Fuente: límites publicados por Elephant Robotics para el myCobot 280.

- **Alcance geométrico máximo** de la cadena desde {0}:
  d1 + |a2| + |a3| + d5 + d6 = 131.22 + 110.4 + 96 + 75.05 + 45.6 ≈ 458.3 mm.
- **Límite de admisión usado**: `80 mm ≤ ‖(x, y, z)‖ ≤ 480 mm`, con `z ≥ 0`.
  Medido desde {0}.
- **Radio de trabajo nominal del fabricante**: 280 mm, medido **desde J2**
  (no desde {0}). Describe la zona donde se garantizan ±0.5 mm de repetibilidad
  y 250 g de carga útil. **No es un límite cinemático**: la propia pose cero
  del robot (q = 0) está a ≈ 292 mm de J2 y a ≈ 420 mm de {0}, por encima de
  280 mm, y sin embargo es una configuración normal y segura.
- Por eso la admisión usa el alcance geométrico con margen (480 mm desde {0})
  y no rechaza configuraciones alcanzables físicamente.

## 5. Predicciones (declaradas ANTES de medir)

Calculadas con `fk.fk(q)` usando la tabla DH del manual oficial.

| Pose | q [rad] | x_pred | y_pred | z_pred |
|---|---|---:|---:|---:|
| `cero` (referencia) | [0, 0, 0, 0, 0, 0] | 45.6 | −63.4 | 412.7 |
| `ready` | [0, −0.5, 0.5, 0, 0.5, 0] | 92.9 | −41.5 | 399.2 |
| `girada` (referencia, NO congelada) | [0.6, −0.4, 0.4, 0, 0.3, 0] | 99.6 | 7.7 | 404.0 |
| `baja` | [0, −1.2, 1.2, 0, 0, 0] | 148.5 | −63.4 | 342.3 |

Valores en mm. **Poses finales elegidas y congeladas (3):** `cero`, `ready`, `baja`
(commit de congelación: `[hash]` — fecha `[ ]` — firmado por `[ ]`).

## 6. Resultados de la medición (rellenar después)

Los datos crudos van en `evidencias/medición_fk.csv`. Resumen:

| Pose | x_med | y_med | z_med | Error [mm] | ¿≤ 10 mm? |
|---|---:|---:|---:|---:|:-:|
| `cero` | `[ ]` | `[ ]` | `[ ]` | `[ ]` | `[ ]` |
| `ready` | `[ ]` | `[ ]` | `[ ]` | `[ ]` | `[ ]` |
| `baja` | `[ ]` | `[ ]` | `[ ]` | `[ ]` | `[ ]` |

Análisis del error (si es grande y constante → offset de marco o de `d6`; si
crece con la distancia → revisar los `a_i`): `[ ]`

## 7. Firma

Firmado antes de ejecutar las mediciones:
`[Nombre 1]` · `[Nombre 2]` · `[Nombre 3]` · `[Nombre 4]` — Fecha/hora: `[ ]`

---

*Este documento se completa luego con: diagrama de secuencia (Ítem 2) y la
predicción del p95 por política (Ítem 3), que forman parte del mismo
«documento de diseño previo».*
