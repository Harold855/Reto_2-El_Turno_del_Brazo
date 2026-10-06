# Documento de diseño previo — Reto 2 (El Turno del Brazo)

| Campo | Valor |
|---|---|
| Equipo / `ROS_DOMAIN_ID` | 8 / 50 (42 + n° de grupo: 8) |
| Integrantes | `Harold Lincoln Payco Espinoza, Jorge Daniel Rivera Nagaro, Rodrigo Sebastian Escobar Rosado` |

**Motivo del cambio de este documento luego de haberlo subido al aula virtual** 

Los ítems 2 y 4 se habían cerrado sin pruebas físicas por la fecha límite de entrega (lunes 05
de octubre). El lunes 06 de octubre se aprovechó el espacio para probarlos con el robot y se actualizaron el código, la
evidencia y la documentación, resultando en que estos items queden completados. El ítem 3 se realizó a medias, su código se creó
inicialmente para probarlo en el brazo, pero por falta de tiempo y de apoyo grupal no se pudo completar la medición. 

## 1. Tabla DH y predicción previa (ítem 1)

### 1.1 Convención y tabla DH

Denavit-Hartenberg **estándar**: `A_i = Rot_z(θ_i)·Trans_z(d_i)·Trans_x(a_i)·Rot_x(α_i)`, con
`θ_i = q_i + offset_i` y `T_0_6 = A1·A2·A3·A4·A5·A6`; la posición del efector es la última columna de
`T_0_6`. Longitudes en mm, ángulos en rad dentro del código. Medidas de los eslabones tomadas del manual del JetCobot (Yahboom); los marcos y la tabla se
con los vectores x, y, z (rotación y traslación) de cada articulación. Implementada en `src/arm_broker/arm_broker/fk.py`:

| i | θ_i | d_i [mm] | a_i [mm] | α_i |
|:-:|:---|---:|---:|---:|
| 1 | q1 | 134.75 | 0 | +90° |
| 2 | q2 − 90° | 0 | −110 | 0° |
| 3 | q3 | 0 | −96 | 0° |
| 4 | q4 − 90° | 63.4 | 0 | +90° |
| 5 | q5 + 90° | 75.55 | 0 | −90° |
| 6 | q6 | 50 | 0 | 0° |

Detalle (marcos, límites articulares, workspace y limitaciones) en [`tabla_dh.md`](tabla_dh.md).

### 1.2 Predicción previa de las 3 poses

Estas se declararon antes de medir, calculada con `fk.fk(q)` para el `q` comandado. Se probaron 4 poses y se
usan las 3 primeras (`cero`, `ready`, `girada`):

| Pose | q comandado [rad] | x_pred [mm] | y_pred [mm] | z_pred [mm] |
|---|---|---:|---:|---:|
| `cero` | [0, 0, 0, 0, 0, 0] | 50.00 | −63.40 | 416.30 |
| `ready` | [0, −0.5, 0.5, 0, 0.5, 0] | 96.62 | −39.43 | 402.83 |
| `girada` | [0.6, −0.4, 0.4, 0, 0.3, 0] | 102.23 | 11.03 | 407.62 |

Se encuentra en `evidencias/item1/predicciones_antes_de_medir.txt`. 

**Criterio de aceptación:** 

Error de posición ≤ 10 mm en las tres poses. 

Procedimiento: Se lee el `q` realmente adoptado (`get_angles()`), se calcula `FK(q_real)` y se compara con la posición 
que reporta el robot (`get_coords()`). Los resultados (5.8, 5.9 y 5.9 mm) están en `tabla_dh.md` §6 y en el README.

## 2. Diagrama de secuencia (ítem 2)

Camino de un goal: cliente → `goal_callback` (admisión con FK) → `handle_accepted_callback`
(encolado) → worker (política) → `execute_callback` → `/joint_states`.

```mermaid
sequenceDiagram
    autonumber
    participant C as Cliente
    participant A as ActionServer move_arm
    participant Q as Cola + política (FIFO/RR)
    participant W as Worker único
    participant J as /joint_states (JetCobot)
    C->>A: send_goal(q_objetivo)
    A->>A: goal_callback: límites, workspace (FK), paso, cupo
    alt goal inválido
        A-->>C: REJECT (motivo) + registro en rechazos.csv
    else goal válido
        A-->>C: ACCEPT (reserva cupo)
        A->>Q: handle_accepted_callback: encola Pedido (no ejecuta)
    end
    loop cada 20 ms
        W->>Q: política.siguiente()
    end
    W->>A: goal_handle.execute()
    A->>A: execute_callback: revalida el paso
    loop interpolación
        A->>J: publica q
        A-->>C: feedback EXECUTING
    end
    A-->>C: Result(success, wait_time_s, exec_time_s)
    A->>W: pedido.fin (libera el brazo)
    Note over A,C: /arm/queue_state a 5 Hz
```

## 2.1 Pruebas con el robot (ítem 2, 06 de octubre)

El ítem 2 se había cerrado sin pruebas físicas. El día siguente se aprovechó la tardepara probar el broker con el brazo y se agregó la evidencia en `evidencias/item_2/`.

**Entorno usado**

```
Grupo: 8
ROS_DOMAIN_ID = 42 + 8 = 50
Jetson / JetCobot: 172.51.1.20
Discovery Server: puerto 11811
ROS 2: Humble
RMW: rmw_fastrtps_cpp
```

En el Jetson (donde corre `arm_broker` y el Discovery Server):

```bash
export ROS_DOMAIN_ID=50
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jetson/super_client_configuration_file.xml
```

En la Raspberry Pi 400 (clientes):

```bash
export ROS_DOMAIN_ID=50
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=172.51.1.20:11811
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/super_client_g8.xml
```

**Tres rechazos físicos** (`goal_callback`, admisión con la FK). Todos terminaron con `Goal was rejected`:

| Prueba | `client_id` | `q` enviado [rad] | Resultado | Motivo registrado |
|---|---|---|---|---|
| Límite articular | `g8_limite` (prioridad 1) | `[3.5, 0, 0, 0, 0, 0]` | REJECT | `1_Joint fuera de rango: 3.500 rad, límite [-2.93, 2.93]` |
| Workspace | `g8_workspace` | `[0.0, 1.5, 2.4, 0, 0, 0]` | REJECT | `efector a 71 mm de la base, demasiado cerca` |
| Paso excesivo | `g8_paso` | `[1.3, 0, 0, 0, 0, 0]` | REJECT | `paso articular de 1.30 rad desde la pose actual, máximo 1.20` |

Son las filas representativas de cada causa de `evidencias/item_2/rechazos_reconstruidos.csv`. Ese CSV es una
reconstrucción: el `rechazos.csv` original se generó en el Jetson durante la prueba pero no se preservó en el
repositorio, así que se rehízo a partir de las capturas (comandos enviados, `Goal was rejected` y el `cat` del CSV
original); no se reconstruyeron las marcas de tiempo (`evidencias/item_2/README.md`). Capturas:
`rechazo_limite1.jpeg`, `rechazo_limite2.jpeg`, `rechazo_workspace.jpeg` y `rechazo_paso.jpeg`.

**Goal válido.** Con `client_id: g8_valido_3`, prioridad 1 y `joint_positions: [0.3, 0, 0, 0, 0, 0]` se obtuvo el
flujo completo:

```
Goal accepted → Feedback EXECUTING → el brazo se mueve → Result (success: true, message: ok)
→ Goal finished with status: SUCCEEDED          (≈ 3.02 s de ejecución)
```

Es evidencia de que el broker no solo rechaza goals incorrectos: también admite y **ejecuta físicamente**
solicitudes válidas (`goal_valido_1.jpeg`, `goal_valido_2.jpeg`).

**Exclusión mutua.** Una prueba automática lanzó dos goals de clientes distintos, uno tras otro
(`exclusion_mutua_A2_B2.jpeg`, `g8_A2.log`, `g8_B2.log`):

| Cliente | Prioridad | `q` [rad] | Comportamiento | `wait_time_s` | `exec_time_s` |
|---|:-:|---|---|---:|---:|
| `g8_A2` | 1 | `[0.0, -0.5, 0.5, 0.0, 0.5, 0.0]` | Pasó directo a ejecutar; `SUCCEEDED` | 0.0109 | 3.0156 |
| `g8_B2` | 2 | `[0.6, -0.4, 0.4, 0.0, 0.3, 0.0]` | `state: QUEUED`, `queue_position: 1` repetido hasta `elapsed_s ≈ 2.49`; luego `EXECUTING`, `queue_position: 0`; `SUCCEEDED` | 2.6129 | 3.0147 |

B2 no empezó hasta que A2 terminó: su espera (2.61 s) es la ejecución restante de A2, y nunca hubo dos goals en
`EXECUTING` a la vez. También hay capturas de `/arm/queue_state` a 5 Hz (`queue_state_5hz.jpeg`) y de un único
publicador de `/joint_states` (`publicador_unico_joint_states.jpeg`).

## 3. Medición bajo contención: FIFO frente a Round Robin (ítem 3)

### 3.1 Qué se quiere saber

Si repartir el turno por clientes (Round Robin) cambia las esperas y el reparto respecto de atender
por orden de llegada (FIFO), con **exactamente las mismas condiciones**. La única variable es
`politica:=fifo` o `politica:=round_robin`.

### 3.2 Modelo y predicción (antes de medir)

Se usa un modelo de cola de un servidor con servicio constante `T` (≈ `duracion_movimiento_s`
+ un par de centésimas: 3.05 s con el valor por defecto) y las mismas clases `FIFO` y `RoundRobin`
del broker (`herramientas/simular_politicas.py`). Supuestos: cada cliente envía sus goals seguidos
(modo asíncrono, `pausa_s=0`), no hay rechazos y las poses son las válidas de la traza.

**Ideas que sostienen la predicción**

1. **La espera media global no depende de la política.** Con servicio constante los goals arrancan
   en los mismos instantes `T, 2T, 3T…` sea cual sea el orden; la política solo decide *a quién*
   le toca cada instante. Con `N` goals en total: media ≈ `(N−1)·T/2` y p95 global ≈ `0.95·(N−1)·T`.
2. **El orden de llegada de los clientes decide todo lo demás.** Con arranques escalonados
   (A, B, C, D, cada uno enviando su ráfaga completa) FIFO atiende cliente por cliente; Round Robin
   los intercala.
3. **Round Robin iguala las esperas medias por cliente, no reduce las extremas.** El último goal de
   cada cliente espera casi toda la corrida en Round Robin; en FIFO solo el último cliente.
4. **Equidad de Jain sobre goals atendidos.** Con una traza finita todos los clientes acaban
   atendidos por igual, así que al final de la corrida Jain vale 1.000 en ambas. La diferencia
   aparece **a mitad de la corrida** (`metricas.py` lo reporta como «Jain a mitad»).
5. **Si los clientes arrancan a la vez y sus goals se intercalan, FIFO y Round Robin coinciden.**
   Por eso el protocolo escalona el arranque.

**Fórmulas** (`n = poses × repeticiones` por cliente; cliente `k = 0…3` en orden de arranque;
`e = 0.5 s` de escalón entre clientes; `j = round(0.95·(n−1))`, el índice que usa `metricas.py` para el p95)

| | FIFO | Round Robin |
|---|---|---|
| Media del cliente `k` | `(k·n + (n−1)/2)·T − k·e` | `(2(n−1) + k)·T − k·e` |
| p95 del cliente `k` | `(k·n + j)·T − k·e` | `(4j + k)·T − k·e` |

Comprobación con `n = 10`, `T = 3.05`: FIFO, cliente 0 → media 13.7 s y p95 27.4 s; cliente 3 → p95
`(30 + 9)·3.05 − 1.5 = 117.4 s`. Round Robin, cliente 0 → media 54.9 s y p95 109.8 s.

**Valores para el caso de referencia**: 4 clientes × 10 poses × 1 repetición (`N = 40`), `T = 3.05 s`,
arranque escalonado A→D, prioridades A=1, B=2, C=3, D=4. Salen de
`python3 herramientas/simular_politicas.py --poses 10`. **Con la traza oficial se hubieran recalculado
con su número real de poses antes de congelar este documento.**

| Métrica | FIFO | Round Robin |
|---|---|---|
| Violaciones de exclusión mutua | 0 | 0 |
| Espera media global | 58.7 s | 58.7 s |
| **p95 global** | **111 s** | **112 s** |
| Espera máxima global | 117 s | 117 s |
| Media por prioridad 1 / 2 / 3 / 4 (A / B / C / D) | 13.7 / 43.7 / 73.7 / 103.7 s | 54.9 / 57.4 / 60.0 / 62.5 s |
| **p95 por prioridad** 1 / 2 / 3 / 4 | **27 / 57 / 87 / 117 s** | **110 / 112 / 115 / 117 s** |
| Índice de inanición (máx. espera de la prioridad 1) | 27 s | 110 s |
| Equidad de Jain al final | 1.000 | 1.000 |
| Equidad de Jain a mitad de corrida | 0.500 (A:10, B:10, C:0, D:0) | 1.000 (5 c/u) |
| Goals rechazados | 0 | 0 |

**Resumen de esta prediccion:**

FIFO y Round Robin tendrán la misma espera media y
un p95 global casi igual (≈ 111–112 s con la traza de referencia); FIFO dará esperas muy
desiguales según el orden de llegada (prioridad 1: p95 ≈ 27 s, prioridad 4: ≈ 117 s) mientras
Round Robin las igualará (≈ 110–117 s para todas); en la mitad de la corrida Jain será ≈ 0.5 con
FIFO y ≈ 1.0 con Round Robin; y el índice de inanición será **peor** en Round Robin (≈ 110 s frente
a ≈ 27 s) porque en este orden de llegada el cliente de menor prioridad (A) es el primero en FIFO.

**Cómo se contrasta:** 

La medición tiene resolución de 0.2 s (`/arm/queue_state` a 5 Hz) y no ve
los goals que empiezan a ejecutarse antes de la siguiente publicación, así que se aceptan
diferencias de unos pocos segundos.

### 3.3 Protocolo

| Elemento | Valor |
|---|---|
| Traza | Se uso para ensayos: `trazas/prueba.csv` |
| Número de clientes | 4, en la misma máquina o en cuatro Raspberry Pi 400, con nodos `cliente_A` … `cliente_D` |
| Prioridad por cliente | A=1, B=2, C=3, D=4 |
| Repeticiones | `[1]` (igual en ambas corridas) |
| Modo | `asincrono` |
| Pausa entre envíos | `0.0` s |
| Duración del movimiento | `3.0` s (`duracion_movimiento_s`) |
| Pasos de interpolación | `10` |
| Forma y orden de inicio | 1) broker con la política; 2) `ros2 bag record`; 3) clientes A, B, C, D con `inicio_unix` común más `0.5 s` de escalón entre uno y otro, para que las ráfagas lleguen en ese orden |
| Tópicos grabados | `/arm/queue_state`, `/joint_states` |
| Otros parámetros del broker | `cola_max=20` por defecto: si la traza tiene más de 20 poses por cliente, subirlo **igual en ambas corridas** para que no haya rechazos por cola llena |
| Una corrida por política | El orden de las corridas (FIFO y luego Round Robin, o al revés) se anota en `resultados.md` |

Variables que se controlan: traza, clientes, prioridades, repeticiones, pausa, modo, duración,
interpolación, orden de inicio, tópicos y `ROS_DOMAIN_ID`. Variable que cambia: la política.

| Amenaza | Medida |
|---|---|
| Un cliente arranca tarde por lo que tarda `ros2 run` | Todos esperan a `inicio_unix` (margen de 8 s en `ARRANQUE_S`) |
| Rechazos por cola llena | Ajustar `cola_max` igual en ambas |
| Otro publicador en `/joint_states` | `ros2 topic info /joint_states -v` guardado en `publicadores_joint_states.txt` |
| Datos de una corrida anterior en el bag | El script no sobreescribe: exige una carpeta vacía |
| Otro equipo en el mismo dominio DDS | `ROS_DOMAIN_ID` propio (42 + n.º de equipo) |

### 3.4 Estructura de la evidencia

```
evidencias/
└── item_3/
    ├── fifo/
    │   ├── bag/                       (ros2 bag: /arm/queue_state y /joint_states)
    │   ├── queue_state.csv
    │   ├── joint_states.csv
    │   ├── rechazos.csv               (si hubo rechazos)
    │   ├── protocolo.txt              (parámetros reales de la corrida)
    │   ├── publicadores_joint_states.txt, bag_info.txt, broker.log, cliente_*.log
    ├── round_robin/                   (igual)
    ├── comparacion_politicas.png
    └── resultados.md                  (copia de resultados_plantilla.md, completada)
```
Nota: Anteriormente se diseño esta estructura en el repositorio, con el fin de tener una idea clara sobre lo que realizariamos, como grupo, en el item 3. Sin embargo, debido a los motivos explicados. Se tuvo que borrar estas evidencias ya que no contenían nada en su interior.

### 3.5 Comandos de ejecución

Ensayo previo con ROS 2 (traza provisional): [`ensayo_previo_ros2.md`](ensayo_previo_ros2.md).

Corrida oficial (una por política; el script deja todo en `evidencias/item_3/<política>/`):

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=<42 + 8>

TRAZA=/ruta/traza_oficial.csv bash herramientas/experimento_item3.sh fifo
TRAZA=/ruta/traza_oficial.csv bash herramientas/experimento_item3.sh round_robin

python3 analisis/metricas.py evidencias/item_3/fifo/queue_state.csv \
    evidencias/item_3/round_robin/queue_state.csv \
    --salida evidencias/item_3/comparacion_politicas.png
```

### 3.6 Criterios de una corrida válida

- `total_rejected = 0` (o, si lo hubo, con causa en `rechazos.csv` y explicado).
- `total_accepted = total_completed` al final.
- `metricas.py` reporta 0 violaciones de exclusión mutua y `publicadores_joint_states.txt`
  muestra un único publicador.
- Misma traza (`sha256`) y mismos parámetros en `protocolo.txt` de las dos corridas.

### 3.7 Plantilla de resultados

[`evidencias/item_3/resultados_plantilla.md`](../evidencias/item_3/resultados_plantilla.md): métricas predichas y
medidas lado a lado, y contraste punto por punto con esta predicción.

## 4. Auditoría de la IK del firmware con la FK propia (ítem 4)

**No escribimos un solver de cinemática inversa**: pedimos un objetivo cartesiano con `send_coords()` y dejamos
que el firmware resuelva la IK. Luego se lee el `q` que el brazo adoptó (`get_angles()`, grados → radianes), se
aplica la FK propia y se compara con el objetivo pedido:

`e = √((X_FK − X_obj)² + (Y_FK − Y_obj)² + (Z_FK − Z_obj)²)`, criterio **e ≤ 10 mm**.

- **Predicción antes de medir.** Si el firmware llega al objetivo, `e` queda cerca del desfase conocido de la FK
  frente a `get_coords()` (≈ 5.9 mm en el ítem 1, casi constante, ≈ +5.5 mm en z); un valor mucho mayor
  indicaría un objetivo inalcanzable con esa orientación o un problema de marco.
- **Sesión del 06 de octubre.** Diagnóstico con `--solo-leer` (sin mover): FK(q_real) a 5.94 mm de
  `get_coords()`. Después, objetivo cartesiano `[129.3, 11.4, 398.8]` mm con orientación
  `[−92.46, −22.42, −39.03]` (la pose actual desplazada +20 mm en X), velocidad 20.
- **Resultado.** El firmware adoptó `q = (25.83, −19.42, 0.17, 13.79, 24.43, −23.29)°` y la FK dio
  (127.3, 14.1, 400.0) mm: errores (−2.02, +2.75, +1.20) mm y **error cartesiano 3.62 mm ≤ 10 mm**.
- **Codo contrario.** El firmware eligió una solución cercana a la configuración inicial (continuidad de
  movimiento, sin cambiar los signos de `q2`, `q3`, `q5`); `pymycobot` no expone el criterio interno, así que la
  conclusión se basa en los ángulos observados.
- **Herramienta y evidencia.** `herramientas/auditar_ik.py` y `evidencias/item_4/auditoria_ik.csv` (una fila con la
  auditoría oficial, reconstruida porque el CSV original del laboratorio no se preservó). Detalle en [`item4_auditoria_ik.md`](item4_auditoria_ik.md).

## 5. Cierre reflexivo

Ver [`cierre_reflexivo.md`](cierre_reflexivo.md): la decisión se razona con la predicción.

Nota Final: Para la estructura de este documento, se realizó con IA, para tener una visión sobre como podriamos organizarla. Aunque, después, por nuestra parte, se terminó por corregir datos mal implementados y detallar mejor la estructura de todo el documento. 
