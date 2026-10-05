# Cierre reflexivo — ¿Qué política de cola llevaríamos a CapyTown y por qué?

> Versión final. No se hicieron las corridas oficiales con el robot, así que la columna «medido» se redacta
> como se hubiera contrastado y la decisión se apoya en la predicción del diseño previo.

**Equipo:** 8 (H.L.P.E, J.D.R.N, R.S.E.R) · **Fecha:** octubre de 2026

## 1. Qué medimos

Se hubieran comparado FIFO y Round Robin con la misma traza, los mismos cuatro clientes y prioridades, el modo
asíncrono, la misma duración e interpolación y el mismo procedimiento de inicio; solo cambiaría
`politica:=`. Se hubieran registrado `/arm/queue_state` y `/joint_states` con `ros2 bag`.

## 2. Qué esperábamos y qué se hubiera contrastado

| Métrica | Predicho FIFO / RR (caso de referencia) | Cómo se hubiera contrastado |
|---|---|---|
| Violaciones de exclusión mutua | 0 / 0 | `metricas.py` sobre `/joint_states`: debía dar 0 |
| Espera media / p95 global | 58.7 s · 111 s / 58.7 s · 112 s | Misma media; p95 casi igual, con ±unos segundos por la resolución |
| p95 por prioridad 1/2/3/4 | 27/57/87/117 s / 110/112/115/117 s | Desigual en FIFO, casi parejo en Round Robin |
| Índice de inanición | 27 s / 110 s | Peor en Round Robin con este orden de llegada |
| Jain al final / a mitad | 1.000 · 0.5 / 1.000 · 1.0 | Final igual; a mitad FIFO < Round Robin |
| Rechazos | 0 / 0 | `rechazos.csv` vacío en ambas |

Cualquier desviación mayor a unos pocos segundos se hubiera explicado por la resolución de 0.2 s o por el
orden real de llegada de las ráfagas.

## 3. Inanición

En la predicción, la espera máxima de la prioridad más baja (cliente A) es 27 s en FIFO, porque llega primero,
y 110 s en Round Robin, porque su último goal espera casi toda la corrida. Round Robin iguala las esperas medias
por cliente, pero no reduce las esperas extremas; FIFO las concentra en quien llega al final.

## 4. Decisión

Llevaríamos a CapyTown **Round Robin**, porque los visitantes son pares y lo importante es que ninguno
acapare el brazo: iguala las esperas por cliente (Jain ≈ 1.0 a mitad de corrida) y un cliente con ráfagas no
bloquea a los demás. La espera media global no cambia entre políticas, así que no hay un costo de eficiencia.
Si importara más la espera máxima de quien llega primero, o la urgencia, FIFO o una política con prioridades
serían preferibles. Esta decisión sale de la predicción y se hubiera confirmado con las corridas.

## 5. Limitaciones

Resolución de 0.2 s (5 Hz), una corrida por política y traza finita (Jain final siempre 1.000). Además, no se
hicieron las corridas oficiales con el robot ni el ensayo con ROS 2 real: las pruebas se hicieron con ROS
simulado y datos sintéticos.
