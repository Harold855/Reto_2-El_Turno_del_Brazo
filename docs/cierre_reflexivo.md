# Cierre reflexivo 

**Equipo:** 8 (Harold Lincoln Payco Espinoza, Jorge Daniel Rivera Nagaro, Rodrigo Sebastian Escobar Rosado) · **Fecha:** 4 de octubre de 2026

## 1. Qué hubieramos medimos

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

## 4. Decisión: ¿Que politica es mejor para CapyTown?

Como CapyTown es una mini ciudad ficticia, donde los habitantes son capibaras, que usan autos robots automatizados. Entonces la politica se aplicaría
a estos autos, eligiríamos Round Robin(RR), pero no eliminaríamos FIFO, si no que se podría implementar para el caso de calles cortas o grupos reducidos 
de autos.
**Razon**
Imaginemos una calle cualquiera de la mini ciudad. Con FIFO, un auto espera al otro que se "atienda" o mueva de ese lugar, siendo simple y ordenado. 
Pero, el problema esta en que muchos autos pueden llegar al mismo tiempo o permanecer alli por mucho tiempo, entonces al llegar nuevos vehículos 
todos se amontonan en esa zona, causando tráfico en esa calle. Por lo que, con Round Robin (RR) se puede administrar mejor las intersecciones, calles estrechas, 
estacionamientos, u otros lugares compartidos. Porque el objetivo, es que todos loa autos tengan la oportunidad de avanzar y el tráfico sea más equitativo.
Por lo que la propuesta final sería: RR entre calles y FIFO dentro de calles.

