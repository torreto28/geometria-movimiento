# GEOMETRÍA EN MOVIMIENTO
## Plan Arquitectónico — Simulador Interactivo de Billar

**Documento Técnico de Diseño Estructural**  
Basado en: Berrío et al. (2017), Harrison (1994), Sabharwal et al. (2013)  
Universidad de Medellín — Ingeniería de Sistemas

---

## 1. MODELO DE MOVIMIENTO: CINEMÁTICA DE HARRISON

### 1.1 Fundamento Teórico

Harrison (1994) propone un modelo cinemático **sin simulación dinámica explícita**. Cada bola no aplica fuerzas Newton-Lagrange, sino que se desplaza hacia una **posición objetivo** a una velocidad controlada.

**Adaptación al billar:**
- Bola en posición actual: `P(t) = (x, y)`
- Velocidad vectorial: `v = (vx, vy)` — magnitud = "fuerza" de tiro
- **Sin colisión:** se mueve en línea recta: `P(t+Δt) = P(t) + v·Δt`
- **Con colisión:** cambia `v` según reflexión, continúa

### 1.2 Implementación de Movimiento Cinemático

```
SISTEMA DE ESTADO POR BOLA:
├─ pos: [x, y]           // Posición actual
├─ vel: [vx, vy]         // Vector de velocidad
├─ radius: r             // Radio de la bola (constante)
├─ mass: m               // Masa (opcional, solo para bola-bola)
└─ lastCollisionTime: t  // Evitar múltiples rebotes en mismo Δt
```

**Loop de Simulación (cada frame, ~60Hz):**

```
PASO 1: Predecir movimiento libre
  FOR cada bola b:
    p_próx ← b.pos + b.vel * Δt
    
PASO 2: Detectar colisiones (ver § 2)
  colisiones_banda ← detectarColisionesBanda(todas_bolas)
  colisiones_bola ← detectarColisionesBola(todas_bolas)
  
PASO 3: Resolver colisiones
  FOR cada colisión_banda en colisiones_banda:
    v_nuevo ← reflexionarVector(b.vel, normal_banda)
    b.vel ← v_nuevo
    b.pos ← proyectarAlBorde(p_próx)  // No penetra banda
    
  FOR cada colisión_bola en colisiones_bola:
    (v1_nuevo, v2_nuevo) ← resolverChoqueBolas(b1, b2)
    b1.vel ← v1_nuevo
    b2.vel ← v2_nuevo
    
PASO 4: Aplicar amortiguamiento (fricción mesa)
  FOR cada bola b:
    b.vel ← b.vel * (1 - fricción * Δt)
    
PASO 5: Actualizar posición final
  FOR cada bola b:
    b.pos ← p_próx
```

### 1.3 Fricción y Detención

**Fricción:** 
```
vel_new = vel_old * (1 - fricción_coef * Δt)
```
Típicamente `fricción_coef ≈ 0.98–0.99` por frame.

**Detención automática:**
```
IF |vel| < umbral_eps THEN vel = [0, 0]
```
Ejemplo: `umbral_eps = 0.01` unidades/frame.

---

## 2. GESTIÓN DE COLISIONES

### 2.1 Colisión Banda-Circunferencia

**Dato geométrico:** Mesa rectangular con 4 bandas (bordes).

Cada banda = recta en forma implícita:
- **Banda superior:** y = H
- **Banda inferior:** y = 0
- **Banda izquierda:** x = 0
- **Banda derecha:** x = W

**Detección:**

```
FUNCIÓN detectarColisionesBanda(bola):
  FOR cada banda B:
    // Distancia de centro de bola a recta banda
    d = distancia_punto_a_recta(bola.pos, B)
    
    IF d < bola.radius THEN
      // Hay penetración
      punto_tangencia = proyectar_punto_a_recta(bola.pos, B)
      normal = vector_perpendicular_a_banda(B)
      
      RETORNA {
        punto_impacto: punto_tangencia,
        normal: normal,
        profundidad_penetración: bola.radius - d
      }
```

**Cálculo de punto tangencia (para visualizar impacto):**

Para banda horizontal (y=cte):
```
punto_tangencia = (bola.pos.x, banda_y)
```

Para banda vertical (x=cte):
```
punto_tangencia = (banda_x, bola.pos.y)
```

### 2.2 Colisión Bola-Bola

**Condición:** Dos circunferencias se tocan cuando distancia entre centros = r₁ + r₂.

```
FUNCIÓN detectarColisionesBola(b1, b2):
  distancia = ||b1.pos - b2.pos||
  
  IF distancia < b1.radius + b2.radius THEN
    // Colisión
    normal = (b2.pos - b1.pos) / distancia  // Normalizar
    
    RETORNA {
      bola1: b1,
      bola2: b2,
      punto_contacto: b1.pos + normal * b1.radius,
      normal: normal,
      penetración: (b1.radius + b2.radius) - distancia
    }
```

**Predicado de Sabharwal et al. (2013):**
Cuando dos bolas colisionan, sus trayectorias (líneas) **intersectan en un punto único**. La clasificación es:
- **Punto:** contacto instantáneo (caso típico)
- **Segmento:** nunca ocurre con circunferencias rígidas
- **Área:** nunca ocurre con circunferencias rígidas

### 2.3 Resolución de Colisión Banda

**Ley de reflexión (fundamento teórico de Berrío et al.):**

```
v' = v - 2·(v·n)·n
```

Donde:
- `v` = velocidad incidente
- `n` = normal unitaria de la banda (apunta hacia adentro)
- `·` = producto punto
- `v'` = velocidad reflejada

**Implementación:**

```
FUNCIÓN reflexionarVector(v, n):
  // n debe estar normalizado
  dot_product = v.x * n.x + v.y * n.y
  v_new.x = v.x - 2 * dot_product * n.x
  v_new.y = v.y - 2 * dot_product * n.y
  RETORNA v_new
```

**Ajuste de posición (evitar penetración):**

```
// Proyectar bola de vuelta al borde
profundidad = bola.radius - distancia
bola.pos += normal * profundidad
```

### 2.4 Resolución de Colisión Bola-Bola

**Caso simplificado (masas iguales, choque elástico):**

```
FUNCIÓN resolverChoqueBolas(b1, b2):
  normal = (b2.pos - b1.pos) / ||b2.pos - b1.pos||
  
  // Componentes de velocidad a lo largo de normal
  v1_normal = (b1.vel · normal) * normal
  v2_normal = (b2.vel · normal) * normal
  
  // Componentes tangenciales (no cambian en choque elástico)
  v1_tangent = b1.vel - v1_normal
  v2_tangent = b2.vel - v2_normal
  
  // En choque elástico con masas iguales: intercambiar normales
  v1_new = v2_normal + v1_tangent
  v2_new = v1_normal + v2_tangent
  
  RETORNA (v1_new, v2_new)
```

**Separación para evitar múltiples rebotes:**

```
// Separar bolas ligeramente
overlap = (b1.radius + b2.radius) - ||b2.pos - b1.pos||
b1.pos -= normal * (overlap / 2)
b2.pos += normal * (overlap / 2)
```

---

## 3. CAPAS DE VISUALIZACIÓN TEÓRICA

Cada capa es un **toggle interactivo**. Se visualizan SOLO cuando la bandeja está activada en el panel de control.

### 3.1 Capa 1: Normales y Ángulos (α = β)

**Propósito:** Demostrar la **Ley de Reflexión** numéricamente.

**Cálculos por impacto:**

```
DATOS DEL IMPACTO:
├─ punto_impacto: P (en la banda)
├─ normal: n (perpendicular a banda)
├─ velocidad_incidente: v_in
├─ velocidad_reflejada: v_out

ÁNGULO DE INCIDENCIA (α):
  α = arccos(|v_in · n| / (|v_in| * |n|))
  
ÁNGULO DE REFLEXIÓN (β):
  β = arccos(|v_out · n| / (|v_out| * |n|))

VERIFICACIÓN:
  error = |α - β|
  SI error < ε ENTONCES ley_reflexión_válida
```

**Visualización en canvas:**

```
1. Línea discontinua: vector v_in (desde punto anterior)
2. Línea discontinua: vector v_out (hacia siguiente posición)
3. Línea sólida perpendicular: vector normal n
4. Arcos de círculo: marcadores de ángulos α y β
5. Etiquetas: "α = XY°", "β = XY°"
```

**Color:** Verde claro, transparencia 0.5.

---

### 3.2 Capa 2: Isometría y Punto Simétrico (P*)

**Propósito:** Visualizar la **reflexión como transformación isométrica** (movimiento rígido, sin deformación).

**Fundamento (Berrío et al., 2017):**

La reflexión de una bola respecto a una banda es equivalente a reflejar el punto sobre la recta que representa la banda. Si imagina la banda como espejo, la trayectoria reflejada es idéntica a una línea recta hacia el punto simétrico P*.

**Cálculo del punto simétrico:**

```
FUNCIÓN calcularPuntoSimetrico(P, banda):
  // P es la posición de la bola
  // banda es una recta (puede ser horizontal o vertical)
  
  proyección = proyectar_punto_a_recta(P, banda)
  
  P* = 2 * proyección - P
  
  RETORNA P*
```

**Ejemplo numérico (banda horizontal y = 0):**
```
P = (3, 5), banda = y = 0
proyección = (3, 0)
P* = 2*(3,0) - (3,5) = (3, -5)
```

**Verificación de isometría (mediatriz):**

La mediatriz del segmento PP* debe coincidir exactamente con la banda:

```
FUNCIÓN verificarMediatriz(P, P*, banda):
  punto_medio = (P + P*) / 2
  
  distancia_P_a_banda = distancia_punto_a_recta(P, banda)
  distancia_P_estrella_a_banda = distancia_punto_a_recta(P*, banda)
  
  SI distancia_P_a_banda ≈ distancia_P_estrella_a_banda ENTONCES
    isometría_válida = true
```

**Visualización en canvas:**

```
1. Círculo relleno (transparente): marca posición P (bola real)
2. Punto simulado: marca posición P* (fuera de la mesa)
3. Línea discontinua: segmento PP*
4. Línea sólida gruesa: mediatriz (coincide con banda)
5. Anotación: "Distancia(P, banda) = Distancia(P*, banda)"
```

**Color:** Azul claro, transparencia 0.4.

---

### 3.3 Capa 3: Congruencia Lado-Ángulo-Lado (L-A-L)

**Propósito:** Demostrar mediante **criterio de congruencia L-A-L** que el triángulo de incidencia ≅ triángulo de reflexión.

**Fundamento geométrico:**

En cada impacto, se construyen dos triángulos:

```
TRIÁNGULO INCIDENTE (T_in):
  Vértices:
    A = punto anterior de la bola
    P = punto de impacto
    C = proyección de A sobre la banda (perpendicular)
  
  Lados:
    AP = trayectoria incidente
    PC = distancia perpendicular a banda
    AC = proyección sobre banda

TRIÁNGULO REFLEJADO (T_out):
  Vértices:
    P = punto de impacto
    B = punto siguiente de la bola
    D = proyección de B sobre la banda (perpendicular)
  
  Lados:
    PB = trayectoria reflejada
    PD = distancia perpendicular a banda
    BD = proyección sobre banda
```

**Criterio L-A-L:**

Dos triángulos son congruentes si:
- Lado 1: AP ≅ PB (trayectorias equidistantes de la banda)
- Ángulo: ∠APC ≅ ∠BPD (ángulos con respecto a normal)
- Lado 2: PC ≅ PD (distancias perpendiculares iguales)

**Cálculos:**

```
// Obtener trayectorias
punto_anterior = bola.pos_anterior
punto_impacto = bola.pos_actual
punto_siguiente = predicción_movimiento_reflejado

// Proyecciones perpendiculares
C = proyectar_perpendicular(punto_anterior, banda)
D = proyectar_perpendicular(punto_siguiente, banda)

// Lados del triángulo incidente
lado_AP = ||punto_anterior - punto_impacto||
lado_PC = ||punto_impacto - C||
lado_AC = ||punto_anterior - C||

// Lados del triángulo reflejado
lado_PB = ||punto_siguiente - punto_impacto||
lado_PD = ||punto_impacto - D||
lado_BD = ||punto_siguiente - D||

// Verificación
congruencia_lado1 = |lado_AP - lado_PB| < ε
ángulo_APC = ángulo_entre_vectores(PA, PC)
ángulo_BPD = ángulo_entre_vectores(PB, PD)
congruencia_ángulo = |ángulo_APC - ángulo_BPD| < ε_angular
congruencia_lado2 = |lado_PC - lado_PD| < ε

SI todas las congruencias ENTONCES
  criterio_LAL_válido = true
```

**Clasificación de intersección (Sabharwal et al., 2013):**

Cuando se dibujan dos triángulos, sus lados (segmentos) pueden intersectar:

```
FUNCIÓN clasificarIntersección(segmento1, segmento2):
  
  IF segmento1 y segmento2 se intersectan en UN PUNTO ÚNICO ENTONCES
    tipo = "PUNTO"
    punto_intersección = calcular_punto()
    
  ELSE IF segmento1 ⊆ segmento2 (superpuesto) ENTONCES
    tipo = "SEGMENTO"
    
  ELSE
    tipo = "NINGUNO"
    
  RETORNA tipo, punto_intersección
```

Para triángulos congruentes en el contexto del billar, la intersección típica será:
- Lado AP y lado PB comparten el punto P → intersección de **PUNTO**
- Lados AC y BD generalmente no se intersectan (paralelos o separados)

**Visualización en canvas:**

```
1. Triángulo incidente: relleno con color rojo, transparencia 0.3
   Vértices: A, P, C
   Etiquetas: |AP|, |PC|, |AC|
   
2. Triángulo reflejado: relleno con color verde, transparencia 0.3
   Vértices: P, B, D
   Etiquetas: |PB|, |PD|, |BD|
   
3. Punto de intersección P: marca prominente
   
4. Arco de ángulo: marca ∠APC con ∠BPD
   
5. Cuadro de texto: "L-A-L Válido" o "Discrepancia: X%"
```

**Color:** Rojo incidente (C₁), Verde reflejado (C₂), transparencia 0.3.

---

### 3.4 Validación Conjunta de Capas

En cada impacto, registrar en un **panel de datos**:

```
REGISTRO DE IMPACTO #N:
├─ Timestamp: t_ms
├─ Posición impacto: (x, y)
├─ Ley Reflexión: α = XY°, β = XY°, error = Z°
├─ Isometría: Distancia(P, banda) = A, Distancia(P*, banda) = B
├─ L-A-L: Lado1 = X, Ángulo = Y°, Lado2 = Z, Válido = [SI/NO]
└─ Intersección (Sabharwal): PUNTO en (x, y)
```

Este registro **exportable a CSV** para análisis cuantitativo en informe.

---

## 4. FLUJO UX/UI — SANDBOX INTERACTIVO

### 4.1 Componentes de Interfaz

#### A. Canvas Principal (680px × 480px típico)

```
┌─────────────────────────────────────────┐
│                                         │
│  [Mesa verde billar]                    │
│                                         │
│  ◉ Bola blanca (interactiva)            │
│  ○ Bola objetivo                        │
│                                         │
│  Línea guía (al arrastrar cue)          │
│  Visualizaciones teóricas (toggleables) │
│                                         │
└─────────────────────────────────────────┘
```

#### B. Panel de Control (debajo del canvas)

```
┌────────────────────────────────────────┐
│ MODOS DE VISUALIZACIÓN:                │
│ ☐ Normales & Ángulos (α=β)             │
│ ☐ Punto Simétrico & Mediatriz          │
│ ☐ Triángulos L-A-L                     │
│                                        │
│ ACCIONES:                              │
│ [Nuevo Tiro]  [Limpiar Mesa]  [Reset] │
│                                        │
│ DATOS:                                 │
│ Impactos: 3 | Ángulo promedio: 42.3° │
│ [Exportar Datos]                       │
└────────────────────────────────────────┘
```

### 4.2 Interacción: Drag-and-Fire

**Entrada del usuario (mouse/táctil):**

```
EVENTO: mousedown en bola blanca
  ├─ Capturar posición inicial (x₀, y₀)
  └─ Iniciar modo "carga"

EVENTO: mousemove (mientras está presionado)
  ├─ Calcular vector desde centro hacia cursor
  │  dirección = (x_cursor - x₀, y_cursor - y₀)
  ├─ Normalizar dirección
  ├─ Calcular magnitud (fuerza)
  │  fuerza = ||dirección||
  ├─ Limitar fuerza: fuerza = min(fuerza, fuerza_máx)
  └─ Dibujar línea guía
     └─ Línea desde bola blanca en dirección, largo ∝ fuerza

EVENTO: mouseup
  ├─ Aplicar velocidad inicial
  │  bola.vel = dirección_normalizada * fuerza_escalada
  ├─ Lanzar simulación
  └─ Desactivar modo carga
```

**Visualización de línea guía:**

```
Línea discontinua (5px seg, 3px hueco)
Color: amarillo/blanco con transparencia 0.7
Grosor: 2px
Desde: centro bola blanca
Hacia: cursor del ratón
Largo visible: min(fuerza * escala_visual, largo_máximo)
```

### 4.3 Flujo de Simulación

```
FASE 1: Posicionamiento inicial
  ├─ Mostrar 1 bola blanca en esquina
  ├─ Mostrar 3–5 bolas objetivo dispersas
  ├─ Mensaje: "Arrastra la bola blanca y suelta"

FASE 2: Esperar input usuario
  ├─ En tiempo real, dibujar línea guía
  ├─ Mostrar vector de velocidad predicho

FASE 3: Lanzamiento
  ├─ Aplicar vel inicial
  ├─ Desactivar entrada de usuario
  ├─ Loop de simulación activo (§1.2)

FASE 4: Colisión y visualización
  ├─ Detectar impactos (§2)
  ├─ Dibujar capas teóricas activas (§3)
  ├─ Registrar datos de impacto

FASE 5: Fin del movimiento
  ├─ Todas las bolas vel ≈ 0
  ├─ Permitir nuevo tiro
  ├─ Mostrar resumen de impactos
```

### 4.4 Estados de Interactividad

```
ESTADO: Reposo
  ├─ Input habilitado (drag en bola blanca)
  ├─ Botones disponibles: [Nuevo Tiro], [Limpiar]
  └─ Capas teóricas: NO visibles

ESTADO: Cargando
  ├─ Línea guía visible y reactiva
  ├─ Input activo (mousemove)
  ├─ Botones deshabilitados
  └─ Información visual: "Fuerza: XY%"

ESTADO: Simulando
  ├─ Input deshabilitado
  ├─ Capas teóricas: mostrar según toggles
  ├─ Datos en tiempo real actualizándose
  └─ Botones: solo [Pausa] (opcional)

ESTADO: Fin
  ├─ Input habilitado nuevamente
  ├─ Resumen de datos visible
  ├─ Opción [Exportar] activa
  └─ Botón [Nuevo Tiro] destacado
```

---

## 5. ARQUITECTURA DE SOFTWARE

### 5.1 Capas y Módulos

```
┌─────────────────────────────────────┐
│  CAPA 1: PRESENTACIÓN (Canvas + UI) │
│  - Renderizado 60fps                 │
│  - Event listeners (mouse/touch)     │
│  - State display (toggles, datos)    │
└─────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────┐
│  CAPA 2: VISUALIZACIÓN TEÓRICA      │
│  - Renderer de ángulos (α, β)        │
│  - Renderer de isometría (P*, mediatr|
│  - Renderer L-A-L (triángulos)      │
│  - Overlay en canvas                 │
└─────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────┐
│  CAPA 3: FÍSICA CINEMÁTICA          │
│  - Motor de movimiento (Harrison)    │
│  - Fricción y amortiguamiento        │
│  - Detección colisiones              │
│  - Resolución colisiones             │
└─────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────┐
│  CAPA 4: DATOS Y ESTADÍSTICAS       │
│  - Registro de impactos              │
│  - Cálculos de ángulos y distancias  │
│  - Validaciones teóricas             │
│  - Exportación CSV                   │
└─────────────────────────────────────┘
```

### 5.2 Pseudostructura de Clases

```
CLASS Bola {
  pos: [x, y]
  vel: [vx, vy]
  radius: r
  mass: m
  lastCollisionTime: t
  posAnterior: [x', y']  // Para cálculo L-A-L
  
  MÉTODOS:
  - actualizar(dt)
  - aplicarFricción(coef, dt)
  - reflejar(normal)
  - distancia(otra_bola)
}

CLASS Banda {
  tipo: "horizontal" | "vertical"
  posición: número  // x=0, x=W, y=0, y=H
  normal: [nx, ny]
  
  MÉTODOS:
  - distanciaAlPunto(punto)
  - proyectarPunto(punto)
  - verificarColisión(bola)
}

CLASS SimuladorBillar {
  bolas: Bola[]
  bandas: Banda[]
  tiempo: t
  fricción: coef
  
  MÉTODOS:
  - crearBola(pos, vel)
  - removerBola(index)
  - simularFrame(dt)
  - detectarColisiones()
  - resolverColisiones()
  - renderizar()
}

CLASS CapaVisualización {
  toggles: { normales, isometría, LAL }
  datosImpactos: Impacto[]
  
  MÉTODOS:
  - dibujarNormalesYÁngulos(ctx, impacto)
  - dibujarIsometría(ctx, bola, banda)
  - dibujarTriángulosLAL(ctx, impacto)
  - registrarImpacto(datos)
  - exportarCSV()
}
```

### 5.3 Flujo de Datos

```
INPUT USUARIO (drag)
    ↓
Calcular vel inicial
    ↓
SimuladorBillar.simularFrame()
  ├─ Actualizar pos (cinemática Harrison)
  ├─ Detectar colisiones (§2)
  ├─ Resolver colisiones
  ├─ Aplicar fricción
  └─ Registrar datos
    ↓
CapaVisualización
  ├─ Si toggle "normales": dibujar α, β
  ├─ Si toggle "isometría": dibujar P*, mediatriz
  ├─ Si toggle "L-A-L": dibujar triángulos
  └─ Actualizar panel de datos
    ↓
Canvas.render()
    ↓
[Pantalla]
```

---

## 6. FUNCIONES MATEMÁTICAS CLAVE

### 6.1 Producto Punto (para ángulos y reflexión)

```
FUNCIÓN dotProduct(v1, v2):
  RETORNA v1.x * v2.x + v1.y * v2.y
```

### 6.2 Magnitud de Vector

```
FUNCIÓN magnitud(v):
  RETORNA sqrt(v.x² + v.y²)
```

### 6.3 Normalización

```
FUNCIÓN normalizar(v):
  mag = magnitud(v)
  SI mag = 0 ENTONCES RETORNA [0, 0]
  RETORNA [v.x / mag, v.y / mag]
```

### 6.4 Distancia Punto a Recta

Para banda horizontal (y = k):
```
FUNCIÓN distancia_punto_a_banda_horizontal(punto, banda_y):
  RETORNA |punto.y - banda_y|
```

Para banda vertical (x = k):
```
FUNCIÓN distancia_punto_a_banda_vertical(punto, banda_x):
  RETORNA |punto.x - banda_x|
```

### 6.5 Reflexión Vectorial

```
FUNCIÓN reflejar(v, n):
  // n debe ser unitario
  d = dotProduct(v, n)
  RETORNA [
    v.x - 2 * d * n.x,
    v.y - 2 * d * n.y
  ]
```

### 6.6 Cálculo de Ángulo entre Vectores

```
FUNCIÓN ángulo(v1, v2):
  cos_ángulo = dotProduct(v1, v2) / (magnitud(v1) * magnitud(v2))
  // Limitar a [-1, 1] para evitar errores numéricos
  cos_ángulo = max(-1, min(1, cos_ángulo))
  RETORNA arccos(cos_ángulo) en radianes
```

### 6.7 Proyección de Punto sobre Recta

Para banda horizontal (y = k):
```
FUNCIÓN proyectar_horizontal(punto, banda_y):
  RETORNA [punto.x, banda_y]
```

Para banda vertical (x = k):
```
FUNCIÓN proyectar_vertical(punto, banda_x):
  RETORNA [banda_x, punto.y]
```

### 6.8 Punto Simétrico (Isometría)

```
FUNCIÓN puntoSimétrico(P, banda_recta):
  proyección = proyectar_punto_a_recta(P, banda_recta)
  RETORNA 2 * proyección - P
```

---

## 7. CONSIDERACIONES DE IMPLEMENTACIÓN

### 7.1 Tolerancias Numéricas

```
ε_distancia = 0.01        // Tolerancia colusión
ε_velocidad = 0.001       // Umbral detención
ε_ángulo = 0.5°           // Tolerancia verificación ley reflexión
ε_tiempo = 0.001          // Δt simulación (ajustable)
```

### 7.2 Límites de Simulación

```
velocidad_máxima = 10 unidades/frame
fuerza_máxima = 300 píxeles  // Drag máximo
fricción_coef = 0.98  // Por frame
amortiguamiento_colisión = 0.95  // Pérdida energética
```

### 7.3 Prevención de Inestabilidades

- **Substeps:** Si Δt > umbral, subdividir en múltiples pasos pequeños
- **Clamp de velocidad:** Si |vel| > máx, limitar
- **Separación post-colisión:** Separar bolas que se solapan
- **Lock-out temporal:** Evitar múltiples rebotes en mismo frame (usar `lastCollisionTime`)

### 7.4 Tecnología Recomendada

```
FRONTEND:
  - HTML5 Canvas 2D (rendering rápido)
  - JavaScript vanilla (física + lógica)
  - O: React + Canvas (si se integra con UI más compleja)
  
LIBRERÍAS OPCIONALES:
  - p5.js (simplifica canvas, pero overhead)
  - Babylon.js o Three.js (si se quiere 3D, no necesario)
  
HERRAMIENTAS EDUCATIVAS PARALELAS:
  - GeoGebra: demostración estática del punto simétrico
  - Python (matplotlib + numpy): análisis de datos exportados
```

---

## 8. MAPEO A OBJETIVOS ESPECÍFICOS DEL PROYECTO

### Objetivo Especifico 1: Punto Simétrico y Mediatriz (Berrío)

**Implementación:** § 3.2 Capa de Isometría
- Cálculo P* según fórmula reflexión
- Verificación mediatriz
- Visualización en toggle

### Objetivo Específico 2: Criterio L-A-L y Sabharwal

**Implementación:** § 3.3 Capa L-A-L
- Construcción triángulos incidente y reflejado
- Verificación congruencia Lado-Ángulo-Lado
- Clasificación intersecciones (Sabharwal)

### Objetivo Específico 3: Herramienta Interactiva (Harrison)

**Implementación:** § 1 (Cinemática Harrison) + § 4 (UX)
- Modelo cinemático sin simulación física explícita
- Interfaz sandbox drag-and-fire
- Tiempo real, 60fps

### Objetivo Específico 4: Conexión con Motores de Física

**En documentación técnica:**
- Mostrar fórmula v' = v - 2(v·n)n es idéntica a reflexión geométrica
- Citación de su uso en Unity/Godot
- Tabla comparativa: demostración formal vs. implementación algorítmica

---

## 9. CRONOGRAMA DE DESARROLLO

```
SEMANA 8-9: Implementación
  ├─ Canvas setup + loop simulación
  ├─ Cinemática y fricción
  ├─ Colisión banda
  └─ Colisión bola-bola

SEMANA 10: Capas visualización
  ├─ Capa normales/ángulos
  ├─ Capa isometría
  └─ Capa L-A-L

SEMANA 11: UI y exportación
  ├─ Panel de toggles
  ├─ Registro de datos
  ├─ Exportar CSV
  └─ Refinamiento interacción

SEMANA 12-13: Documentación y pulido
  ├─ Redacción marco teórico
  ├─ Includ demostraciones formales
  ├─ Análisis de datos colectados
  └─ Preparación exposición

SEMANA 14: Exposición
```

---

## REFERENCIAS

**[1]** Berrío Valbuena, J. D., Valbuena Duarte, S., & Sánchez Anillo, R. (2017). *Simulación de composición de simetrías en GeoGebra para simplificar la teoría de los diamantes en la práctica del billar*. Pensamiento Americano, 10(19), 55–67.

**[2]** Harrison, J. (1994). *A kinematic model for collision response* [Master's thesis]. The University of British Columbia.

**[3]** Sabharwal, C. L., Leopold, J. L., & McGeehan, D. (2013). Triangle-triangle intersection determination and classification to support qualitative spatial reasoning. *Polibits*, 48, 13–22.

**[4]** Segunda Revisión — Universidad de Medellín (2026). *Geometría en Movimiento: Análisis de Trayectorias y Reflexión de Ángulos en el Billar*.

---

**Documento técnico — Fase 2, Semana 7**  
*Arquitectura lista para implementación Fase 3*
