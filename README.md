# Geometría en Movimiento: Análisis de Trayectorias y Reflexión en el Billar

**Geometría en Movimiento** es una aplicación web interactiva diseñada como una herramienta pedagógica para la asignatura de **Análisis Geométrico**. El proyecto integra principios de la geometría euclidiana clásica con algoritmos de computación gráfica de alto rendimiento para demostrar que las trayectorias en el billar no son producto del azar, sino de leyes axiomáticas verificables.

## 🎯 Propósito del Proyecto
El objetivo central es validar empíricamente la **Ley de Reflexión** y otros teoremas fundamentales del curso mediante un simulador cinemático. A diferencia de un motor de física convencional, este sistema traduce axiomas abstractos en algoritmos deterministas, permitiendo visualizar en tiempo real conceptos como la simetría axial y la congruencia de triángulos.

## 🧠 Fundamentos Geométricos e Ingeniería
El simulador se construye sobre los siguientes pilares del curso de Análisis Geométrico:

1.  **La Circunferencia (Tema Actual):** Las bolas se modelan como circunferencias de radio constante ($r = 30.75\text{ mm}$). La detección de colisiones se basa en la **tangencia**, verificando si la distancia entre centros es igual a la suma de sus radios [208 - Cap. VI].
2.  **Ley de Reflexión y Simetría Axial:** Se demuestra que el ángulo de incidencia es igual al de reflexión ($\alpha = \beta$) utilizando la construcción del **Punto Simétrico (P*)** respecto a la banda, la cual actúa como mediatriz del segmento formado.
3.  **Congruencia de Triángulos (Criterio L-A-L):** Cada rebote genera una validación visual de congruencia entre los triángulos de entrada y salida, fundamentando físicamente el impacto.
4.  **Perpendicularidad:** Crucial para el trazado de las rectas normales en los puntos de colisión y el cálculo de la distancia mínima punto-recta.

## 🛠️ Especificaciones Técnicas
Para asegurar la relevancia técnica en la **Ingeniería de Sistemas**, el proyecto implementa marcos científicos de vanguardia:

*   **Modelo Cinemático de Harrison (1994):** Gestiona la respuesta a colisiones como desplazamientos hacia "posiciones objetivo" geométricas, garantizando estabilidad numérica y evitando errores de punto flotante.
*   **Algoritmo de Sabharwal (2013):** Clasificación de intersecciones (Punto, Segmento, Área) bajo el modelo JEPD, permitiendo al motor identificar con precisión el tipo de contacto entre partículas y fronteras.
*   **Reflexión Vectorial:** Implementación algebraica de la ley geométrica mediante la fórmula $v' = v - 2(v \cdot n)n$, donde $n$ es el vector normal unitario de la banda.

## 🚀 Características del Simulador
*   **Entorno Sandbox:** El usuario puede spawnear múltiples bolas y controlar la fuerza y dirección del tiro mediante una interfaz intuitiva [Chat History].
*   **Capas de Teoría Interactiva:** Interruptores para visualizar líneas normales, puntos simétricos, triángulos congruentes y puntos de tangencia [Chat History].
*   **Mesa Profesional:** Área de juego con relación de aspecto 2:1 (1.42m x 2.84m) según los estándares del billar francés.

## 🧰 Tecnologías Utilizadas
*   **HTML5 & JavaScript (Canvas API):** Para el motor de renderizado y lógica del simulador.
*   **GeoGebra:** Para la validación estática y construcciones geométricas rigurosas.
*   **Python (Matplotlib/Numpy):** Para el análisis cuantitativo de datos y gráficas de ángulos.

## 👥 Equipo de Trabajo
*   Santiago Ospina Ramírez
*   Miguel Ángel Tobón Zapata
*   Juan Manuel Pava Higuita
*   Gerónimo Torres Correa
*   **Docente:** Sirwuendy Cardona Posada.