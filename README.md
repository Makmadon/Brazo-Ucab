# Control Gestual de Brazo Robótico

**Trabajo de Fin de Curso UCAB** - Interfaz Gestual de Brazo Robótico Mediante Visión por Computadora y la Mano Humana

**Autores:** Stalin Franco, Luis Salazar  
**Docente:** Eladio Lobo  
**Universidad:** Universidad Católica Andrés Bello (UCAB Guayana)  
**Fecha:** Junio 2025

---

## 📋 Descripción

Este proyecto implementa un sistema de control de servomotores mediante gestos de la mano, utilizando visión por computadora (MediaPipe + OpenCV) y una plataforma Arduino. El sistema captura en tiempo real la postura de la mano humana a través de una cámara web, extrae parámetros de control y los envía vía serial a un Arduino que controla 4 servomotores (SG90/MG90S) que actúan como las articulaciones de un brazo robótico.

### Características principales:
- **Control continuo y proporcional** (no solo binario) de 4 grados de libertad
- **Detección de 21 puntos clave 3D** de la mano usando MediaPipe Hands
- **Comunicación serial** a 9600 baudios entre Python y Arduino
- **Visualización en tiempo real** con los landmarks dibujados en pantalla
- **Manejo de pérdida de detección**: mantiene la última posición válida

---

## 🧠 Parámetros de Control

| Parámetro | Cálculo | Servo Controlado |
|-----------|---------|------------------|
| **Ángulo de inclinación** | `atan2(dy, dx)` entre dedo medio y muñeca | BASE (rotación) - Pin 4 |
| **Distancia pulgar-meñique** | Distancia euclidiana × 2 | PINZA (garra) - Pin 7 |
| **Posición X de muñeca** | `wrist.x × 180` | ALCANCE (codo) - Pin 6 |
| **Posición Y de muñeca** | `wrist.y × 180` | ALTURA (hombro) - Pin 5 |

---

## 📁 Estructura del Proyecto

```
Brazo robotico/
├── python/
│   ├── main.py              # Código principal de visión por computadora
│   └── requirements.txt     # Dependencias de Python
├── arduino/
│   └── control_servos.ino   # Código para Arduino Uno
└── README.md                # Este archivo
```

---

## 🔧 Requisitos de Hardware

### Componentes:
1. **Computador** con cámara web (integrada o USB)
   - Recomendado: Lenovo Ideapad 3 (i5-1135G7, 20GB RAM) o similar
2. **Arduino Uno** (o compatible)
3. **4 Servomotores** (SG90 o MG90S)
4. **Brazo robótico** LAFVIN 4DOF Acrylic Robot Mechanical Arm Claw Kit (o similar)
5. **Fuente de alimentación externa** para servos (5V, ~5A recomendado)
6. **Cable USB** para conectar Arduino al computador

### Conexiones de Servos al Arduino:

| Servo | Pin Arduino | Función |
|-------|-------------|---------|
| Base | Digital 4 | Rotación |
| Altura | Digital 5 | Hombro/Elevación |
| Alcance | Digital 6 | Codo |
| Pinza | Digital 7 | Garra |

> **Nota:** Se recomienda usar una fuente de alimentación externa de 5V para los servomotores, con conexión de tierra común con el Arduino.

---

## 💻 Instalación y Configuración

### 1. Instalar dependencias de Python

```bash
cd python
pip install -r requirements.txt
```

Esto instalará:
- `opencv-python` - Procesamiento de imágenes y video
- `mediapipe` - Detección de landmarks de la mano (Google)
- `pyserial` - Comunicación serial con Arduino

### 2. Cargar el código en Arduino

1. Abre `arduino/control_servos.ino` en el Arduino IDE
2. Conecta el Arduino por USB
3. Selecciona el puerto correcto (Herramientas > Puerto)
4. Haz clic en "Cargar" (Upload)

### 3. Configurar y ejecutar el código Python

1. Identifica el puerto COM de tu Arduino (desde Arduino IDE)
2. Abre `python/main.py` y actualiza la variable `PUERTO_SERIAL` con el puerto correcto (ej: `'COM7'`)
3. Ejecuta el script:

```bash
cd python
python main.py
```

---

## 🎮 Cómo Usar

1. Ejecuta el script Python
2. Coloca tu mano frente a la cámara (a ~0.5m de distancia)
3. Los servos responderán a tus gestos:
   - **Inclinar la mano** → Controla la rotación de la base
   - **Abrir/cerrar la mano** (separar pulgar y meñique) → Controla la pinza
   - **Mover la mano horizontalmente** → Controla el alcance del codo
   - **Mover la mano verticalmente** → Controla la altura del hombro
4. Presiona **'q'** o **'ESC'** para salir

---

## 📊 Resultados Esperados

| Métrica | Valor |
|---------|-------|
| Latencia del sistema | ~0.1 - 0.2 segundos |
| FPS de procesamiento | ~30 FPS (en Core i5) |
| Precisión de posicionamiento | ~1° (condiciones favorables) |
| Grados de libertad controlados | 4 |

---

## 🔍 Solución de Problemas

### "No se pudo conectar al puerto serial"
- Verifica que el Arduino esté conectado por USB
- Confirma el puerto COM correcto desde Arduino IDE
- Cierra el Arduino IDE (puede bloquear la comunicación serial)

### "No se pudo abrir la cámara"
- Verifica que la cámara esté conectada y no esté siendo usada por otra aplicación
- Prueba cambiando el índice de la cámara en `cv2.VideoCapture(0)` a `1` si usas una cámara USB

### Los servos tiemblan
- Asegúrate de usar una fuente de alimentación externa adecuada
- Mejora las condiciones de iluminación
- Considera implementar un filtro de suavizado (Kalman o media móvil)

---

## 🚀 Mejoras Futuras

- [ ] Implementar filtro de Kalman para suavizar el movimiento de los servos
- [ ] Agregar controlador PWM PCA9685 para más servos
- [ ] Reconocimiento de gestos simbólicos (puño, pulgar arriba, etc.)
- [ ] Integración con ROS (Robot Operating System)
- [ ] Modo de reposo seguro al perder detección prolongada
- [ ] Interfaz gráfica de usuario (GUI)

---

## 📚 Referencias

- [MediaPipe Hands - Google AI](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker)
- [OpenCV](https://opencv.org/)
- [Arduino](https://www.arduino.cc/)
- [Servo Motor SG90 - SunFounder](https://docs.sunfounder.com/projects/ultimate-sensor-kit/en/latest/components_basic/27-component_servo.html)

---

## 📄 Licencia

Este proyecto es de código abierto y fue desarrollado con fines educativos para la Universidad Católica Andrés Bello (UCAB Guayana).
