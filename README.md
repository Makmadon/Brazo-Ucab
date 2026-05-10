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
- **Detección de 21 puntos clave 3D** de la mano usando MediaPipe Hands (API `tasks`)
- **Comunicación serial** a 9600 baudios entre Python y Arduino
- **Visualización en tiempo real** con los landmarks dibujados en pantalla
- **Manejo de pérdida de detección**: mantiene la última posición válida
- **Modo simulación**: funciona sin Arduino conectado (solo muestra valores en pantalla)
- **Descarga automática del modelo**: el modelo de MediaPipe se descarga la primera vez

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
│   ├── requirements.txt     # Dependencias de Python
│   └── hand_landmarker.task # Modelo de MediaPipe (se descarga automáticamente)
├── arduino/
│   └── control_servos.ino   # Código para Arduino Uno
└── README.md                # Este archivo
```

---

## 🔧 Requisitos de Hardware

### Componentes:
1. **Computador** con cámara web (integrada o USB)
   - Recomendado: Lenovo Ideapad 3 (i5-1135G7, 20GB RAM) o similar
2. **Arduino Uno** (o compatible) - *Opcional, el sistema funciona en modo simulación*
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
- `mediapipe` - Detección de landmarks de la mano (Google) - API `tasks`
- `pyserial` - Comunicación serial con Arduino
- `numpy` - Operaciones numéricas

### 2. Cargar el código en Arduino (opcional, solo si tienes el hardware)

1. Abre `arduino/control_servos.ino` en el Arduino IDE
2. Conecta el Arduino por USB
3. Selecciona el puerto correcto (Herramientas > Puerto)
4. Haz clic en "Cargar" (Upload)

### 3. Ejecutar el código Python

```bash
cd python
python main.py
```

> **Nota:** La primera vez que ejecutes el script, descargará automáticamente el modelo `hand_landmarker.task` (~7.8 MB) desde los servidores de Google.

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

### Sin Arduino conectado:
El sistema funciona en **modo simulación**. Los valores de control se muestran en pantalla y en la consola cada 30 frames, pero no se envía ninguna señal física.

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
- El sistema arrancará igual en **modo simulación**
- Verifica que el Arduino esté conectado por USB
- Confirma el puerto COM correcto desde Arduino IDE
- Cierra el Arduino IDE (puede bloquear la comunicación serial)

### "No se pudo abrir la cámara"
- Verifica que la cámara esté conectada y no esté siendo usada por otra aplicación
- Prueba cambiando el índice de la cámara en `CAMARA_INDEX = 0` a `1` si usas una cámara USB

### Los servos tiemblan
- Asegúrate de usar una fuente de alimentación externa adecuada
- Mejora las condiciones de iluminación
- Considera implementar un filtro de suavizado (Kalman o media móvil)

### Error al descargar el modelo
- Verifica tu conexión a internet
- Descarga manualmente desde: https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
- Guárdalo en la carpeta `python/` como `hand_landmarker.task`

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
