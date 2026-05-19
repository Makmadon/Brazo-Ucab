"""
Sistema de Control de Servomotores mediante Visión por Computadora y Arduino
============================================================================
Trabajo de Fin de Curso UCAB: Interfaz Gestual de Brazo Robótico
Autores: Stalin Franco, Luis Salazar
Docente: Eladio Lobo

Este script captura video en tiempo real, detecta los 21 landmarks de la mano
usando MediaPipe (API tasks), calcula parámetros de control (ángulo de inclinación,
distancia pulgar-meñique, posición 2D de la muñeca) y los envía vía serial
a un Arduino para controlar 4 servomotores.

Funciona incluso si el Arduino no está conectado (modo simulación).
"""

import cv2
import math
import serial
import time
import os
import sys
import numpy as np
import urllib.request

# ============================================================================
# IMPORTACIÓN DE MEDIAPIPE (Nueva API tasks)
# ============================================================================
from mediapipe.tasks.python.vision import (
    HandLandmarker,
    HandLandmarkerOptions,
    HandLandmarkerResult,
    RunningMode,
    drawing_utils,
    HandLandmarksConnections
)
from mediapipe.tasks.python.core.base_options import BaseOptions
from mediapipe.tasks.python.vision.core.image import Image
from mediapipe import ImageFormat

# ============================================================================
# CONFIGURACIÓN
# ============================================================================

# --- Configuración Serial (Arduino) ---
# Cambia 'COM7' por el puerto donde esté conectado tu Arduino.
# Si no hay Arduino conectado, el sistema funciona igual en modo simulación.
PUERTO_SERIAL = 'COM4'
BAUD_RATE = 9600
TIMEOUT = 1

# --- Configuración del modelo MediaPipe ---
# El modelo se descarga automáticamente la primera vez
MODEL_URL = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand_landmarker.task")

# --- Configuración de cámara ---
CAMARA_INDEX = 0  # 0 = integrada, 1 = USB, etc.
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# ============================================================================
# FUNCIÓN: DESCARGA DEL MODELO
# ============================================================================

def descargar_modelo():
    """
    Descarga el modelo hand_landmarker.task si no existe localmente.
    """
    if os.path.exists(MODEL_PATH):
        print(f"Modelo encontrado: {MODEL_PATH}")
        return True

    print("=" * 60)
    print("DESCARGANDO MODELO DE MEDIAPIPE HANDS...")
    print("=" * 60)
    print(f"Origen: {MODEL_URL}")
    print(f"Destino: {MODEL_PATH}")
    print("Esto puede tomar unos segundos...")

    try:
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
        print("✅ Modelo descargado exitosamente.")
        return True
    except Exception as e:
        print(f"❌ Error al descargar el modelo: {e}")
        print("\nPuedes descargarlo manualmente desde:")
        print(MODEL_URL)
        print(f"Y guardarlo en: {MODEL_PATH}")
        return False

# ============================================================================
# FUNCIÓN: INICIALIZAR ARDUINO (OPCIONAL)
# ============================================================================

def inicializar_arduino(puerto, baudios, timeout):
    """
    Intenta conectar con el Arduino. Si no está disponible, el sistema
    funciona en modo simulación (sin Arduino).

    Returns:
        tuple: (arduino_objeto or None, modo_simulacion bool)
    """
    try:
        arduino = serial.Serial(puerto, baudios, timeout=timeout)
        time.sleep(2)  # Esperar a que se establezca la conexión serial
        print(f"✅ Conectado a Arduino en {puerto} a {baudios} baudios.")
        return arduino, False
    except serial.SerialException as e:
        print(f"⚠️  No se pudo conectar al Arduino en {puerto}.")
        print(f"   Razón: {e}")
        print("   El sistema funcionará en MODO SIMULACIÓN (sin Arduino).")
        print("   Los valores se mostrarán solo en pantalla.")
        return None, True

# ============================================================================
# FILTRO DE KALMAN 1D
# ============================================================================

class KalmanFilter1D:
    """
    Filtro de Kalman unidimensional para suavizar datos de control.
    
    Suaviza el jitter (temblor) de los landmarks detectados por MediaPipe,
    prediciendo la posición basada en velocidad y corrigiendo con la medición.
    
    Args:
        process_noise: Ruido del proceso (qué tanto confiar en la predicción)
                      Mayor valor = más suave pero menos responsivo
        measurement_noise: Ruido de medición (qué tanto confiar en la medición)
                          Mayor valor = más suave pero más lento en responder
        initial_value: Valor inicial del estado
    """
    
    def __init__(self, process_noise=0.01, measurement_noise=0.1, initial_value=90.0):
        # Estado: [posición, velocidad]
        self.x = np.array([[initial_value], [0.0]])
        
        # Matriz de covarianza del error (incertidumbre inicial)
        self.P = np.array([[1.0, 0.0], [0.0, 1.0]])
        
        # Matriz de transición de estado (modelo de velocidad constante)
        # x_{k+1} = x_k + v_k * dt, asumimos dt=1 (entre frames)
        self.F = np.array([[1.0, 1.0], [0.0, 1.0]])
        
        # Matriz de control (no usamos)
        self.B = None
        
        # Matriz de observación (medimos solo posición)
        self.H = np.array([[1.0, 0.0]])
        
        # Ruido del proceso (Q)
        self.Q = np.array([
            [process_noise, 0.0],
            [0.0, process_noise * 0.5]
        ])
        
        # Ruido de medición (R)
        self.R = np.array([[measurement_noise]])
        
    def predict(self):
        """
        Etapa de predicción: estima el siguiente estado basado en el modelo.
        """
        # x = F * x
        self.x = self.F @ self.x
        # P = F * P * F^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q
        
    def update(self, measurement):
        """
        Etapa de actualización: corrige la predicción con la medición real.
        
        Args:
            measurement: Valor medido (float)
            
        Returns:
            float: Valor filtrado (posición estimada)
        """
        # Innovación: y = z - H * x (escalar)
        y = measurement - (self.H @ self.x).item()
        
        # Covarianza de innovación: S = H * P * H^T + R
        S = self.H @ self.P @ self.H.T + self.R
        
        # Ganancia de Kalman: K = P * H^T * S^{-1} (matriz 2x1)
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Actualizar estado: x = x + K * y (K es 2x1, y es escalar -> multiplicación normal)
        self.x = self.x + K * y
        
        # Actualizar covarianza: P = (I - K * H) * P
        I = np.eye(2)
        self.P = (I - K @ self.H) @ self.P
        
        # Retornar posición filtrada
        return self.x[0, 0]

    
    def reset(self, value=None):
        """
        Reinicia el filtro a su estado inicial.
        """
        if value is not None:
            self.x = np.array([[value], [0.0]])
        else:
            self.x = np.array([[90.0], [0.0]])
        self.P = np.array([[1.0, 0.0], [0.0, 1.0]])


# ============================================================================
# FUNCIONES DE CÁLCULO DE PARÁMETROS
# ============================================================================

def calcular_angulo_inclinacion(wrist, middle_tip):
    """
    Calcula el ángulo de inclinación de la mano usando la posición de la
    muñeca y la punta del dedo medio.

    Fórmula: ángulo = atan2(dy, dx) convertido a grados y normalizado a 0-180.

    Args:
        wrist: NormalizedLandmark de la muñeca (índice 0)
        middle_tip: NormalizedLandmark de la punta del dedo medio (índice 12)

    Returns:
        float: Ángulo de inclinación normalizado entre 0 y 180 grados.
    """
    reference_dx = middle_tip.x - wrist.x
    reference_dy = middle_tip.y - wrist.y

    angle_rad = math.atan2(reference_dy, reference_dx)
    angle_deg = math.degrees(angle_rad) % 360

    # Mapear a 0-180 grados
    if angle_deg > 180:
        angle_deg = 360 - angle_deg

    return angle_deg


def calcular_distancia_pulgar_menique(thumb_tip, pinky_tip):
    """
    Calcula la distancia entre la punta del pulgar y la punta del meñique.
    Se multiplica por 2 porque los valores de MediaPipe están entre 0 y 1,
    y la distancia rara vez supera 0.5.

    Fórmula: distancia = 2 * sqrt((x1-x2)² + (y1-y2)²)

    Args:
        thumb_tip: NormalizedLandmark de la punta del pulgar (índice 4)
        pinky_tip: NormalizedLandmark de la punta del meñique (índice 20)

    Returns:
        float: Distancia normalizada entre 0 y 180.
    """
    distance = 2 * math.sqrt(
        (thumb_tip.x - pinky_tip.x) ** 2 +
        (thumb_tip.y - pinky_tip.y) ** 2
    )

    distance_px = min(180, int(distance * 180))
    return distance_px


def calcular_posicion_muneca(wrist):
    """
    Calcula la posición 2D de la muñeca escalada a 0-180 grados.

    Args:
        wrist: NormalizedLandmark de la muñeca (índice 0)

    Returns:
        tuple: (posicion_x, posicion_y) valores entre 0 y 180.
    """
    movimiento_x = wrist.x * 180
    movimiento_y = wrist.y * 180

    movimiento_x = max(0, min(180, movimiento_x))
    movimiento_y = max(0, min(180, movimiento_y))

    return movimiento_x, movimiento_y


def enviar_a_arduino(arduino, modo_simulacion, angulo, distancia, mov_x, mov_y):
    """
    Envía los datos al Arduino por puerto serial.
    Si está en modo simulación, solo muestra los datos en consola.

    Formato: "ángulo,distancia_pulgar_menique,posicion_x,posicion_y\n"
    """
    data = f"{angulo:.1f},{distancia:.1f},{mov_x:.1f},{mov_y:.1f}\n"

    if modo_simulacion:
        # En modo simulación, mostrar cada 30 frames para no saturar la consola
        if not hasattr(enviar_a_arduino, "frame_count"):
            enviar_a_arduino.frame_count = 0
        enviar_a_arduino.frame_count += 1
        if enviar_a_arduino.frame_count % 30 == 0:
            print(f"[SIMULACIÓN] Datos enviados: {data.strip()}")
    else:
        try:
            arduino.write(data.encode())
        except serial.SerialException as e:
            print(f"❌ Error de comunicación serial: {e}")
            return False

    return True


# ============================================================================
# FUNCIÓN: DIBUJAR INFORMACIÓN EN EL FRAME
# ============================================================================

def dibujar_info_en_frame(frame, angulo, distancia, mov_x, mov_y,
                          mano_detectada, fps=None, enviando=True):
    """
    Dibuja la información de control y los valores calculados en el frame.
    """
    alto, ancho = frame.shape[:2]

    # Fondo semitransparente para la información
    overlay = frame.copy()

    if mano_detectada:
        # Mostrar valores de control
        info_text = [
            f"Angulo (Base): {angulo:.1f}°",
            f"Pinza (dist): {distancia:.1f}",
            f"Base (X): {mov_x:.1f}",
            f"Altura (Y): {mov_y:.1f}"
        ]

        # Rectángulo de fondo
        cv2.rectangle(overlay, (5, 5), (250, 145), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

        for i, text in enumerate(info_text):
            cv2.putText(
                frame, text, (10, 30 + i * 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
            )

        # Indicador de envío (PAUSADO o ENVIANDO)
        status_color = (0, 255, 0) if enviando else (0, 0, 255)
        status_text = "ENVIANDO" if enviando else "PAUSADO"
        cv2.putText(
            frame, f"[{status_text}]",
            (10, 135), cv2.FONT_HERSHEY_SIMPLEX,
            0.6, status_color, 2
        )
    else:
        cv2.rectangle(overlay, (5, 5), (300, 40), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
        cv2.putText(
            frame, "MANO NO DETECTADA",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 0, 255), 2
        )

    # Mostrar FPS si está disponible
    if fps is not None:
        cv2.putText(
            frame, f"FPS: {fps:.1f}",
            (ancho - 120, 30), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (255, 255, 0), 1
        )

    # Instrucciones
    cv2.putText(
        frame, "'ESPACIO': pausar/env | 'q'/'ESC': salir",
        (10, alto - 10), cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (200, 200, 200), 1
    )


# ============================================================================
# FUNCIÓN PRINCIPAL
# ============================================================================

def main():
    print("\n" + "=" * 60)
    print("🤖 CONTROL GESTUAL DE BRAZO ROBÓTICO")
    print("=" * 60)
    print("Visión por Computadora + Arduino")
    print("Autores: Stalin Franco, Luis Salazar")
    print("=" * 60 + "\n")

    # --- PASO 1: Descargar modelo si es necesario ---
    if not descargar_modelo():
        print("❌ No se pudo obtener el modelo. Saliendo...")
        sys.exit(1)

    # --- PASO 2: Inicializar MediaPipe HandLandmarker ---
    print("\nInicializando MediaPipe HandLandmarker...")
    try:
        options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=RunningMode.IMAGE,
            num_hands=1,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        landmarker = HandLandmarker.create_from_options(options)
        print("✅ MediaPipe HandLandmarker inicializado.")
    except Exception as e:
        print(f"❌ Error al inicializar MediaPipe: {e}")
        sys.exit(1)


    # --- PASO 3: Inicializar cámara ---
    print(f"\nInicializando cámara (índice {CAMARA_INDEX})...")
    cap = cv2.VideoCapture(CAMARA_INDEX)

    if not cap.isOpened():
        print(f"❌ Error: No se pudo abrir la cámara {CAMARA_INDEX}.")
        print("   Verifica que la cámara esté conectada.")
        print("   Prueba cambiando CAMARA_INDEX a 1 si usas cámara USB.")
        landmarker.close()
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    print("✅ Cámara inicializada.")

    # --- PASO 4: Inicializar Arduino (opcional) ---
    print(f"\nConectando con Arduino en {PUERTO_SERIAL}...")
    arduino, modo_simulacion = inicializar_arduino(PUERTO_SERIAL, BAUD_RATE, TIMEOUT)

    # --- PASO 5: Inicializar Filtros de Kalman ---
    # Cada parámetro de control tiene su propio filtro Kalman 1D
    # Ajusta process_noise y measurement_noise para controlar suavizado vs respuesta
    kf_angulo = KalmanFilter1D(process_noise=0.01, measurement_noise=0.15, initial_value=90.0)
    kf_distancia = KalmanFilter1D(process_noise=0.02, measurement_noise=0.3, initial_value=90.0)
    kf_mov_x = KalmanFilter1D(process_noise=0.02, measurement_noise=0.2, initial_value=90.0)
    kf_mov_y = KalmanFilter1D(process_noise=0.02, measurement_noise=0.2, initial_value=90.0)

    # --- PASO 6: Variables de control ---
    ultimo_angulo = 90.0
    ultima_distancia = 90.0
    ultimo_mov_x = 90.0
    ultimo_mov_y = 90.0

    # Variables para FPS
    frame_count = 0
    fps_start_time = time.time()
    fps_actual = 0.0

    # Control de envío al Arduino (toggle con barra espaciadora)
    enviando = True

    # --- PASO 6: BUCLE PRINCIPAL ---
    print("\n" + "=" * 60)
    print("🎯 SISTEMA INICIADO")
    if modo_simulacion:
        print("   MODO: SIMULACIÓN (sin Arduino)")
    else:
        print("   MODO: CONTROL REAL (con Arduino)")
    print("=" * 60)
    print("Presiona 'q' o 'ESC' para salir.")
    print("Muestra tu mano frente a la cámara para controlar los servos.")
    print("=" * 60 + "\n")

    while cap.isOpened():
        # Leer frame de la cámara
        ret, frame = cap.read()
        if not ret:
            print("❌ Error al leer frame de la cámara.")
            break

        # Voltear horizontalmente para efecto espejo
        frame = cv2.flip(frame, 1)

        # Convertir BGR a RGB (OpenCV usa BGR, MediaPipe usa RGB)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Crear imagen de MediaPipe y procesar (modo síncrono)
        mp_image = Image(image_format=ImageFormat.SRGB, data=frame_rgb)
        result = landmarker.detect(mp_image)

        # Variables para este frame (con filtro Kalman: predecir primero)
        kf_angulo.predict()
        kf_distancia.predict()
        kf_mov_x.predict()
        kf_mov_y.predict()

        angulo_filtrado = kf_angulo.x[0, 0]
        distancia_filtrada = kf_distancia.x[0, 0]
        mov_x_filtrado = kf_mov_x.x[0, 0]
        mov_y_filtrado = kf_mov_y.x[0, 0]

        mano_detectada = False

        # Procesar resultado si existe
        if result is not None:

            if result.hand_landmarks and len(result.hand_landmarks) > 0:
                hand_landmarks = result.hand_landmarks[0]  # Primera mano

                # Verificar que tenemos suficientes landmarks (21)
                if len(hand_landmarks) >= 21:
                    # Obtener landmarks específicos por índice
                    wrist = hand_landmarks[0]       # WRIST
                    thumb_tip = hand_landmarks[4]    # THUMB_TIP
                    middle_tip = hand_landmarks[12]  # MIDDLE_FINGER_TIP
                    pinky_tip = hand_landmarks[20]   # PINKY_TIP

                    # --- CALCULAR PARÁMETROS (raw/medidos) ---
                    angulo_raw = calcular_angulo_inclinacion(wrist, middle_tip)
                    distancia_raw = calcular_distancia_pulgar_menique(thumb_tip, pinky_tip)
                    mov_x_raw, mov_y_raw = calcular_posicion_muneca(wrist)

                    # --- APLICAR FILTRO DE KALMAN (actualizar con medición) ---
                    angulo_filtrado = kf_angulo.update(angulo_raw)
                    distancia_filtrada = kf_distancia.update(distancia_raw)
                    mov_x_filtrado = kf_mov_x.update(mov_x_raw)
                    mov_y_filtrado = kf_mov_y.update(mov_y_raw)

                    # Actualizar últimos valores válidos (filtrados)
                    ultimo_angulo = angulo_filtrado
                    ultima_distancia = distancia_filtrada
                    ultimo_mov_x = mov_x_filtrado
                    ultimo_mov_y = mov_y_filtrado

                    mano_detectada = True

                    # --- DIBUJAR LANDMARKS EN EL FRAME ---
                    # Convertir NormalizedLandmarks a lista de tuplas para drawing_utils
                    drawing_utils.draw_landmarks(
                        frame,
                        hand_landmarks,
                        HandLandmarksConnections.HAND_CONNECTIONS
                    )

        # --- ENVIAR DATOS FILTRADOS AL ARDUINO (solo si no está en pausa) ---
        if enviando:
            enviar_a_arduino(arduino, modo_simulacion,
                             angulo_filtrado, distancia_filtrada,
                             mov_x_filtrado, mov_y_filtrado)

        # --- CALCULAR FPS ---
        frame_count += 1
        if frame_count >= 30:
            elapsed = time.time() - fps_start_time
            fps_actual = frame_count / elapsed
            frame_count = 0
            fps_start_time = time.time()

        # --- DIBUJAR INFORMACIÓN EN PANTALLA ---
        dibujar_info_en_frame(frame, angulo_filtrado, distancia_filtrada,
                              mov_x_filtrado, mov_y_filtrado,
                              mano_detectada, fps_actual, enviando)

        # --- MOSTRAR FRAME ---
        cv2.imshow('Control Gestual de Brazo Robotico', frame)

        # --- TECLADO ---
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord(' '):  # Barra espaciadora -> toggle envío
            enviando = not enviando

    # ========================================================================
    # LIMPIEZA Y CIERRE
    # ========================================================================
    print("\n" + "=" * 60)
    print("Cerrando sistema de control gestual...")
    print("=" * 60)

    cap.release()
    cv2.destroyAllWindows()
    landmarker.close()

    if arduino is not None:
        arduino.close()

    print("✅ Sistema cerrado correctamente.")
    print("Gracias por usar el Control Gestual de Brazo Robotico.")


# ============================================================================
# PUNTO DE ENTRADA
# ============================================================================

if __name__ == "__main__":
    main()
