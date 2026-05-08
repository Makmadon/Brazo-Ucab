"""
Sistema de Control de Servomotores mediante Visión por Computadora y Arduino
============================================================================
Trabajo de Fin de Curso UCAB: Interfaz Gestual de Brazo Robótico
Autores: Stalin Franco, Luis Salazar
Docente: Eladio Lobo

Este script captura video en tiempo real, detecta los 21 landmarks de la mano
usando MediaPipe, calcula parámetros de control (ángulo de inclinación,
distancia pulgar-meñique, posición 2D de la muñeca) y los envía vía serial
a un Arduino para controlar 4 servomotores.
"""

import cv2
import math
import serial
import time
import mediapipe as mp

# ============================================================================
# CONFIGURACIÓN DE COMUNICACIÓN SERIAL
# ============================================================================
# IMPORTANTE: Cambia 'COM7' por el puerto donde esté conectado tu Arduino.
# Puedes identificarlo desde el Arduino IDE (Herramientas > Puerto).
PUERTO_SERIAL = 'COM7'
BAUD_RATE = 9600
TIMEOUT = 1

# ============================================================================
# INICIALIZACIÓN DE COMPONENTES
# ============================================================================

# Inicializar MediaPipe Hands
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

# Inicializar captura de video (0 = cámara integrada, 1 = USB, etc.)
cap = cv2.VideoCapture(0)

# Verificar que la cámara se abrió correctamente
if not cap.isOpened():
    print("ERROR: No se pudo abrir la cámara.")
    print("Verifica que la cámara esté conectada y no esté siendo usada por otra aplicación.")
    exit(1)

# Configurar resolución de la cámara (opcional)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Inicializar comunicación serial con Arduino
try:
    arduino = serial.Serial(PUERTO_SERIAL, BAUD_RATE, timeout=TIMEOUT)
    time.sleep(2)  # Esperar a que se establezca la conexión serial
    print(f"Conectado a Arduino en {PUERTO_SERIAL} a {BAUD_RATE} baudios.")
except serial.SerialException as e:
    print(f"ERROR: No se pudo conectar al puerto serial {PUERTO_SERIAL}.")
    print(f"Detalles: {e}")
    print("Verifica que:")
    print("  1. El Arduino esté conectado por USB.")
    print("  2. El puerto COM sea el correcto (revisa en Arduino IDE).")
    print("  3. No haya otra aplicación usando el puerto (cierra Arduino IDE si está abierto).")
    cap.release()
    exit(1)

# ============================================================================
# VARIABLES DE CONTROL
# ============================================================================

# Para mantener el último valor válido en caso de pérdida de detección
ultimo_angulo = 90.0
ultima_distancia = 90.0
ultimo_mov_x = 90.0
ultimo_mov_y = 90.0

# ============================================================================
# FUNCIONES AUXILIARES
# ============================================================================

def calcular_angulo_inclinacion(wrist, middle_tip):
    """
    Calcula el ángulo de inclinación de la mano usando la posición de la
    muñeca y la punta del dedo medio.

    Fórmula: ángulo = atan2(dy, dx) convertido a grados y normalizado a 0-180.

    Args:
        wrist: Landmark de la muñeca (HandLandmark.WRIST)
        middle_tip: Landmark de la punta del dedo medio (HandLandmark.MIDDLE_FINGER_TIP)

    Returns:
        float: Ángulo de inclinación normalizado entre 0 y 180 grados.
    """
    # Calcular diferencias entre los puntos
    reference_dx = middle_tip.x - wrist.x
    reference_dy = middle_tip.y - wrist.y

    # Calcular ángulo en radianes usando arco tangente
    angle_rad = math.atan2(reference_dy, reference_dx)

    # Convertir a grados y normalizar a 0-360
    angle_deg = math.degrees(angle_rad) % 360

    # Mapear a 0-180 grados
    if angle_deg <= 180:
        angle_deg = angle_deg
    else:
        angle_deg = 360 - angle_deg

    return angle_deg


def calcular_distancia_pulgar_menique(thumb_tip, pinky_tip):
    """
    Calcula la distancia entre la punta del pulgar y la punta del meñique.
    Se multiplica por 2 porque los valores de MediaPipe están entre 0 y 1,
    y la distancia rara vez supera 0.5.

    Fórmula: distancia = 2 * sqrt((x1-x2)² + (y1-y2)²)

    Args:
        thumb_tip: Landmark de la punta del pulgar (HandLandmark.THUMB_TIP)
        pinky_tip: Landmark de la punta del meñique (HandLandmark.PINKY_TIP)

    Returns:
        float: Distancia normalizada entre 0 y 180.
    """
    # Distancia euclidiana multiplicada por 2 para mejor escalado
    distance = 2 * math.sqrt(
        (thumb_tip.x - pinky_tip.x) ** 2 +
        (thumb_tip.y - pinky_tip.y) ** 2
    )

    # Escalar a 0-180 y limitar
    distance_px = min(180, int(distance * 180))

    return distance_px


def calcular_posicion_muneca(wrist):
    """
    Calcula la posición 2D de la muñeca escalada a 0-180 grados.

    Args:
        wrist: Landmark de la muñeca (HandLandmark.WRIST)

    Returns:
        tuple: (posicion_x, posicion_y) valores entre 0 y 180.
    """
    movimiento_x = wrist.x * 180
    movimiento_y = wrist.y * 180

    # Limitar a 0-180 por seguridad
    movimiento_x = max(0, min(180, movimiento_x))
    movimiento_y = max(0, min(180, movimiento_y))

    return movimiento_x, movimiento_y


# ============================================================================
# BUCLE PRINCIPAL
# ============================================================================

print("\n" + "=" * 60)
print("SISTEMA DE CONTROL GESTUAL INICIADO")
print("=" * 60)
print("Presiona 'q' o 'ESC' para salir.")
print("Muestra tu mano frente a la cámara para controlar los servos.")
print("=" * 60 + "\n")

while cap.isOpened():
    # Leer un frame de la cámara
    ret, frame = cap.read()

    if not ret:
        print("ERROR: No se pudo leer el frame de la cámara.")
        break

    # Voltear el frame horizontalmente para efecto espejo
    frame = cv2.flip(frame, 1)

    # Convertir BGR a RGB (MediaPipe requiere RGB)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Procesar el frame con MediaPipe Hands
    results = hands.process(frame_rgb)

    # Variables para los datos a enviar
    mano_detectada = False

    # Verificar si se detectaron landmarks de la mano
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Dibujar los landmarks y conexiones en el frame
            mp_drawing.draw_landmarks(
                frame,
                hand_landmarks,
                mp_hands.HAND_CONNECTIONS,
                mp_drawing_styles.get_default_hand_landmarks_style(),
                mp_drawing_styles.get_default_hand_connections_style()
            )

            # Obtener los landmarks específicos que necesitamos
            landmarks = hand_landmarks.landmark

            wrist = landmarks[mp_hands.HandLandmark.WRIST]
            middle_tip = landmarks[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
            middle_mcp = landmarks[mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
            thumb_tip = landmarks[mp_hands.HandLandmark.THUMB_TIP]
            pinky_tip = landmarks[mp_hands.HandLandmark.PINKY_TIP]

            # --- CÁLCULO DE PARÁMETROS DE CONTROL ---

            # 1. Ángulo de inclinación de la palma
            angle_deg = calcular_angulo_inclinacion(wrist, middle_tip)

            # 2. Distancia entre pulgar y meñique (control de la pinza/garra)
            distance_px = calcular_distancia_pulgar_menique(thumb_tip, pinky_tip)

            # 3. Posición 2D de la muñeca (control de base y altura)
            movimiento_x, movimiento_y = calcular_posicion_muneca(wrist)

            # Actualizar últimos valores válidos
            ultimo_angulo = angle_deg
            ultima_distancia = distance_px
            ultimo_mov_x = movimiento_x
            ultimo_mov_y = movimiento_y

            mano_detectada = True

            # --- MOSTRAR INFORMACIÓN EN PANTALLA ---

            # Mostrar los valores calculados en el frame
            info_text = [
                f"Angulo: {angle_deg:.1f}°",
                f"Pinza (dist): {distance_px:.1f}",
                f"Base (X): {movimiento_x:.1f}",
                f"Altura (Y): {movimiento_y:.1f}"
            ]

            for i, text in enumerate(info_text):
                cv2.putText(
                    frame, text, (10, 30 + i * 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                )

            # --- ENVÍO DE DATOS POR SERIAL AL ARDUINO ---

            # Formato: "ángulo,distancia_pulgar_menique,posicion_x,posicion_y\n"
            data = f"{angle_deg:.1f},{distance_px:.1f},{movimiento_x:.1f},{movimiento_y:.1f}\n"

            try:
                arduino.write(data.encode())
            except serial.SerialException as e:
                print(f"ERROR de comunicación serial: {e}")
                break

    else:
        # Si no se detecta mano, mantener los últimos valores válidos
        # Esto evita que los servos se muevan a posiciones erráticas
        cv2.putText(
            frame, "MANO NO DETECTADA",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 0, 255), 2
        )

        # Enviar los últimos valores válidos para mantener posición
        data = f"{ultimo_angulo:.1f},{ultima_distancia:.1f},{ultimo_mov_x:.1f},{ultimo_mov_y:.1f}\n"
        try:
            arduino.write(data.encode())
        except serial.SerialException as e:
            print(f"ERROR de comunicación serial: {e}")
            break

    # --- MOSTRAR EL FRAME CON LA INFORMACIÓN ---

    # Agregar instrucciones en pantalla
    cv2.putText(
        frame, "Presiona 'q' para salir",
        (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (255, 255, 255), 1
    )

    # Mostrar el frame
    cv2.imshow('Control Gestual de Brazo Robotico', frame)

    # Salir con 'q' o ESC
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:  # 27 = ESC
        break

# ============================================================================
# LIMPIEZA Y CIERRE
# ============================================================================

print("\nCerrando sistema de control gestual...")

# Liberar recursos
cap.release()
cv2.destroyAllWindows()
hands.close()
arduino.close()

print("Sistema cerrado correctamente.")
print("Gracias por usar el Control Gestual de Brazo Robotico.")
