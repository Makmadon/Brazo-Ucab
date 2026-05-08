/*
 * Control de Servomotores mediante Visión por Computadora y Arduino
 * =================================================================
 * Trabajo de Fin de Curso UCAB: Interfaz Gestual de Brazo Robótico
 * Autores: Stalin Franco, Luis Salazar
 * Docente: Eladio Lobo
 *
 * Este programa recibe datos por comunicación serial desde Python,
 * parsea 4 valores separados por comas (ángulo, distancia, posición X,
 * posición Y) y controla 4 servomotores (SG90/MG90S) que actúan como
 * las articulaciones de un brazo robótico.
 *
 * Conexiones de los servos:
 *   - Servo BASE (rotación):     Pin digital 4
 *   - Servo ALTURA (hombro):     Pin digital 5
 *   - Servo ALCANCE (codo):      Pin digital 6
 *   - Servo PINZA (garra):       Pin digital 7
 *
 * Comunicación Serial:
 *   - Baud rate: 9600
 *   - Formato de datos: "ángulo,distancia,posX,posY\n"
 *     donde cada valor está en el rango 0-180.
 */

#include <Servo.h>

// ============================================================================
// CREACIÓN DE OBJETOS SERVO
// ============================================================================
Servo base;      // Rotación de la base del brazo
Servo altura;    // Articulación del hombro (elevación)
Servo alcance;   // Articulación del codo (alcance horizontal)
Servo pinza;     // Apertura/cierre de la pinza (garra)

// ============================================================================
// CONFIGURACIÓN DE PINES
// ============================================================================
const int PIN_BASE = 4;
const int PIN_ALTURA = 5;
const int PIN_ALCANCE = 6;
const int PIN_PINZA = 7;

// ============================================================================
// VARIABLES GLOBALES
// ============================================================================
String datosRecibidos = "";   // Buffer para almacenar datos seriales
bool datosCompletos = false;  // Bandera para indicar datos listos

// ============================================================================
// CONFIGURACIÓN INICIAL
// ============================================================================
void setup() {
  // Inicializar comunicación serial a 9600 baudios
  Serial.begin(9600);

  // Esperar a que el puerto serial esté listo (opcional, útil en algunos Arduinos)
  while (!Serial) {
    ; // Esperar por conexión serial (para Arduino Leonardo/Micro)
  }

  // Asignar los servos a sus pines correspondientes
  base.attach(PIN_BASE);
  altura.attach(PIN_ALTURA);
  alcance.attach(PIN_ALCANCE);
  pinza.attach(PIN_PINZA);

  // Inicializar todos los servos en la posición central (90 grados)
  base.write(90);
  altura.write(90);
  alcance.write(90);
  pinza.write(90);

  // Indicar que el sistema está listo
  Serial.println("Sistema de control de servos iniciado.");
}

// ============================================================================
// BUCLE PRINCIPAL
// ============================================================================
void loop() {
  // Verificar si hay datos disponibles en el puerto serial
  while (Serial.available() > 0) {
    // Leer un carácter del buffer serial
    char caracter = Serial.read();

    // Si encontramos el carácter de nueva línea, los datos están completos
    if (caracter == '\n') {
      datosCompletos = true;
      break;
    } else {
      // Agregar el carácter al buffer de datos
      datosRecibidos += caracter;
    }
  }

  // Si tenemos datos completos, procesarlos
  if (datosCompletos) {
    // Eliminar espacios en blanco y saltos de línea adicionales
    datosRecibidos.trim();

    // Buscar las posiciones de las comas (separadores)
    int primeraComa = datosRecibidos.indexOf(',');
    int segundaComa = datosRecibidos.indexOf(',', primeraComa + 1);
    int terceraComa = datosRecibidos.indexOf(',', segundaComa + 1);

    // Verificar que se encontraron las 3 comas (4 valores)
    if (primeraComa > 0 && segundaComa > 0 && terceraComa > 0) {
      // Extraer los 4 valores como substrings y convertirlos a enteros
      // NOTA: El orden debe coincidir con el envío desde Python:
      //   ángulo_inclinacion -> base
      //   distancia_pulgar_menique -> pinza
      //   posicion_x_muneca -> alcance
      //   posicion_y_muneca -> altura
      int angulo = datosRecibidos.substring(0, primeraComa).toInt();
      int distancia = datosRecibidos.substring(primeraComa + 1, segundaComa).toInt();
      int posX = datosRecibidos.substring(segundaComa + 1, terceraComa).toInt();
      int posY = datosRecibidos.substring(terceraComa + 1).toInt();

      // ====================================================================
      // MAPEO DE PARÁMETROS A SERVOS
      // ====================================================================
      // Según la documentación del proyecto:
      //   - angle_deg (ángulo de inclinación) -> Servo BASE (rotación)
      //   - distance_px (distancia pulgar-meñique) -> Servo PINZA (garra)
      //   - movimiento_x (posición X muñeca) -> Servo ALCANCE (codo)
      //   - movimiento_y (posición Y muñeca) -> Servo ALTURA (hombro)
      // ====================================================================

      // Limitar los valores al rango seguro 0-180 usando constrain()
      // Esto protege los servos de recibir valores fuera de rango
      angulo = constrain(angulo, 0, 180);
      distancia = constrain(distancia, 0, 180);
      posX = constrain(posX, 0, 180);
      posY = constrain(posY, 0, 180);

      // Enviar los ángulos a los servos
      base.write(angulo);       // Ángulo de inclinación -> rotación base
      pinza.write(distancia);   // Distancia pulgar-meñique -> apertura pinza
      alcance.write(posX);      // Posición X muñeca -> alcance codo
      altura.write(posY);       // Posición Y muñeca -> altura hombro

      // (Opcional) Enviar confirmación de vuelta a Python
      // Serial.println("OK");
    }

    // Limpiar el buffer y la bandera para la siguiente lectura
    datosRecibidos = "";
    datosCompletos = false;
  }
}
