/**
 * @file mpu_block.h
 * @brief Gestión del sensor MPU6050: calibración, lectura y cálculo de ángulo y RPM.
 *
 * Módulo responsable de:
 *  - Mantener instancias globales del objeto MPU6050 y del almacenamiento Preferences.
 *  - Almacenar y cargar offsets de calibración para acelerómetro y giroscopio.
 *  - Calibrar el sensor promediando múltiples muestras.
 *  - Leer periódicamente el MPU6050, calcular el ángulo usando un filtro complementario
 *    y calcular las RPM de ruedas a partir de contadores de pulsos externos.
 *
 * Notas:
 *  - Las unidades esperadas:
 *      * Aceleración cruda: LSB (se normaliza dividiendo entre 16384.0 para obtener g).
 *      * Giroscopio crudo: LSB (se normaliza dividiendo entre 131.0 para obtener °/s).
 *  - La calibración se guarda en Preferences bajo el namespace "mpu" con las claves:
 *      "ax_off", "ay_off", "az_off", "gx_off".
 *  - Las variables countR y countL (contadores de pulsos de encoder) deben existir fuera
 *    de este módulo (se espera que sean incrementadas por interrupciones). Deben ser
 *    de tipo apropiado (p. ej. volatile long) para uso seguro desde ISR.
 */

/**
 * @brief Instancia global del sensor MPU6050 utilizada por las funciones del módulo.
 *
 * Uso: variable global ya inicializada en este fichero; debe llamarse begin() en setup()
 * del programa principal según sea necesario.
 */
 
/**
 * @brief Instancia global de Preferences usada para guardar/cargar offsets de calibración.
 *
 * Namespace utilizado: "mpu"
 */

/**
 * @brief Offsets de calibración para el acelerómetro (componentes X/Y/Z).
 *
 * Formato: valores en las mismas unidades que devuelve el sensor (LSB).
 * Se aplican restando estos offsets a las lecturas crudas del acelerómetro.
 */

/**
 * @brief Offset de calibración para el giroscopio en el eje X.
 *
 * Formato: valor en LSB; se resta a la lectura cruda del giroscopio antes de escalar.
 */

/**
 * @brief Valor del ángulo filtrado (resultado del filtro complementario).
 *
 * Unidad: grados (°). Variable global que mantiene el estado entre llamadas.
 */

/**
 * @brief Guarda los offsets actuales de calibración en la memoria no volátil (Preferences).
 *
 * Comportamiento:
 *  - Abre el espacio de preferencias "mpu" en modo escritura.
 *  - Escribe las claves "ax_off", "ay_off", "az_off", "gx_off" con los valores actuales.
 *  - Cierra Preferences y emite por Serial un mensaje indicando que la calibración
 *    ha sido guardada.
 *
 * Retorno: Ninguno.
 * Efectos secundarios: Modifica el almacenamiento no volátil.
 */

/**
 * @brief Carga los offsets de calibración desde Preferences si existen.
 *
 * Comportamiento:
 *  - Abre el espacio de preferencias "mpu" en modo solo lectura.
 *  - Comprueba si la clave "ax_off" existe; si no existe devuelve false.
 *  - Si existen, lee "ax_off", "ay_off", "az_off", "gx_off" y actualiza las variables
 *    globales correspondientes.
 *  - Cierra Preferences.
 *
 * Retorno:
 *  - true si la calibración fue encontrada y cargada correctamente.
 *  - false si no hay datos de calibración disponibles.
 */

/**
 * @brief Realiza una calibración del MPU6050 promediando múltiples muestras.
 *
 * Algoritmo:
 *  - Recolecta 'samples' lecturas del sensor (aquí 500), con un pequeño retardo entre lecturas (3 ms).
 *  - Acumula las lecturas brutas de acelerómetro (ax, ay, az) y giroscopio (gx) en sumas.
 *  - Calcula el promedio para cada eje y lo asigna a los offsets correspondientes.
 *  - Para el eje Z del acelerómetro, se resta 16384 LSB (valor aproximado de 1g) para
 *    compensar la gravedad cuando se asume que el dispositivo está en posición vertical.
 *  - Guarda los offsets usando saveCalibration().
 *
 * Requisitos y consideraciones:
 *  - El dispositivo debe mantenerse en la orientación deseada (p. ej. vertical y en reposo)
 *    durante la calibración para que az_offset compense correctamente la gravedad.
 *  - Esta función bloquea mientras realiza las lecturas y el retardo entre ellas.
 *
 * Retorno: Ninguno (los offsets se actualizan globalmente y se persisten en Preferences).
 */

/**
 * @brief Lee el MPU6050 y actualiza el ángulo filtrado y las RPM medias de dos encoders.
 *
 * Parámetros:
 *  - dt: intervalo de tiempo en segundos desde la última llamada (float).
 *  - angle: referencia de salida (float&) donde se devuelve el ángulo filtrado en grados.
 *  - rpmL_f: referencia de salida (float&) para RPM filtrada de la rueda izquierda.
 *  - rpmR_f: referencia de salida (float&) para RPM filtrada de la rueda derecha.
 *
 * Funcionamiento:
 *  1. Lee ax, ay, az, gx, gy, gz con mpu.getMotion6().
 *  2. Aplica los offsets de calibración y escala:
 *       - accelY/Z normalizados: (raw - offset) / 16384.0  -> g
 *       - gyroX normalizado: (raw - gx_offset) / 131.0    -> °/s
 *  3. Calcula el ángulo desde el acelerómetro: angleAcc = atan2(accelY, accelZ) en grados.
 *  4. Actualiza angleFiltered usando un filtro complementario:
 *       angleFiltered = 0.98 * (angleFiltered + gyroX * dt) + 0.02 * angleAcc
 *     (predomina la integración del giroscopio con corrección lenta del acelerómetro).
 *  5. Lee y reinicia los contadores de pulsos externos countR y countL (se asume que existen
 *     fuera de este módulo). Estos contadores representan pulsos acumulados desde la última
 *     llamada y deben ser manejados de forma segura si se actualizan desde ISR.
 *  6. Convierte pulsos a revoluciones usando PPR = 44.0 * 119.0 (pulsos por revolución),
 *     calcula RPM = (revs / dt) * 60.
 *  7. Aplica un filtrado exponencial simple sobre las RPM:
 *       rpm_filtered = 0.7 * rpm_filtered_prev + 0.3 * rpm_medida
 *  8. Devuelve angle, rpmL_f y rpmR_f con los valores filtrados.
 *
 * Constantes y supuestos:
 *  - Escalados del sensor:
 *      * Acelerómetro: 16384 LSB/g (configuración típica ±2g).
 *      * Giroscopio: 131 LSB/(°/s) (configuración típica ±250 °/s).
 *  - PPR: producto 44.0 * 119.0 (pulsos por revolución); ajustar si la relación mecánica cambia.
 *  - Coeficientes del filtro complementario y del filtrado de RPM son constantes fijas en el código:
 *      * Complementario: 0.98 (gyro) / 0.02 (acc).
 *      * Filtrado RPM: 0.7 (anterior) / 0.3 (actual).
 *
 * Efectos secundarios:
 *  - Resetea countR y countL a 0 después de leerlos.
 *  - Actualiza la variable global angleFiltered.
 *
 * Recomendaciones:
 *  - Asegurarse de que dt sea consistente y preciso (p. ej. medido con micros()/millis()).
 *  - Las variables countR/countL deben ser tipo volatile y manipuladas de forma segura si
 *    se usan interrupciones para incrementar los pulsos.
 *  - Ajustar PPR y coeficientes de filtro según la dinámica del sistema y la precisión deseada.
 */
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Preferences.h>

// ===================== MPU ====================================
extern MPU6050 mpu;
extern Preferences prefs;
extern float ax_offset, ay_offset, az_offset;
extern float gx_offset;
extern float angleFiltered;

MPU6050 mpu;
Preferences prefs;
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0;
float angleFiltered = 0;

inline void saveCalibration()
{
  prefs.begin("mpu", false);
  prefs.putFloat("ax_off", ax_offset);
  prefs.putFloat("ay_off", ay_offset);
  prefs.putFloat("az_off", az_offset);
  prefs.putFloat("gx_off", gx_offset);
  prefs.end();
  Serial.println(">> Calibración guardada.");
}

inline bool loadCalibration()
{
  prefs.begin("mpu", true);
  if (!prefs.isKey("ax_off"))
  {
    prefs.end();
    return false;
  }
  ax_offset = prefs.getFloat("ax_off");
  ay_offset = prefs.getFloat("ay_off");
  az_offset = prefs.getFloat("az_off");
  gx_offset = prefs.getFloat("gx_off");
  prefs.end();
  return true;
}

inline void calibrateMPU()
{
  Serial.println(">> Iniciando calibración MPU...");

  long ax_sum = 0, ay_sum = 0, az_sum = 0, gx_sum = 0;
  const int samples = 500;

  for (int i = 0; i < samples; i++)
  {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    delay(3);
  }

  ax_offset = ax_sum / samples;
  ay_offset = ay_sum / samples;
  az_offset = (az_sum / samples) - 16384;
  gx_offset = gx_sum / samples;

  saveCalibration();
  Serial.println(">> Calibración MPU COMPLETA.");
}

inline void updateAngleAndRPM(float dt,
                              float &angle,
                              float &rpmL_f,
                              float &rpmR_f)
{
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float accelY = (ay - ay_offset) / 16384.0;
  float accelZ = (az - az_offset) / 16384.0;
  float gyroX = (gx - gx_offset) / 131.0;
  float angleAcc = atan2(accelY, accelZ) * 180.0 / PI;
  angleFiltered = 0.98 * (angleFiltered + gyroX * dt) + 0.02 * angleAcc;

  long pulsesR = countR;
  long pulsesL = countL;
  countR = 0;
  countL = 0;

  const float PPR = 44.0 * 119.0;
  float revR = pulsesR / PPR;
  float revL = pulsesL / PPR;
  float rpmR = (revR / dt) * 60.0;
  float rpmL = (revL / dt) * 60.0;

  static float rpmR_f_local = 0, rpmL_f_local = 0;
  rpmR_f_local = 0.7 * rpmR_f_local + 0.3 * rpmR;
  rpmL_f_local = 0.7 * rpmL_f_local + 0.3 * rpmL;

  angle = angleFiltered;
  rpmL_f = rpmL_f_local;
  rpmR_f = rpmR_f_local;
}
