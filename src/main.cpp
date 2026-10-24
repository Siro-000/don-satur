#include <Arduino.h>
#include <Bluepad32.h>

// ============================================================
// PINES ESP32 -> TB6612FNG
// ============================================================
const int PIN_PWMA = 2;
const int PIN_AIN1 = 16;
const int PIN_AIN2 = 4;

const int PIN_PWMB = 19;
const int PIN_BIN1 = 5;
const int PIN_BIN2 = 18;

const int PIN_STBY = 17;

// ============================================================
// PWM
// ============================================================
const int PWM_FREQ = 10000;
const int PWM_RESOLUTION = 8;
const int PWM_CHANNEL_A = 0;
const int PWM_CHANNEL_B = 1;

// ============================================================
// CONFIGURACION CONTROL
// ============================================================
// CONDUCCION:
//   R2 = avanzar, L2 = retroceder (analogicos: mas apretado = mas rapido).
//   Stick izquierdo (eje X) = giro.
//   - Con un gatillo apretado: gira mientras se mueve (la rueda del lado
//     hacia donde giras va mas lenta).
//   - Sin gatillos: gira sobre su propio eje.
//
// Bluepad32 entrega el stick de -511 a 512 y los gatillos de 0 a 1023.
// Zona muerta del stick X (tiene drift). Calibracion: mirar en el monitor
// serie los valores en reposo y poner la zona unos 20-30 puntos por encima
// del maximo observado.
const int ZONA_MUERTA_X = 90;

// Zona muerta de los gatillos (0 a 1023).
const int ZONA_MUERTA_GATILLO = 40;

// Velocidad maxima (0 a 255).
const int VELOCIDAD_MAXIMA = 255;

// Velocidad maxima al girar sobre su propio eje (0 a 255).
// Mas bajo = mas facil apuntar a la pelota, y menos tiron si se suelta el
// gatillo con el stick de costado (el robot pasa de curva a giro sobre su
// eje; la rampa de velocidad suaviza ese cambio). Probar entre 140 y 180.
const int VELOCIDAD_GIRO_SOBRE_EJE = 160;

// Que tan cerrado gira en marcha, con el stick a fondo:
//   0.0 = la rueda de adentro queda frenada (curva suave)
//   1.0 = la rueda de adentro va a la misma velocidad pero al reves (muy cerrado)
const float CIERRE_GIRO_EN_MARCHA = 0.0;

const float FACTOR_A = 1.00;
const float FACTOR_B = 1.00;

// ============================================================
// RAMPA DE VELOCIDAD Y FRENO
// ============================================================
// Cuanto puede cambiar la velocidad de cada rueda por ciclo (el loop corre
// cada 20 ms). Evita que un cambio brusco (R2 a fondo -> L2 a fondo)
// invierta los motores de golpe: ese pico de corriente puede reiniciar el
// ESP32 por caida de tension, sobre todo con la bateria baja.
//   ACELERAR: alejarse de 0.             25 -> de 0 a 255 en ~200 ms
//   FRENAR:   acercarse a 0 (frenar o
//             empezar a cambiar de sentido). 50 -> de 255 a 0 en ~100 ms
// Si el robot se sigue reiniciando, bajar los dos valores. Si se siente
// "pesado", subirlos.
const int RAMPA_ACELERAR = 25;
const int RAMPA_FRENAR = 50;

// Que hace la rueda cuando su velocidad llega a 0:
//   true  = freno activo (el TB6612 pone el motor en corto y frena en seco)
//   false = rueda libre (el robot sigue deslizando)
const bool FRENO_ACTIVO = true;

// ============================================================
// QUE ROBOT ES ESTE
// ============================================================
// Normalmente lo define platformio.ini con -D ROBOT=1 / -D ROBOT=2.
// Este bloque es por si se compila sin ese flag (Arduino IDE): cambiar
// el numero a mano antes de flashear.
#ifndef ROBOT
#define ROBOT 1
#endif

// ============================================================
// CORRECCION DE MOTORES AL REVES
// ============================================================
// Si el robot avanza cuando deberia retroceder, o gira para el lado
// equivocado, invertir el motor que corresponda y volver a flashear.
// No hace falta tocar cables.
#if ROBOT == 1
const bool INVERTIR_A = false;
const bool INVERTIR_B = false;
const bool invertir_giro = true;
#elif ROBOT == 2
const bool INVERTIR_A = true;
const bool INVERTIR_B = true;
const bool invertir_giro = false;
#else
const bool INVERTIR_A = false;
const bool INVERTIR_B = false;
const bool invertir_giro = false;
#endif

// MAC del unico control permitido para ESTE robot.
// Dejar en ceros la primera vez: el ESP32 acepta cualquier control e
// imprime su MAC por serie. Copiarla aca, volver a flashear, y desde
// entonces solo ese control puede manejar este robot.

#if ROBOT == 1
const uint8_t MAC_PERMITIDA[6] = {0x90, 0xB6, 0x85, 0x12, 0x9A, 0x65};  // control 1
#elif ROBOT == 2
const uint8_t MAC_PERMITIDA[6] = {0x58, 0x10, 0x31, 0x7E, 0x52, 0x55};  // control 2
#else
const uint8_t MAC_PERMITIDA[6] = {0, 0, 0, 0, 0, 0};  // acepta cualquiera
#endif

// Puntero para guardar el control de PS5
ControllerPtr miControl = nullptr;

bool macEsCero(const uint8_t* mac) {
    for (int i = 0; i < 6; i++) if (mac[i] != 0) return false;
    return true;
}

bool macCoincide(const uint8_t* a, const uint8_t* b) {
    for (int i = 0; i < 6; i++) if (a[i] != b[i]) return false;
    return true;
}

// ============================================================
// MOTOR A (IZQUIERDO)
// ============================================================
void motorA(int velocidad) {
    if (INVERTIR_A) velocidad = -velocidad;
    velocidad = constrain(velocidad, -255, 255);
    if (velocidad > 0) {
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);
        ledcWrite(PWM_CHANNEL_A, velocidad);
    } else if (velocidad < 0) {
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, HIGH);
        ledcWrite(PWM_CHANNEL_A, -velocidad);
    } else {
        // Freno activo: IN1 = IN2 = HIGH. Rueda libre: IN1 = IN2 = LOW.
        digitalWrite(PIN_AIN1, FRENO_ACTIVO ? HIGH : LOW);
        digitalWrite(PIN_AIN2, FRENO_ACTIVO ? HIGH : LOW);
        ledcWrite(PWM_CHANNEL_A, 0);
    }
}

// ============================================================
// MOTOR B (DERECHO)
// ============================================================
void motorB(int velocidad) {
    if (INVERTIR_B) velocidad = -velocidad;
    velocidad = constrain(velocidad, -255, 255);
    if (velocidad > 0) {
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, LOW);
        ledcWrite(PWM_CHANNEL_B, velocidad);
    } else if (velocidad < 0) {
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, HIGH);
        ledcWrite(PWM_CHANNEL_B, -velocidad);
    } else {
        // Freno activo: IN1 = IN2 = HIGH. Rueda libre: IN1 = IN2 = LOW.
        digitalWrite(PIN_BIN1, FRENO_ACTIVO ? HIGH : LOW);
        digitalWrite(PIN_BIN2, FRENO_ACTIVO ? HIGH : LOW);
        ledcWrite(PWM_CHANNEL_B, 0);
    }
}

// ============================================================
// RAMPA
// ============================================================
// Velocidad que tiene ahora cada rueda (la que se le mando en el ultimo ciclo).
int velocidadActualA = 0;
int velocidadActualB = 0;

// Acerca "actual" a "objetivo" sin cambiar mas de un paso por ciclo.
// Si se aleja de 0 usa RAMPA_ACELERAR; si se acerca a 0 usa RAMPA_FRENAR.
// Un cambio de sentido primero frena hasta 0 y despues acelera al reves.
int aplicarRampa(int objetivo, int actual) {
    bool acelerando = (objetivo > actual) ? (actual >= 0) : (actual <= 0);
    int paso = acelerando ? RAMPA_ACELERAR : RAMPA_FRENAR;
    if (objetivo > actual) return min(objetivo, actual + paso);
    if (objetivo < actual) return max(objetivo, actual - paso);
    return actual;
}

// Parada inmediata, sin rampa (seguridad: mando desconectado).
void detenerMotores() {
    velocidadActualA = 0;
    velocidadActualB = 0;
    motorA(0);
    motorB(0);
}

// ============================================================
// PROCESAMIENTO DE ENTRADAS
// ============================================================
// Stick: aplica zona muerta y reescala desde el borde de la zona hasta 512.
// Asi el centro da exactamente 0 y no hay salto al salir de la zona muerta.
int procesarEje(int valor, int zonaMuerta) {
    if (abs(valor) < zonaMuerta) return 0;
    int signo = (valor > 0) ? 1 : -1;
    int magnitud = map(abs(valor), zonaMuerta, 512, 0, VELOCIDAD_MAXIMA);
    return signo * constrain(magnitud, 0, VELOCIDAD_MAXIMA);
}

// Gatillo (0 a 1023): zona muerta y reescala a 0..VELOCIDAD_MAXIMA.
int procesarGatillo(int valor) {
    if (valor < ZONA_MUERTA_GATILLO) return 0;
    int magnitud = map(valor, ZONA_MUERTA_GATILLO, 1023, 0, VELOCIDAD_MAXIMA);
    return constrain(magnitud, 0, VELOCIDAD_MAXIMA);
}

// ============================================================
// EVENTOS BLUEPAD32 (PS5)
// ============================================================
void onConnectedController(ControllerPtr ctl) {
    ControllerProperties p = ctl->getProperties();
    Serial.printf("Control detectado, MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  p.btaddr[0], p.btaddr[1], p.btaddr[2],
                  p.btaddr[3], p.btaddr[4], p.btaddr[5]);

    // FILTRO POR MAC: si hay una MAC configurada y no coincide, rechazo
    if (!macEsCero(MAC_PERMITIDA) && !macCoincide(p.btaddr, MAC_PERMITIDA)) {
        Serial.println("Control NO autorizado para este robot, rechazado.");
        ctl->disconnect();
        return;
    }

    if (miControl == nullptr) {
        miControl = ctl;
        // Ya tengo mi control: no acepto conexiones nuevas
        BP32.enableNewBluetoothConnections(false);
        Serial.println();
        Serial.println("==============================");
        Serial.println("MANDO PS5 CONECTADO");
        Serial.println("==============================");
    } else {
        // Ya hay un control asignado, rechazo este
        ctl->disconnect();
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    if (miControl == ctl) {
        miControl = nullptr;
        detenerMotores();
        // Vuelvo a aceptar conexiones para poder reconectar
        BP32.enableNewBluetoothConnections(true);
        Serial.println();
        Serial.println("==============================");
        Serial.println("MANDO PS5 DESCONECTADO");
        Serial.println("MOTORES DETENIDOS");
        Serial.println("==============================");
    }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);
    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, HIGH);

    ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWMA, PWM_CHANNEL_A);

    ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWMB, PWM_CHANNEL_B);

    detenerMotores();

    // Iniciar Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // Descomentar solo en el primer flasheo si el control no conecta,
    // borra pareos viejos. Despues volver a comentar.
    // BP32.forgetBluetoothKeys();

    Serial.println();
    Serial.println("==============================");
    Serial.println("ESP32 + TB6612 + PS5 (Bluepad32)");
    Serial.println("==============================");
    Serial.println("Pon tu control de PS5 en modo emparejamiento (Create + PS)...");
}

// ============================================================
// LOOP
// ============================================================

void loop() {
    // Actualizar datos del Bluetooth
    BP32.update();

    // SEGURIDAD: SIN MANDO = MOTORES DETENIDOS
    if (!miControl || !miControl->isConnected()) {
        detenerMotores();
        delay(50);
        return;
    }

    // LEER ENTRADAS
    // R2 = throttle (0 a 1023), L2 = brake (0 a 1023), stick izq X (-511 a 512)
    int gatilloAdelante = procesarGatillo(miControl->throttle());
    int gatilloAtras    = procesarGatillo(miControl->brake());
    int ejeX            = miControl->axisX();
    if (invertir_giro) ejeX = -ejeX;

    // AVANCE NETO: R2 suma, L2 resta (positivo = adelante, negativo = atras)
    int avance = gatilloAdelante - gatilloAtras;

    // GIRO: positivo = derecha, negativo = izquierda
    int giro = procesarEje(ejeX, ZONA_MUERTA_X);

    int velocidadA;  // izquierdo
    int velocidadB;  // derecho

    if (avance == 0) {
        // SIN GATILLOS: giro sobre su propio eje (una rueda para cada lado)
        int g = map(abs(giro), 0, VELOCIDAD_MAXIMA, 0, VELOCIDAD_GIRO_SOBRE_EJE);
        g = (giro >= 0) ? g : -g;
        velocidadA = g;
        velocidadB = -g;
    } else {
        // CON GATILLO: las dos ruedas siguen el avance, y la rueda del lado
        // hacia el que giras se frena segun cuanto muevas el stick.
        velocidadA = avance;
        velocidadB = avance;

        float cuanto = (float)abs(giro) / VELOCIDAD_MAXIMA;          // 0.0 a 1.0
        float factorInterior = 1.0 - (1.0 + CIERRE_GIRO_EN_MARCHA) * cuanto;

        if (giro > 0) {
            velocidadB = (int)(avance * factorInterior);   // giro a la derecha
        } else if (giro < 0) {
            velocidadA = (int)(avance * factorInterior);   // giro a la izquierda
        }
    }

    // CALIBRACION ENTRE MOTORES
    velocidadA = (int)(velocidadA * FACTOR_A);
    velocidadB = (int)(velocidadB * FACTOR_B);

    velocidadA = constrain(velocidadA, -VELOCIDAD_MAXIMA, VELOCIDAD_MAXIMA);
    velocidadB = constrain(velocidadB, -VELOCIDAD_MAXIMA, VELOCIDAD_MAXIMA);

    // RAMPA: cada rueda se acerca de a poco a la velocidad pedida
    velocidadActualA = aplicarRampa(velocidadA, velocidadActualA);
    velocidadActualB = aplicarRampa(velocidadB, velocidadActualB);

    // MOVER MOTORES

    // La inversion por robot se aplica adentro de motorA/motorB
    // (constantes INVERTIR_A / INVERTIR_B arriba).
    motorA(velocidadActualA);
    motorB(velocidadActualB);

    // DEBUG (Solo imprime si hay movimiento para no saturar)
    if (avance != 0 || giro != 0 || velocidadActualA != 0 || velocidadActualB != 0) {
        Serial.print("R2: ");
        Serial.print(gatilloAdelante);
        Serial.print(" | L2: ");
        Serial.print(gatilloAtras);
        Serial.print(" | X(Giro): ");
        Serial.print(ejeX);
        Serial.print(" | MotorA: ");
        Serial.print(velocidadActualA);
        Serial.print(" | MotorB: ");
        Serial.println(velocidadActualB);
    }

    delay(20);
}
/*void loop() {
    Serial.println("A adelante");  motorA(150); delay(1500); motorA(0); delay(500);
    Serial.println("A atras");     motorA(-150); delay(1500); motorA(0); delay(500);
    Serial.println("B adelante");  motorB(150); delay(1500); motorB(0); delay(500);
    Serial.println("B atras");     motorB(-150); delay(1500); motorB(0); delay(500);
}*/
