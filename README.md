# Don Satur

Firmware del robot de fútbol **Don Satur**: un ESP32 que recibe un mando de PS5 por Bluetooth (Bluepad32) y maneja dos motores con un driver TB6612FNG.

## Controles

| Mando | Acción |
| --- | --- |
| R2 | Avanzar (cuanto más apretado, más rápido) |
| L2 | Retroceder |
| Stick izquierdo (eje X) | Girar: en marcha si hay un gatillo apretado, sobre su eje si no |
| Flecha arriba / abajo | Sensibilidad de giro: más agresivo / más suave |

Si el mando se desconecta, los motores se detienen.

## Sensibilidad de giro

Hay 5 niveles. Cada toque de flecha sube o baja uno, el mando vibra para confirmar y la barra de luz cambia de color:

| Nivel | Color | Uso |
| --- | --- | --- |
| 1 | Azul | Muy suave, para apuntar fino |
| 2 | Celeste | |
| 3 | Verde | Normal (nivel de fábrica) |
| 4 | Amarillo | |
| 5 | Rojo | Agresivo |

Cada nivel combina una curva exponencial en el stick (el centro del recorrido queda suave y la potencia completa se guarda para el final) y la velocidad máxima de giro sobre el eje. Se ajustan en `NIVELES_GIRO`, en `src/main.cpp`.

El nivel queda guardado en la memoria del ESP32: si el robot se reinicia, arranca con el último nivel usado.

## Conexiones (ESP32 → TB6612FNG)

| Señal | Pin ESP32 |
| --- | --- |
| PWMA | 2 |
| AIN1 / AIN2 | 16 / 4 |
| PWMB | 19 |
| BIN1 / BIN2 | 5 / 18 |
| STBY | 17 |

Motor A = rueda izquierda, motor B = rueda derecha.

## Dos robots, un código

El mismo código sirve para los dos robots. Cuál es cuál se elige con la macro `ROBOT`:

- En PlatformIO, con `build_flags = -D ROBOT=1` (o `2`) en cada entorno de `platformio.ini`.
- En Arduino IDE, cambiando el `#define ROBOT 1` de `src/main.cpp` antes de flashear.

Para cada robot se configuran en `src/main.cpp`:

- `INVERTIR_A`, `INVERTIR_B` e `invertir_giro`, por si un motor o el giro quedan al revés.
- `MAC_PERMITIDA`, la MAC del único mando que puede manejarlo. Con la MAC en ceros acepta cualquier mando e imprime su MAC por el monitor serie (115200 baudios).

## Primer emparejamiento

1. Flashear y abrir el monitor serie.
2. Poner el mando en modo emparejamiento (Create + PS).
3. Si no conecta, descomentar `BP32.forgetBluetoothKeys();` en `setup()`, flashear una vez y volver a comentarlo.
