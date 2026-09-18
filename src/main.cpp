/*
 * ============================================================================
 *  Tono senoidal de prueba para verificar la cadena de audio completa
 * ============================================================================
 *
 *  ESP32 --I2S--> GY-PCM5102 (DAC) --linea--> HW-323 / PAM8403 --> parlante
 *
 *  Objetivo: confirmar que toda la cadena esta bien cableada antes de montar
 *  el proyecto grande encima.
 *
 *  ---------------------------------------------------------------------------
 *  CABLEADO
 *  ---------------------------------------------------------------------------
 *
 *  ESP32 DevKit (WROOM-32, 30 pines, CP2102) -> GY-PCM5102:
 *
 *      ESP32          PCM5102     Notas
 *      -------------------------------------------------------------------
 *      GPIO 26   ->   BCK         Bit clock (reloj de bit)
 *      GPIO 22   ->   DIN         Datos serie hacia el DAC
 *      GPIO 25   ->   LRCK        Word/LR clock (reloj de canal)
 *      Riel GND  ->   SCK         A tierra = el chip usa su PLL interno y
 *                                 NO espera MCLK del ESP32
 *      3V3       ->   XSMT        En alto = fuera del soft mute
 *      Riel +5V  ->   VIN         Alimentacion (el modulo lleva su regulador)
 *      Riel GND  ->   GND         Masa comun
 *
 *  GY-PCM5102 -> HW-323 (PAM8403):
 *
 *      LROUT (canal izquierdo)  ->  L   (entrada)
 *      AGND contiguo            ->  GND L
 *
 *  HW-323 -> parlante (40 mm, 4 ohm, 3 W):
 *
 *      L+  ->  un terminal del parlante
 *      L-  ->  el otro terminal
 *
 *      OJO: el PAM8403 es clase D con salida PUENTEADA (BTL). Sus dos salidas
 *      estan activas; NINGUNA va a tierra. Conectar L- a GND puede matar el
 *      chip. El cableado descrito arriba es el correcto.
 *
 *      El canal derecho del ampli queda sin usar.
 *
 *  Alimentacion: todo cuelga del USB, via VIN/GND del ESP32 a los rieles.
 *
 *  El jack de 3.5 mm del modulo DAC va en paralelo con la entrada del ampli,
 *  para escuchar con audifonos.
 *
 *  ---------------------------------------------------------------------------
 *  ANTES DE ENCENDER: audifonos fuera de las orejas.
 *  ---------------------------------------------------------------------------
 *
 *  Framework: Arduino-ESP32 core 2.0.x (ESP-IDF 4.4.x) -> driver I2S "legacy"
 *  de <driver/i2s.h>. Ver la nota sobre el core 3.x al final del archivo.
 * ============================================================================
 */

#include <Arduino.h>
#include <driver/i2s.h>
#include <WiFi.h>       // solo para poder apagar el WiFi
#include <esp_bt.h>     // solo para poder apagar el Bluetooth
#include <math.h>

// ---------------------------------------------------------------------------
// Pines (segun el cableado de arriba)
// ---------------------------------------------------------------------------
static const int PIN_BCK  = 26;   // BCK  del PCM5102
static const int PIN_LRCK = 25;   // LRCK / WS
static const int PIN_DIN  = 22;   // DIN  (salida de datos del ESP32)

// Usamos el periferico I2S numero 0
static const i2s_port_t PUERTO_I2S = I2S_NUM_0;

// ---------------------------------------------------------------------------
// Parametros de audio
// ---------------------------------------------------------------------------
static const uint32_t FRECUENCIA_MUESTREO = 44100;  // Hz
static const float    FRECUENCIA_TONO     = 440.0f; // Hz (La4)

/* ===========================================================================
 *  CONTROL DE AMPLITUD Y LIMITE DE SEGURIDAD
 * ===========================================================================
 *
 *  POR QUE EL VOLUMEN SE CONTROLA AQUI Y NO EN EL AMPLIFICADOR
 *  ------------------------------------------------------------------------
 *  El HW-323 no tiene control de volumen: el PAM8403 amplifica con ganancia
 *  FIJA de 24 dB (x15.85 en tension). El PCM5102A entrega nivel de linea, que
 *  para ese ampli es una senal ENORME. El unico sitio donde se puede bajar el
 *  volumen es en el dominio digital, antes del DAC. Es decir, aqui.
 *
 *  DE DONDE SALE EL LIMITE
 *  ------------------------------------------------------------------------
 *  Cadena de niveles, con el ampli alimentado a 5 V:
 *
 *      Salida del DAC a escala completa .......... 2.1  Vrms
 *      Ganancia del PAM8403 (24 dB) .............. x15.85
 *      Tension de recorte del ampli (3 W / 4 ohm)  3.46 Vrms
 *
 *      Entrada maxima antes de recortar = 3.46 / 15.85 = 0.218 Vrms
 *      Como fraccion de escala completa = 0.218 / 2.1  = 0.104  -> 10.4 %
 *
 *  O sea: pasado el ~10.4 % de escala completa el amplificador RECORTA. El
 *  datasheet del PAM8403 advierte que el recorte sostenido puede danar el
 *  chip, ademas de sonar horrible y de castigar al parlante.
 *
 *  Por eso AMPLITUD_MAXIMA esta en 0.08: deja unos 2 dB de margen por debajo
 *  del recorte, que cubre la tolerancia del nivel real del DAC y las caidas
 *  del riel de 5 V bajo carga.
 *
 *  NO SUBAS ESTE TECHO SIN LEER ESTO
 *  ------------------------------------------------------------------------
 *  Si algun dia tienes la tentacion de subir AMPLITUD_MAXIMA, primero cambia
 *  el hardware, no el numero:
 *    - pon un divisor resistivo entre el DAC y el ampli, o
 *    - usa un ampli con control de ganancia, o
 *    - alimenta el ampli con menos tension.
 *  Subir el numero a secas solo mete al PAM8403 en recorte permanente.
 *
 *  El static_assert de abajo hace que el build FALLE si alguien sube el techo
 *  por encima del umbral fisico de recorte. Es intencionado.
 * ===========================================================================
 */

// Techo absoluto. Ninguna amplitud aplicada podra superar este valor.
static constexpr float AMPLITUD_MAXIMA = 0.08f;

// Guardia en tiempo de compilacion: 0.104 es el umbral fisico de recorte.
static_assert(AMPLITUD_MAXIMA <= 0.10f,
              "AMPLITUD_MAXIMA por encima de 0.10 mete al PAM8403 en recorte. "
              "Lee el comentario de arriba antes de tocar esto.");

// >>> ESTA ES LA CONSTANTE QUE AJUSTAS A MANO <<<
// Fraccion de escala completa de 16 bits que quieres reproducir.
// Si pides mas que AMPLITUD_MAXIMA, el codigo la recorta y te avisa por Serial.
//
//    Valor   Vrms al ampli   Potencia (4 ohm)   Comentario
//    -----------------------------------------------------------------
//    0.005       0.17 V           7 mW          apenas audible
//    0.01        0.33 V          28 mW          bajito
//    0.02        0.67 V         111 mW          suficiente para la prueba
//    0.05        1.66 V         692 mW          fuerte
//    0.08        2.66 V         1.77 W          <-- TECHO
//    0.10        3.33 V         2.77 W          recorte + brownout por USB
//
// Para verificar cableado no hace falta pasar de 0.02 o 0.03.
static const float AMPLITUD_SOLICITADA = 0.10f;

// Amplitud realmente aplicada tras el limitador (se calcula en setup)
static float amplitudEfectiva = 0.0f;

// ---------------------------------------------------------------------------
// Tabla de onda y acumulador de fase
// ---------------------------------------------------------------------------
// En vez de llamar a sinf() en cada muestra, precalculamos un ciclo completo
// de senoide en una tabla y la recorremos con un acumulador de fase de 32 bits.
// Asi la frecuencia sale exacta aunque 44100 / 440 no sea un numero entero.
static const uint32_t TAMANO_TABLA   = 1024;      // potencia de 2
static const uint32_t DESPLAZAMIENTO = 32 - 10;   // 2^10 = 1024 entradas
static int16_t tablaSeno[TAMANO_TABLA];

static uint32_t fase = 0;           // acumulador (0 .. 2^32-1 = un ciclo)
static uint32_t incrementoFase = 0; // cuanto avanza la fase por muestra

// Buffer de salida: muestras entrelazadas L,R (estereo, 16 bits con signo)
static const size_t MUESTRAS_POR_BLOQUE = 256;              // por canal
static int16_t bufferAudio[MUESTRAS_POR_BLOQUE * 2];        // *2 por estereo

// ---------------------------------------------------------------------------
// Aplica el limite duro de amplitud y explica por Serial lo que hizo
// ---------------------------------------------------------------------------
static float aplicarLimiteAmplitud(float solicitada) {
  // Un valor negativo o absurdo no tiene sentido: lo tratamos como silencio.
  if (!(solicitada > 0.0f)) {
    Serial.println(F("[ADV] AMPLITUD_SOLICITADA no es valida. Se usa silencio."));
    return 0.0f;
  }

  if (solicitada > AMPLITUD_MAXIMA) {
    Serial.println(F("[LIMITE] ------------------------------------------"));
    Serial.printf("[LIMITE] Pediste %.1f%% pero el techo son %.1f%%.\n",
                  solicitada * 100.0f, AMPLITUD_MAXIMA * 100.0f);
    Serial.println(F("[LIMITE] Por encima del 10.4% el PAM8403 recorta y se"));
    Serial.println(F("[LIMITE] puede danar. Aplico el techo en su lugar."));
    Serial.println(F("[LIMITE] ------------------------------------------"));
    return AMPLITUD_MAXIMA;
  }

  return solicitada;
}

// ---------------------------------------------------------------------------
// Apaga las radios para reducir ruido de RF sobre la senal analogica
// ---------------------------------------------------------------------------
static void apagarRadios() {
  WiFi.disconnect(true);      // true = tambien apaga el stack
  WiFi.mode(WIFI_OFF);

  // btStop() detiene el controlador si estaba encendido. Devuelve false si ya
  // estaba apagado, que es el caso normal aqui, asi que no comprobamos nada.
  btStop();

  Serial.println(F("[OK]  WiFi y Bluetooth desactivados"));
}

// ---------------------------------------------------------------------------
// Rellena la tabla con un ciclo de senoide, escalado por amplitudEfectiva
// ---------------------------------------------------------------------------
static void generarTablaSeno() {
  // 32767 es el maximo de un int16_t. Escalamos por la amplitud ya limitada.
  const float pico = 32767.0f * amplitudEfectiva;

  for (uint32_t i = 0; i < TAMANO_TABLA; i++) {
    float angulo = 2.0f * (float)M_PI * (float)i / (float)TAMANO_TABLA;
    tablaSeno[i] = (int16_t)lroundf(pico * sinf(angulo));
  }

  // Incremento de fase por muestra, en unidades de 2^32 por ciclo:
  //   incremento = frecuencia_tono / frecuencia_muestreo * 2^32
  incrementoFase = (uint32_t)((FRECUENCIA_TONO * 4294967296.0) /
                              (double)FRECUENCIA_MUESTREO);

  // Estimacion de nivel: DAC 2.1 Vrms a escala completa, ampli x15.85, 4 ohm
  const float vDac  = amplitudEfectiva * 2.1f;
  const float vAmp  = vDac * 15.85f;
  const float watts = (vAmp * vAmp) / 4.0f;

  Serial.printf("[OK]  Tabla de seno lista: %u entradas, pico +/-%d LSB\n",
                (unsigned)TAMANO_TABLA, (int)lroundf(pico));
  Serial.printf("[OK]  Nivel estimado: %.3f Vrms al ampli -> %.2f Vrms -> %.0f mW\n",
                vDac, vAmp, watts * 1000.0f);
}

// ---------------------------------------------------------------------------
// Inicializa el driver I2S. Devuelve true si todo salio bien.
// ---------------------------------------------------------------------------
static bool iniciarI2S() {
  esp_err_t err;

  // --- Configuracion del periferico ---------------------------------------
  // OJO: en C++ los inicializadores designados deben ir EN EL MISMO ORDEN que
  // los campos de la estructura, si no el compilador da error.
  i2s_config_t configuracion = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), // ESP32 maestro, solo transmite
      .sample_rate = FRECUENCIA_MUESTREO,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,        // estereo entrelazado
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,   // I2S estandar (Philips)
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,                                  // 8 buffers DMA...
      .dma_buf_len = 256,                                  // ...de 256 muestras c/u
      .use_apll = false,                                   // PLL normal: suficiente y estable
      .tx_desc_auto_clear = true,                          // si falta dato, saca silencio (no basura)
      .fixed_mclk = 0,
      .mclk_multiple = I2S_MCLK_MULTIPLE_256,
      .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT,
  };

  // --- Asignacion de pines -------------------------------------------------
  i2s_pin_config_t pines = {
      .mck_io_num = I2S_PIN_NO_CHANGE,  // sin MCLK: el PCM5102 lo genera solo (SCK a GND)
      .bck_io_num = PIN_BCK,
      .ws_io_num = PIN_LRCK,
      .data_out_num = PIN_DIN,
      .data_in_num = I2S_PIN_NO_CHANGE, // no recibimos nada
  };

  Serial.println(F("[..]  Instalando driver I2S..."));
  err = i2s_driver_install(PUERTO_I2S, &configuracion, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("[ERR] i2s_driver_install fallo: %s (0x%X)\n", esp_err_to_name(err), err);
    return false;
  }
  Serial.println(F("[OK]  Driver I2S instalado"));

  Serial.println(F("[..]  Asignando pines I2S..."));
  err = i2s_set_pin(PUERTO_I2S, &pines);
  if (err != ESP_OK) {
    Serial.printf("[ERR] i2s_set_pin fallo: %s (0x%X)\n", esp_err_to_name(err), err);
    i2s_driver_uninstall(PUERTO_I2S);   // no dejamos el driver a medias
    return false;
  }
  Serial.printf("[OK]  Pines asignados -> BCK=GPIO%d  LRCK=GPIO%d  DIN=GPIO%d\n",
                PIN_BCK, PIN_LRCK, PIN_DIN);

  // Arrancamos con los buffers DMA en silencio para suavizar el "pop" inicial
  err = i2s_zero_dma_buffer(PUERTO_I2S);
  if (err != ESP_OK) {
    Serial.printf("[ADV] i2s_zero_dma_buffer fallo: %s\n", esp_err_to_name(err));
    // No es fatal: seguimos igual.
  }

  return true;
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
static bool i2sListo = false;

void setup() {
  Serial.begin(115200);
  delay(300);   // margen para que el monitor serie se enganche

  Serial.println();
  Serial.println(F("=============================================="));
  Serial.println(F(" Tono de prueba: ESP32 -> PCM5102 -> PAM8403"));
  Serial.println(F("=============================================="));
  Serial.printf("Chip: %s  rev %d  a %d MHz\n",
                ESP.getChipModel(), ESP.getChipRevision(), getCpuFrequencyMhz());
  Serial.printf("Tono: %.1f Hz | Muestreo: %u Hz | 16 bits | estereo\n",
                FRECUENCIA_TONO, (unsigned)FRECUENCIA_MUESTREO);
  Serial.println(F("----------------------------------------------"));

  // 1) Limite de amplitud, antes que nada: es lo que protege al ampli
  amplitudEfectiva = aplicarLimiteAmplitud(AMPLITUD_SOLICITADA);
  Serial.printf("[OK]  Amplitud solicitada: %.1f%% | techo: %.1f%% | aplicada: %.1f%%\n",
                AMPLITUD_SOLICITADA * 100.0f,
                AMPLITUD_MAXIMA * 100.0f,
                amplitudEfectiva * 100.0f);

  // 2) Radios apagadas antes de tocar el audio: menos ruido de RF en la salida
  apagarRadios();

  // 3) Tabla de onda, ya escalada por la amplitud efectiva
  generarTablaSeno();

  // 4) I2S
  i2sListo = iniciarI2S();

  Serial.println(F("----------------------------------------------"));
  if (i2sListo) {
    Serial.println(F("[OK]  I2S inicializado. Deberias oir el tono AHORA."));
    Serial.println(F("      Si no se oye, revisa en este orden:"));
    Serial.println(F("        1. GND comun: ESP32, DAC y ampli en el mismo riel"));
    Serial.println(F("        2. XSMT del DAC a 3V3 (si esta bajo, hay mute)"));
    Serial.println(F("        3. SCK del DAC a GND (obligatorio sin MCLK)"));
    Serial.println(F("        4. BCK/LRCK/DIN en 26/25/22, sin intercambiar"));
    Serial.println(F("        5. LROUT del DAC a L del ampli, con su AGND"));
    Serial.println(F("        6. Parlante entre L+ y L-, NUNCA a GND"));
  } else {
    Serial.println(F("[ERR] I2S NO se pudo inicializar. No habra sonido."));
    Serial.println(F("      Revisa que ningun otro codigo use I2S_NUM_0."));
  }
  Serial.println(F("=============================================="));
}

// ---------------------------------------------------------------------------
// loop(): rellena el buffer y lo empuja al I2S. i2s_write bloquea hasta que
// hay hueco en el DMA, asi que este bucle se auto-regula al ritmo del audio.
// ---------------------------------------------------------------------------
void loop() {
  if (!i2sListo) {
    // Sin I2S no hay nada que hacer: avisamos cada 2 s y esperamos.
    Serial.println(F("[ERR] I2S no inicializado; nada que reproducir."));
    delay(2000);
    return;
  }

  // --- Generamos un bloque de muestras ------------------------------------
  for (size_t i = 0; i < MUESTRAS_POR_BLOQUE; i++) {
    // Indice en la tabla = los 10 bits mas altos del acumulador de fase
    uint32_t indice = fase >> DESPLAZAMIENTO;
    int16_t muestra = tablaSeno[indice];

    bufferAudio[i * 2]     = muestra;  // canal izquierdo (el que va al ampli)
    bufferAudio[i * 2 + 1] = muestra;  // canal derecho (mismo tono)

    // El acumulador desborda solo al final del ciclo: fase continua, sin saltos
    fase += incrementoFase;
  }

  // --- Lo enviamos al DAC --------------------------------------------------
  size_t bytesEscritos = 0;
  esp_err_t err = i2s_write(PUERTO_I2S,
                            bufferAudio,
                            sizeof(bufferAudio),
                            &bytesEscritos,
                            portMAX_DELAY);

  if (err != ESP_OK) {
    Serial.printf("[ERR] i2s_write fallo: %s (0x%X)\n", esp_err_to_name(err), err);
    delay(500);
    return;
  }
  if (bytesEscritos != sizeof(bufferAudio)) {
    Serial.printf("[ADV] i2s_write escribio %u de %u bytes\n",
                  (unsigned)bytesEscritos, (unsigned)sizeof(bufferAudio));
  }

  // --- Latido cada ~5 s para saber que el bucle sigue vivo -----------------
  // Si el uptime se reinicia solo, el riel de 5 V se esta hundiendo: baja la
  // amplitud o alimenta el ampli con una fuente aparte.
  static uint32_t ultimoLatido = 0;
  uint32_t ahora = millis();
  if (ahora - ultimoLatido >= 5000) {
    ultimoLatido = ahora;
    Serial.printf("[..]  Reproduciendo %.1f Hz al %.1f%% | uptime %lu s | heap %u B\n",
                  FRECUENCIA_TONO, amplitudEfectiva * 100.0f,
                  (unsigned long)(ahora / 1000), (unsigned)ESP.getFreeHeap());
  }
}

/*
 * ============================================================================
 *  NOTA SOBRE LA VERSION DEL CORE ARDUINO-ESP32
 * ============================================================================
 *
 *  Este archivo asume Arduino-ESP32 core 2.0.x (el que trae
 *  platform = espressif32@6.12.0, concretamente 2.0.17 sobre ESP-IDF 4.4.7).
 *  Ahi <driver/i2s.h> es la API oficial y compila sin avisos.
 *
 *  Con core 3.x (ESP-IDF 5.x, que en PlatformIO se obtiene con el fork
 *  "pioarduino", p.ej. platform = https://github.com/pioarduino/platform-espressif32
 *  release 51.x/54.x) cambia lo siguiente:
 *
 *   - <driver/i2s.h> sigue existiendo por compatibilidad pero esta DEPRECADO:
 *     compila y funciona, aunque el build escupe warnings de "legacy driver".
 *     Si solo quieres salir del paso, este sketch sirve tal cual.
 *
 *   - La API nueva es <driver/i2s_std.h>, con canales en vez de "puertos":
 *
 *       #include <driver/i2s_std.h>
 *       i2s_chan_handle_t canalTx;
 *       i2s_chan_config_t cfgCanal = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0,
 *                                                               I2S_ROLE_MASTER);
 *       i2s_new_channel(&cfgCanal, &canalTx, NULL);   // NULL = sin RX
 *
 *       i2s_std_config_t cfgStd = {
 *           .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(44100),
 *           .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
 *                           I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
 *           .gpio_cfg = {
 *               .mclk = I2S_GPIO_UNUSED,
 *               .bclk = GPIO_NUM_26,
 *               .ws   = GPIO_NUM_25,
 *               .dout = GPIO_NUM_22,
 *               .din  = I2S_GPIO_UNUSED,
 *               .invert_flags = { false, false, false },
 *           },
 *       };
 *       i2s_channel_init_std_mode(canalTx, &cfgStd);
 *       i2s_channel_enable(canalTx);                  // <-- paso nuevo, obligatorio
 *
 *       // y en el loop, en vez de i2s_write():
 *       i2s_channel_write(canalTx, bufferAudio, sizeof(bufferAudio),
 *                         &bytesEscritos, portMAX_DELAY);
 *
 *     Equivalencias rapidas:
 *       i2s_driver_install + i2s_set_pin  ->  i2s_new_channel + i2s_channel_init_std_mode
 *       (no existia)                      ->  i2s_channel_enable   [hace falta!]
 *       i2s_write                         ->  i2s_channel_write
 *       i2s_zero_dma_buffer               ->  i2s_channel_preload_data / innecesario
 *       i2s_driver_uninstall              ->  i2s_channel_disable + i2s_del_channel
 *
 *  Ni la generacion del tono ni el limitador de amplitud cambian entre una API
 *  y la otra: solo cambia la capa de transporte.
 * ============================================================================
 */
