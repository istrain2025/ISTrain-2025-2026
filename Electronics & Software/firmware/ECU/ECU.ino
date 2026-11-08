// ============================================================
//  ECU PRINCIPAL
//  - Le o Victron SmartShunt por VE.Direct e reenvia por CAN ao TCMS
//  - Recebe do TCMS o texto para a matriz de LEDs 32x8 e mostra-o
//
//  ECU -> TCMS
//    0x0C04EFD0 : [0..3] tensao V (float)   [4..7] corrente A (float)
//    0x0C05EFD0 : [0..3] potencia W (float) [4..7] SOC por-mil (float)
//  TCMS -> ECU
//    0x0C03EFD0 : [0] DC Contactor  [1] Key Switch  (so informativo)
//    0x0C06EFD0 : texto da matriz, partido em pedacos
//                 [0] seq  [1] indice  [2] total  [3..7] 5 caracteres
//
//  NOTA: o controlo dos reles do DC Contactor e do Key Switch esta COMENTADO.
//        Esses reles sao comandados diretamente pelo TCMS (GPIO 17 e 8).
//
//  BIBLIOTECAS: Adafruit GFX, Adafruit NeoMatrix, Adafruit NeoPixel
// ============================================================

#include <Arduino.h>
#include "driver/twai.h"
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>

// ----- VE.Direct (SmartShunt) -----
#define VE_RX_PIN     16
#define VE_TX_PIN     17          // nao usado para transmitir; podes por -1 se precisares do GPIO17
#define VE_BAUD       19200
#define USE_OPTOCOUPLER true      // true = sinal invertido (optoacoplador)

// ----- MATRIZ DE LEDS 32x8 -----
#define MATRIX_PIN         4      // pino de dados da matriz
#define MATRIX_W          32
#define MATRIX_H           8
#define MATRIX_BRIGHTNESS 60      // menos brilho = menos picos de corrente = menos glitches
// Cor fixa do texto: AZUL. Nao muda em lado nenhum do codigo.
#define MATRIX_R           0
#define MATRIX_G           0
#define MATRIX_B         255

// Quando se carrega no STOP na dashboard, a matriz mostra isto a vermelho
#define STOP_TEXT      "PARAR"
#define STOP_R         255
#define STOP_G           0
#define STOP_B           0
#define STOP_BLINK     1       // 1 = pisca, 0 = fica aceso fixo
#define STOP_BLINK_MS  500     // meio segundo aceso, meio segundo apagado
#define MATRIX_SCROLL_MS  80      // menor = texto mais rapido
#define MATRIX_TEXT_MAX   32      // tem de ser igual no TCMS e no COMANDO
#define MATRIX_CHUNK      5       // caracteres por trama CAN

// ----- RELES (DESATIVADOS: o TCMS e que os controla) -----
// #define IN1_PIN       4         // DC Contactor  -> agora no TCMS, GPIO 17
// #define IN2_PIN       9         // Key Switch    -> agora no TCMS, GPIO 8

// ----- CAN -----
#define CAN_TX_PIN    5
#define CAN_RX_PIN    6

#define CAN_ID_TCMS_CMD    0x0C03EFD0   // TCMS -> ECU (estado dos reles)
#define CAN_ID_BATT_MSG1   0x0C04EFD0   // ECU  -> TCMS (tensao + corrente)
#define CAN_ID_BATT_MSG2   0x0C05EFD0   // ECU  -> TCMS (potencia + SOC)
#define CAN_ID_MATRIX_TXT  0x0C06EFD0   // TCMS -> ECU (texto da matriz)
#define CAN_ID_ECU_LOG     0x0C07EFD0   // ECU  -> TCMS (diagnostico)

// ----- DIAGNOSTICO -----
// A ECU nao tem WiFi nem dashboard. Manda um CODIGO por CAN e e o TCMS
// que o traduz para texto e o reencaminha para a consola da dashboard.
// [0] codigo  [1] nivel  [2..3] parametro 1  [4..5] parametro 2
#define LVL_INFO 0
#define LVL_OK   1
#define LVL_WARN 2
#define LVL_ERR  3

#define EL_BOOT_OK        1
#define EL_CAN_FAIL       2
#define EL_VE_SILENT      3
#define EL_VE_BACK        4
#define EL_VE_CHECKSUM    5
#define EL_VE_NO_SOC      6
#define EL_CAN_TXFAIL     7
#define EL_MATRIX_TEXT    8
#define EL_MATRIX_PARTIAL 9
#define EL_UART_FULL     10
#define EL_BLOCK_EMPTY   11
#define EL_STOP_DISPLAY  12

#define VE_SILENT_MS  5000    // sem blocos validos durante isto -> aviso
#define ECU_REPORT_MS 15000   // resumo periodico de contadores

// Poe a 1 para ver no monitor serie o estado que o TCMS esta a anunciar
#define DEBUG_TCMS_CMD 0

HardwareSerial veSerial(2);

Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(MATRIX_W, MATRIX_H, MATRIX_PIN,
  NEO_MATRIX_TOP     + NEO_MATRIX_LEFT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
  NEO_GRB            + NEO_KHZ800);

// ============================================================
// PARSER VE.DIRECT
// ============================================================
#define MAX_LABEL_LEN  9
#define MAX_VALUE_LEN  33
#define MAX_FIELDS     24     // campos de UM bloco (o SmartShunt manda ~18 no bloco do historico)

struct VEField {
  char label[MAX_LABEL_LEN];
  char value[MAX_VALUE_LEN];
  bool valid;
};

// So precisamos dos campos do bloco atual. A tabela "finalFields" que existia
// aqui enchia-se com os 18 campos H1..H18 do primeiro bloco e depois deixava
// cair, em silencio, os campos do segundo bloco que nao coubessem - era por
// isso que o SOC ficava sempre a 0 enquanto V, I e P chegavam bem.
VEField tempFields[MAX_FIELDS];
int tempFieldCount = 0;
uint8_t blockChecksum = 0;

enum ParseState { WAIT_CR, READ_LABEL, READ_VALUE };
ParseState state = WAIT_CR;
char labelBuf[MAX_LABEL_LEN];
char valueBuf[MAX_VALUE_LEN];
int labelIdx = 0, valueIdx = 0;
bool inChecksumField = false;

float batteryVoltage = 0.0;   // V
float batteryCurrent = 0.0;   // A
float batteryPower   = 0.0;   // W
float batterySOC     = 0.0;   // por-mil (876 = 87.6%) - o TCMS e que escala
bool  haveVoltage = false, haveCurrent = false, havePower = false, haveSoc = false;

// ============================================================
// MATRIZ: texto atual e remontagem das tramas CAN
// ============================================================
char matrixText[MATRIX_TEXT_MAX + 1] = "ISTrain";
int  matrixX = MATRIX_W;
int  matrixTextW = 0;          // largura em pixeis, calculada so quando o texto muda
bool matrixStatic = false;     // true = o texto cabe na matriz e fica parado
bool matrixNeedsDraw = true;   // ha um frame novo por desenhar
#define MATRIX_STATIC_REFRESH_MS 1000   // texto parado: redesenha 1x/s por seguranca
bool stopDisplay = false;      // a mostrar PARAR em vez do texto normal
bool stopLit = false;          // fase do piscar
int  stopX = 0;                // posicao x que centra PARAR
uint16_t stopColor = 0;
uint16_t matrixColor = 0;      // preenchida no setup()
unsigned long lastScroll = 0;

char     asmBuf[MATRIX_TEXT_MAX + 1];
int      asmSeq   = -1;      // -1 = ainda nao comecou nenhuma mensagem
uint8_t  asmTotal = 0;
uint32_t asmMask  = 0;       // bit por pedaco recebido

// ============================================================
// DIAGNOSTICO
// ============================================================
uint32_t canTxFail     = 0;   // twai_transmit falhados
uint32_t checksumFail  = 0;   // blocos VE.Direct rejeitados
unsigned long lastGoodBlock = 0;
unsigned long lastReport    = 0;
bool veSilentReported = false;
bool socEverSeen      = false;
bool socWarned        = false;
bool bootReported     = false;

void checkEcuHealth();

void ecuLog(uint8_t code, uint8_t level, uint16_t p1 = 0, uint16_t p2 = 0) {
  twai_message_t m = {};
  m.identifier = CAN_ID_ECU_LOG;
  m.extd = 1;
  m.rtr = 0;
  m.data_length_code = 8;
  m.data[0] = code;
  m.data[1] = level;
  m.data[2] = p1 & 0xFF;  m.data[3] = (p1 >> 8) & 0xFF;
  m.data[4] = p2 & 0xFF;  m.data[5] = (p2 >> 8) & 0xFF;
  twai_transmit(&m, pdMS_TO_TICKS(10));
  Serial.printf("[ECU LOG] codigo=%u nivel=%u p1=%u p2=%u\n", code, level, p1, p2);
}

// ============================================================
// ENVIO PARA O TCMS
// ============================================================
void sendDataToTCMS() {
  twai_message_t msg1 = {};
  msg1.identifier = CAN_ID_BATT_MSG1;
  msg1.extd = 1;
  msg1.rtr = 0;
  msg1.data_length_code = 8;
  memcpy(&msg1.data[0], &batteryVoltage, 4);
  memcpy(&msg1.data[4], &batteryCurrent, 4);
  if (twai_transmit(&msg1, pdMS_TO_TICKS(10)) != ESP_OK) canTxFail++;

  twai_message_t msg2 = {};
  msg2.identifier = CAN_ID_BATT_MSG2;
  msg2.extd = 1;
  msg2.rtr = 0;
  msg2.data_length_code = 8;
  memcpy(&msg2.data[0], &batteryPower, 4);
  memcpy(&msg2.data[4], &batterySOC, 4);
  if (twai_transmit(&msg2, pdMS_TO_TICKS(10)) != ESP_OK) canTxFail++;
}

// ============================================================
// PARSER VE.DIRECT
// ============================================================
void clearTempRecords() {
  for (int i = 0; i < MAX_FIELDS; i++) tempFields[i].valid = false;
  tempFieldCount = 0;
  blockChecksum = 0;
}

void storeField(const char* label, const char* value) {
  if (tempFieldCount >= MAX_FIELDS) return;
  strncpy(tempFields[tempFieldCount].label, label, MAX_LABEL_LEN - 1);
  tempFields[tempFieldCount].label[MAX_LABEL_LEN - 1] = '\0';
  strncpy(tempFields[tempFieldCount].value, value, MAX_VALUE_LEN - 1);
  tempFields[tempFieldCount].value[MAX_VALUE_LEN - 1] = '\0';
  tempFields[tempFieldCount].valid = true;
  tempFieldCount++;
}

// Bloco com checksum valido: lemos os campos que interessam diretamente.
// O SmartShunt divide a informacao por dois blocos (historico H1..H18 num,
// PID/V/I/P/CE/SOC/TTG/... no outro), por isso cada chamada so traz alguns
// campos. As variaveis globais guardam o ultimo valor conhecido de cada um.
void commitBlock() {
  bool touched = false;

  for (int i = 0; i < tempFieldCount; i++) {
    if (!tempFields[i].valid) continue;
    const char* lbl = tempFields[i].label;
    const char* val = tempFields[i].value;

    if (strcmp(lbl, "V") == 0) {
      batteryVoltage = atof(val) / 1000.0f;   // mV -> V
      haveVoltage = true; touched = true;
    } else if (strcmp(lbl, "I") == 0) {
      batteryCurrent = atof(val) / 1000.0f;   // mA -> A
      haveCurrent = true; touched = true;
    } else if (strcmp(lbl, "P") == 0) {
      batteryPower = atof(val);               // W
      havePower = true; touched = true;
    } else if (strcmp(lbl, "SOC") == 0) {
      batterySOC = atof(val);                 // por-mil
      haveSoc = true; touched = true;
      socEverSeen = true;
    }
  }

  lastGoodBlock = millis();
  if (veSilentReported) {
    veSilentReported = false;
    ecuLog(EL_VE_BACK, LVL_OK);
  }

  // Só falamos com o TCMS quando este bloco trouxe mesmo algo de util
  if (touched) sendDataToTCMS();
}

void processChar(char c) {
  blockChecksum += (uint8_t)c;
  switch (state) {
    case WAIT_CR:
      if (c == '\r') { state = READ_LABEL; labelIdx = 0; inChecksumField = false; }
      break;

    case READ_LABEL:
      if (c == '\n') break;
      if (c == '\t') {
        labelBuf[labelIdx] = '\0';
        inChecksumField = (strcmp(labelBuf, "Checksum") == 0);
        valueIdx = 0;
        state = READ_VALUE;
      } else if (labelIdx < MAX_LABEL_LEN - 1) {
        labelBuf[labelIdx++] = c;
      }
      break;

    case READ_VALUE:
      if (c == '\r') {
        valueBuf[valueIdx] = '\0';
        if (inChecksumField) {
          if (blockChecksum == 0) commitBlock();
          else                    checksumFail++;
          clearTempRecords();
        } else {
          storeField(labelBuf, valueBuf);
        }
        state = READ_LABEL;
        labelIdx = 0;
      } else if (!inChecksumField && valueIdx < MAX_VALUE_LEN - 1) {
        valueBuf[valueIdx++] = c;
      }
      break;
  }
}

// ============================================================
// MATRIZ DE LEDS
// ============================================================

// Mede o texto a serio (getTextBounds) e decide:
//   cabe nos 32 pixeis -> fica parado e centrado
//   nao cabe           -> desliza da direita para a esquerda
// Com a letra de tamanho 1, cabem ate 5 caracteres ("PARAR", "OLA", "IST").
void measureMatrixText() {
  int16_t x1, y1;
  uint16_t w, h;
  matrix.getTextBounds(matrixText, 0, 0, &x1, &y1, &w, &h);
  matrixTextW  = (int)w;
  matrixStatic = (matrixTextW <= matrix.width());
  matrixX = matrixStatic ? (matrix.width() - matrixTextW) / 2   // centrado
                         : matrix.width();                      // entra pela direita
  matrixNeedsDraw = true;
}

void drawMatrixAt(int x) {
  matrix.fillScreen(0);
  matrix.setTextColor(matrixColor);   // reposta a cada frame: a cor nunca pode derivar
  matrix.setCursor(x, 0);
  matrix.print(matrixText);
  matrix.show();
}

// Chamada quando o TCMS diz que o STOP foi carregado (ou libertado)
void setStopDisplay(bool on) {
  if (on == stopDisplay) return;
  stopDisplay = on;
  if (on) {
    stopLit = false;                 // o primeiro passo acende logo
    matrixNeedsDraw = true;
  } else {
    measureMatrixText();             // volta ao texto normal, do principio
  }
  ecuLog(EL_STOP_DISPLAY, on ? LVL_WARN : LVL_OK, on ? 1 : 0);
}

void applyMatrixText(const char* txt) {
  if (strcmp(txt, matrixText) == 0) return;        // ja e este, nao reinicia o scroll
  strncpy(matrixText, txt, MATRIX_TEXT_MAX);
  matrixText[MATRIX_TEXT_MAX] = '\0';
  measureMatrixText();                             // decide parado ou a deslizar
  ecuLog(EL_MATRIX_TEXT, LVL_INFO, (uint16_t)strlen(matrixText));
}

void handleMatrixFrame(const twai_message_t& rx) {
  if (rx.data_length_code < 8) return;

  uint8_t seq   = rx.data[0];
  uint8_t idx   = rx.data[1];
  uint8_t total = rx.data[2];

  if (total == 0 || total > 8 || idx >= total) return;

  // Mensagem nova: recomeca a remontagem
  if ((int)seq != asmSeq) {
    // Se a anterior ficou a meio, vale a pena dizer: houve tramas perdidas
    if (asmSeq >= 0 && asmTotal > 0 && asmMask != ((1UL << asmTotal) - 1)) {
      ecuLog(EL_MATRIX_PARTIAL, LVL_WARN, (uint16_t)asmSeq, (uint16_t)asmMask);
    }
    asmSeq   = (int)seq;
    asmTotal = total;
    asmMask  = 0;
    memset(asmBuf, 0, sizeof(asmBuf));
  }

  for (int i = 0; i < MATRIX_CHUNK; i++) {
    int p = idx * MATRIX_CHUNK + i;
    if (p < MATRIX_TEXT_MAX) asmBuf[p] = (char)rx.data[3 + i];
  }
  asmMask |= (1UL << idx);

  // Todos os pedacos chegaram?
  if (asmMask == ((1UL << asmTotal) - 1)) {
    asmBuf[MATRIX_TEXT_MAX] = '\0';
    applyMatrixText(asmBuf);
  }
}

// Sem delay(): o loop tem de continuar a ler o SmartShunt e o CAN
void updateMatrix() {
  // PARAR tem prioridade sobre qualquer texto
  if (stopDisplay) {
    if (!matrixNeedsDraw && millis() - lastScroll < STOP_BLINK_MS) return;
    lastScroll = millis();
    matrixNeedsDraw = false;
#if STOP_BLINK
    stopLit = !stopLit;
#else
    stopLit = true;
#endif
    matrix.fillScreen(0);
    if (stopLit) {
      matrix.setTextColor(stopColor);
      matrix.setCursor(stopX, 0);
      matrix.print(STOP_TEXT);
    }
    matrix.show();
    return;
  }

  if (matrixStatic) {
    // Texto parado: so desenha quando muda. O refresh lento corrige algum
    // frame que tenha chegado corrompido aos LEDs, sem gastar show() a toa.
    if (!matrixNeedsDraw && millis() - lastScroll < MATRIX_STATIC_REFRESH_MS) return;
    lastScroll = millis();
    matrixNeedsDraw = false;
    drawMatrixAt(matrixX);
    return;
  }

  // Texto comprido: desliza 1 pixel por passo
  if (millis() - lastScroll < MATRIX_SCROLL_MS) return;
  lastScroll = millis();
  drawMatrixAt(matrixX);
  if (--matrixX < -matrixTextW) matrixX = matrix.width();
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // ----- RELES DESATIVADOS: comandados pelo TCMS -----
  // pinMode(IN1_PIN, OUTPUT);
  // pinMode(IN2_PIN, OUTPUT);
  // digitalWrite(IN1_PIN, HIGH);   // HIGH = rele em repouso (modulo ativo-baixo)
  // digitalWrite(IN2_PIN, HIGH);

  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(MATRIX_BRIGHTNESS);
  matrixColor = matrix.Color(MATRIX_R, MATRIX_G, MATRIX_B);   // azul, fixo
  matrix.setTextColor(matrixColor);
  matrix.setTextSize(1);
  measureMatrixText();                // "ISTrain" tem 7 letras: nao cabe, desliza

  // PARAR: cor e posicao calculadas uma vez
  stopColor = matrix.Color(STOP_R, STOP_G, STOP_B);
  {
    int16_t x1, y1; uint16_t w, h;
    matrix.getTextBounds(STOP_TEXT, 0, 0, &x1, &y1, &w, &h);
    stopX = ((int)matrix.width() - (int)w) / 2;
    if (stopX < 0) stopX = 0;
  }
  matrix.fillScreen(0);
  matrix.show();

  // Cada matrix.show() de 256 LEDs demora ~8 ms. Com o buffer de origem (256 bytes)
  // isso chegava para perder bytes do SmartShunt e falhar blocos inteiros.
  veSerial.setRxBufferSize(1024);
  veSerial.begin(VE_BAUD, SERIAL_8N1, VE_RX_PIN, VE_TX_PIN, USE_OPTOCOUPLER);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN,
                                                               (gpio_num_t)CAN_RX_PIN,
                                                               TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  g_config.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED;

  bool canOk = (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) &&
               (twai_start() == ESP_OK);
  if (canOk) {
    Serial.println("[ECU PRINCIPAL] CAN iniciado (TX:5, RX:6). A ler SmartShunt...");
    Serial.println("[ECU PRINCIPAL] Matriz 32x8 no GPIO 4. Controlo de reles desativado.");
    // O anuncio de arranque so e enviado no loop, para dar tempo ao TCMS de estar pronto
  } else {
    Serial.println("[ECU PRINCIPAL] FALHA ao iniciar o CAN.");
    bootReported = true;   // nao adianta tentar avisar por CAN
  }

  lastGoodBlock = millis();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  uint32_t alerts;
  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
    if (alerts & TWAI_ALERT_BUS_OFF) {
      Serial.println("[ECU] BUS-OFF, a recuperar...");
      twai_initiate_recovery();
    }
    if (alerts & TWAI_ALERT_BUS_RECOVERED) {
      twai_start();                       // obrigatorio depois de recuperar
      Serial.println("[ECU] Barramento recuperado.");
    }
  }

  twai_message_t rx_msg;
  while (twai_receive(&rx_msg, 0) == ESP_OK) {

    if (rx_msg.identifier == CAN_ID_MATRIX_TXT) {
      handleMatrixFrame(rx_msg);
    }
    else if (rx_msg.identifier == CAN_ID_TCMS_CMD && rx_msg.data_length_code >= 2) {
      bool contactor_on = (rx_msg.data[0] == 1);
      bool keyswitch_on = (rx_msg.data[1] == 1);

      // Byte 6: o STOP foi carregado na dashboard -> matriz mostra PARAR.
      // A trama chega de 200 em 200 ms, por isso uma ECU que reinicie
      // apanha o estado certo sozinha.
      if (rx_msg.data_length_code >= 7) setStopDisplay(rx_msg.data[6] == 1);

      // ----- CONTROLO DE RELES DESATIVADO -----
      // digitalWrite(IN1_PIN, contactor_on ? LOW : HIGH);
      // digitalWrite(IN2_PIN, keyswitch_on ? LOW : HIGH);

#if DEBUG_TCMS_CMD
      Serial.printf("[TCMS] DC=%d KEY=%d\n", contactor_on, keyswitch_on);
#else
      (void)contactor_on;
      (void)keyswitch_on;
#endif
    }
  }

  // Buffer serie a encher: o matrix.show() ou algo esta a atrasar o loop
  int pending = veSerial.available();
  if (pending > 800) {
    static unsigned long lastFull = 0;
    if (millis() - lastFull > 10000) {
      lastFull = millis();
      ecuLog(EL_UART_FULL, LVL_WARN, (uint16_t)pending);
    }
  }
  while (veSerial.available()) {
    processChar((char)veSerial.read());
  }

  updateMatrix();
  checkEcuHealth();
}

// ============================================================
// VIGILANCIA DA ECU
// ============================================================
void checkEcuHealth() {
  // Anuncio de arranque, depois de 3 s: a esta altura o TCMS ja ouve
  if (!bootReported && millis() > 3000) {
    bootReported = true;
    ecuLog(EL_BOOT_OK, LVL_OK);
  }

  // SmartShunt calado
  if (!veSilentReported && millis() - lastGoodBlock > VE_SILENT_MS) {
    veSilentReported = true;
    ecuLog(EL_VE_SILENT, LVL_ERR, (uint16_t)((millis() - lastGoodBlock) / 1000));
  }

  // O shunt fala, mas nunca manda SOC: foi exatamente o bug do parser antigo
  if (!socWarned && !socEverSeen && haveVoltage && millis() > 20000) {
    socWarned = true;
    ecuLog(EL_VE_NO_SOC, LVL_WARN);
  }

  // Resumo periodico dos contadores, so se houver algo a dizer
  if (millis() - lastReport >= ECU_REPORT_MS) {
    lastReport = millis();
    if (checksumFail > 0) {
      ecuLog(EL_VE_CHECKSUM, LVL_WARN, (uint16_t)(checksumFail > 65535 ? 65535 : checksumFail));
      checksumFail = 0;
    }
    if (canTxFail > 0) {
      ecuLog(EL_CAN_TXFAIL, LVL_ERR, (uint16_t)(canTxFail > 65535 ? 65535 : canTxFail));
      canTxFail = 0;
    }
  }
}
