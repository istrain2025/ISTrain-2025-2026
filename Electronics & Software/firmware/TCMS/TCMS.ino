#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "driver/twai.h"

// ============================================================
// ESTRUTURAS E TIPOS (Definidos no topo para o compilador)
// ============================================================
#define MATRIX_TEXT_MAX     32      // TEM de ser igual no COMANDO e na ECU
#define LOG_TEXT_MAX        72
#define NUM_MCU             2

typedef enum { 
  MSG_PING = 0, 
  MSG_PONG = 1, 
  MSG_TELEMETRY = 2, 
  MSG_CONTROL = 3,
  MSG_LOG = 4 
} MsgType;

typedef struct __attribute__((packed)) {
  float    busVoltage;
  float    busCurrent;
  float    phaseCurrent;
  int16_t  speedActual;
  int16_t  ctrlTemp;
  int16_t  motorTemp;
  uint32_t errRaw;       // 32 bits para guardar todos os erros do motor
  bool     running;
  bool     speedMode;
  bool     online;       // handshake feito e telemetria a chegar
} mcu_telemetry_t;

typedef struct __attribute__((packed)) {
  MsgType type;
  char    text[16];
  char    command;
  int16_t targetRpm;
  float   targetAmps;
  uint8_t relayId;      // 0=Luzes, 1=DC Contactor, 2=Key Switch, 3=Buzina
  bool    relayState;   // estado pedido para esse relé

  // Telemetria de cada controlador EZkontrol
  mcu_telemetry_t mcu[NUM_MCU];

  uint8_t  relays;       // bitmask: bit0=luzes bit1=dc bit2=key bit3=buzina
  bool     motorEnabled; // true só com DC Contactor E Key Switch ON

  // Bateria (ECU / Smart Shunt, recebida por CAN)
  float    battVoltage;
  float    battCurrent;
  float    battPower;
  float    battSoc;      // já em % (escalado no TCMS)
  bool     battValid;    // true assim que chegar a 1ª trama da ECU

  // Texto para a matriz de LEDs (comando 'M')
  char     matrixText[MATRIX_TEXT_MAX + 1];

  // Uma linha de diagnostico (so preenchida quando type == MSG_LOG)
  uint8_t  logSrc;
  uint8_t  logLevel;
  char     logText[LOG_TEXT_MAX + 1];
} espnow_message_t;

enum ProtocolState { STATE_WAIT_HANDSHAKE, STATE_RUNNING };

struct Controller {
  const char* name;
  uint32_t canTxId;
  ProtocolState state;
  uint8_t lifeSignal;
  unsigned long lastControlSend;
  unsigned long lastRxTime;
  int targetRpm;
  float targetAmps;
};

// Protótipos explícitos das funções para evitar falhas de auto-geração do Arduino
bool canSend(twai_message_t &msg, const char *what);
void sendHandshakeResponse(Controller &ctrl);
void sendVcuCommand(Controller &ctrl);
void applyControlCommand(const espnow_message_t &msg);

// ============================================================
// CONFIGURAÇÃO GERAL
// ============================================================
#define WIFI_CHANNEL 1

#define CAN_TX_PIN GPIO_NUM_4
#define CAN_RX_PIN GPIO_NUM_5

// ----- RELÉS -----
#define LIGHTS_PIN         7    // Luzes
#define HORN_PIN           15    // Buzina (impulso momentaneo)
#define DC_CONTACTOR_PIN  17    // DC Contactor  (MESTRE: sem isto, nada funciona)
#define KEY_SWITCH_PIN     8    // Key Switch

#define RELAY_ACTIVE_HIGH 1

#define RELAY_LIGHTS 0
#define RELAY_DC     1
#define RELAY_KEY    2
#define RELAY_HORN   3
#define RELAY_COUNT  4

#define HORN_PULSE_MS 250      // duracao do toque da buzina (0,5 s)

#define LIGHTS_AUTO      1
// 0 = as luzes mudam logo que as RPM reais saiam do zero:
//     positivas -> ON, negativas -> OFF, exatamente 0 -> mantem o estado.
// Com 10 havia um problema com os passos de 10 RPM: a 10 RPM de alvo, o
// motor anda a ~10 RPM e a condicao "> 10" nunca se cumpria.
#define LIGHTS_RPM_ON    0
// A variável permanece definida, mas o seu impacto no DC Contactor foi neutralizado
#define LIGHTS_START_FORWARD 1 

#define CONTROL_INTERVAL_MS 50
#define RX_TIMEOUT_MS       500
#define TELEMETRY_INTERVAL_MS 200

// IDs CAN de comando VCU (29-bit Extended)
#define CAN_ID_VCU_CMD_1 0x0C01EFD0
#define CAN_ID_VCU_CMD_2 0x0C01F0D0

// IDs CAN de telemetria do Motor (29-bit: ...DD SS, DD=destino, SS=origem)
#define CAN_ID_MCU1_MSG1 0x1801D0EF
#define CAN_ID_MCU1_MSG2 0x1802D0EF
#define CAN_ID_MCU2_MSG1 0x1801D0F0
#define CAN_ID_MCU2_MSG2 0x1802D0F0

static inline int mcuIndexMsg1(uint32_t id) {
  if (id == CAN_ID_MCU1_MSG1) return 0;
  if (id == CAN_ID_MCU2_MSG1) return 1;
  return -1;
}
static inline int mcuIndexMsg2(uint32_t id) {
  if (id == CAN_ID_MCU1_MSG2) return 0;
  if (id == CAN_ID_MCU2_MSG2) return 1;
  return -1;
}

#define CAN_ID_BATT_MSG1  0x0C04EFD0
#define CAN_ID_BATT_MSG2  0x0C05EFD0

#define CAN_ID_TCMS_CMD   0x0C03EFD0
#define ECU_CMD_INTERVAL_MS 200     

#define CAN_ID_MATRIX_TXT   0x0C06EFD0
#define MATRIX_CHUNK        5
#define MATRIX_REPEAT_MS    3000    

#define ECU_SOC_SCALE 0.1f

#define CAN_ID_ECU_LOG 0x0C07EFD0

#define ERROR_COUNT 22
const char* const ERROR_NAMES[ERROR_COUNT] = {
  "Overcurrent", "Overload", "Overvoltage", "Undervoltage",
  "Controller Overheat", "Motor Overheat", "Motor Stalled", "Motor Out of phase",
  "Motor Sensor", "Motor AUX Sensor", "Encoder Misaligned", "Anti-Runaway Engaged",
  "Main Accelerator", "AUX Accelerator", "Pre-charge", "DC Contactor",
  "Power valve", "Current Sensor", "Auto-tune", "RS485", "CAN", "Software"
};

// ============================================================
// DIAGNÓSTICO
// ============================================================
#define LOG_QUEUE_LEN 32

#define LVL_INFO  0
#define LVL_OK    1
#define LVL_WARN  2
#define LVL_ERR   3

#define SRC_TCMS  0
#define SRC_ECU   1
#define SRC_CMD   2

// Variáveis Globais
espnow_message_t globalTelemetry = {};

typedef struct { uint8_t src; uint8_t level; char text[LOG_TEXT_MAX + 1]; } log_entry_t;
QueueHandle_t logQueue = NULL;
volatile uint16_t logDropped = 0;

void vcuLog(uint8_t src, uint8_t level, const char *fmt, ...) {
  log_entry_t e;
  e.src = src;
  e.level = level;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(e.text, sizeof(e.text), fmt, ap);
  va_end(ap);

  Serial.println(e.text);                       
  if (!logQueue || xQueueSend(logQueue, &e, 0) != pdTRUE) logDropped++;
}

#define LOGI(...) vcuLog(SRC_TCMS, LVL_INFO, __VA_ARGS__)
#define LOGOK(...) vcuLog(SRC_TCMS, LVL_OK,   __VA_ARGS__)
#define LOGW(...) vcuLog(SRC_TCMS, LVL_WARN, __VA_ARGS__)
#define LOGE(...) vcuLog(SRC_TCMS, LVL_ERR,  __VA_ARGS__)

uint8_t senderMac[6] = {0};
volatile bool senderKnown = false;
bool eStopActive = true;                 
// So fica true quando alguem CARREGA no STOP (comando 'P'). Ao contrario do
// eStopActive, nao liga no arranque nem ao desligar o DC Contactor.
// A ECU usa isto para mostrar "PARAR" na matriz.
bool stopPressed = false;
bool relayState[RELAY_COUNT] = { false, false, false, false };
unsigned long hornOffAt = 0;             
unsigned long lastBattRx = 0;            
unsigned long lastEcuCmdSend = 0;        
volatile bool ecuCmdDirty = true;        
char matrixText[MATRIX_TEXT_MAX + 1] = "ISTrain";   
uint8_t matrixSeq = 0;                   
bool matrixDirty = true;                 
unsigned long lastMatrixSend = 0;

uint32_t canTxFailCount = 0;        
uint32_t canBusErrCount = 0;        
uint32_t espNowFailCount = 0;       
uint32_t battRejectCount = 0;       
unsigned long lastHealthReport = 0; 
unsigned long lastCanErrReport = 0;
bool battWasValid = false;          
int  socLevelReported = -1;         
uint32_t lastErrRaw[NUM_MCU] = {0, 0};
int  tempLevelCtrl[NUM_MCU] = {0, 0};
int  tempLevelMotor[NUM_MCU] = {0, 0};
bool wasRunning[NUM_MCU] = {false, false};
#define HEALTH_REPORT_MS 15000
#define BATT_TIMEOUT_MS 3000
SemaphoreHandle_t systemMutex;

Controller controllers[2] = {
  { "CTRL_1", CAN_ID_VCU_CMD_1, STATE_WAIT_HANDSHAKE, 0, 0, 0, 0, 30.0f },
  { "CTRL_2", CAN_ID_VCU_CMD_2, STATE_WAIT_HANDSHAKE, 0, 0, 0, 0, 30.0f }
};

// ============================================================
// RELÉS E INTERLOCK DE SEGURANÇA
// ============================================================
int relayPin(uint8_t id) {
  switch (id) {
    case RELAY_LIGHTS: return LIGHTS_PIN;
    case RELAY_DC:     return DC_CONTACTOR_PIN;
    case RELAY_KEY:    return KEY_SWITCH_PIN;
    case RELAY_HORN:   return HORN_PIN;
  }
  return -1;
}

const char* relayName(uint8_t id) {
  switch (id) {
    case RELAY_LIGHTS: return "LUZES";
    case RELAY_DC:     return "DC CONTACTOR";
    case RELAY_KEY:    return "KEY SWITCH";
    case RELAY_HORN:   return "BUZINA";
  }
  return "?";
}

void writeRelay(uint8_t id, bool on) {
  int pin = relayPin(id);
  if (pin < 0) return;
#if RELAY_ACTIVE_HIGH
  digitalWrite(pin, on ? LOW : HIGH);
#else
  digitalWrite(pin, on ? HIGH : LOW);
#endif
  relayState[id] = on;
  ecuCmdDirty = true;   
  LOGI("[POTENCIA] %s -> %s (GPIO %d)", relayName(id), on ? "ON" : "OFF", pin);
}

bool motorEnabled() {
  return relayState[RELAY_DC] && relayState[RELAY_KEY];
}

void stopMotorTargets() {
  controllers[0].targetRpm = 0;
  controllers[1].targetRpm = 0;
}

void setRelayRequest(uint8_t id, bool want) {
  if (id >= RELAY_COUNT) return;

  if (id == RELAY_DC) {
    if (!want) {
      writeRelay(RELAY_KEY, false);
      writeRelay(RELAY_LIGHTS, false);
      writeRelay(RELAY_HORN, false);
      hornOffAt = 0;
      writeRelay(RELAY_DC, false);
      stopMotorTargets();
      eStopActive = true;
      LOGE("[POTENCIA] DC Contactor OFF: sistema desenergizado, tudo desarmado");
      return;
    }
    bool jaEstavaLigado = relayState[RELAY_DC];
    writeRelay(RELAY_DC, true);

    if (!jaEstavaLigado) {
      LOGI("[POTENCIA] DC Contactor ON (luzes prontas para atuar com a marcha)");
    }
    return;
  }

  if (want && !relayState[RELAY_DC]) {
    LOGW("[POTENCIA] Pedido recusado: %s exige o DC Contactor ON", relayName(id));
    return;
  }

  if (id == RELAY_HORN) {
    if (want) {
      writeRelay(RELAY_HORN, true);
      hornOffAt = millis() + HORN_PULSE_MS;
    } else {
      writeRelay(RELAY_HORN, false);
      hornOffAt = 0;
    }
    return;
  }

  if (id == RELAY_KEY && !want) {
    writeRelay(RELAY_KEY, false);
    stopMotorTargets();
    LOGW("[POTENCIA] Key Switch OFF: motor desarmado, alvos a zero");
    return;
  }

  writeRelay(id, want);
}

void updateAutoLights() {
#if LIGHTS_AUTO
  if (!relayState[RELAY_DC]) return;

  int16_t rpm;
  if      (controllers[0].state == STATE_RUNNING) rpm = globalTelemetry.mcu[0].speedActual;
  else if (controllers[1].state == STATE_RUNNING) rpm = globalTelemetry.mcu[1].speedActual;
  else return;

  if (rpm > LIGHTS_RPM_ON) {
    if (relayState[RELAY_LIGHTS]) {
      LOGI("[POTENCIA] Luzes BRANCAS automaticamente (%d RPM, marcha a frente)", rpm);
      writeRelay(RELAY_LIGHTS, false);
    }
  } else if (rpm < -LIGHTS_RPM_ON) {
    if (!relayState[RELAY_LIGHTS]) {
      LOGI("[POTENCIA] Luzes VERMELHAS automaticamente (%d RPM, marcha atras)", rpm);
      writeRelay(RELAY_LIGHTS, true);
    }
  }
#endif
}

// ============================================================
// PARSING DA TELEMETRIA DO CONTROLADOR
// ============================================================
void parseControllerTelemetry(const twai_message_t& rx_msg) {
  if (rx_msg.data_length_code < 8) return;
  uint32_t id = rx_msg.identifier;

  xSemaphoreTake(systemMutex, portMAX_DELAY);
  
  int i1 = mcuIndexMsg1(id);
  int i2 = mcuIndexMsg2(id);

  if (i1 >= 0) {
    uint16_t rawV    = rx_msg.data[0] | (rx_msg.data[1] << 8);
    uint16_t rawBusI = rx_msg.data[2] | (rx_msg.data[3] << 8);
    uint16_t rawPhaI = rx_msg.data[4] | (rx_msg.data[5] << 8);
    uint16_t rawSpd  = rx_msg.data[6] | (rx_msg.data[7] << 8);

    globalTelemetry.mcu[i1].busVoltage   = rawV * 0.1f;
    globalTelemetry.mcu[i1].busCurrent   = rawBusI * 0.1f - 3200.0f;
    globalTelemetry.mcu[i1].phaseCurrent = rawPhaI * 0.1f - 3200.0f;
    globalTelemetry.mcu[i1].speedActual  = (int32_t)rawSpd - 32000;
  }
  else if (i2 >= 0) {
    globalTelemetry.mcu[i2].ctrlTemp  = (int)rx_msg.data[0] - 40;
    globalTelemetry.mcu[i2].motorTemp = (int)rx_msg.data[1] - 40;
    globalTelemetry.mcu[i2].running   = rx_msg.data[2] & 0x01;
    globalTelemetry.mcu[i2].speedMode = rx_msg.data[2] & 0x02;

    globalTelemetry.mcu[i2].errRaw    = ((uint32_t)rx_msg.data[5] << 16) |
                                        ((uint32_t)rx_msg.data[4] << 8)  |
                                         (uint32_t)rx_msg.data[3];
  }
  else if (id == CAN_ID_BATT_MSG1) {
    float v, a;
    memcpy(&v, &rx_msg.data[0], 4);
    memcpy(&a, &rx_msg.data[4], 4);
    if (isfinite(v) && isfinite(a)) {
      globalTelemetry.battVoltage = v;
      globalTelemetry.battCurrent = a;
      globalTelemetry.battValid   = true;
      lastBattRx = millis();
    } else {
      battRejectCount++;
    }
  }
  else if (id == CAN_ID_BATT_MSG2) {
    float p, soc;
    memcpy(&p, &rx_msg.data[0], 4);
    memcpy(&soc, &rx_msg.data[4], 4);
    if (isfinite(p) && isfinite(soc)) {
      globalTelemetry.battPower = p;
      globalTelemetry.battSoc   = constrain(soc * ECU_SOC_SCALE, 0.0f, 100.0f);
      globalTelemetry.battValid = true;
      lastBattRx = millis();

      int lvl = (globalTelemetry.battSoc < 15.0f) ? 2 : (globalTelemetry.battSoc < 30.0f ? 1 : 0);
      if (lvl != socLevelReported) {
        socLevelReported = lvl;
        if (lvl == 2)      LOGE("[BATERIA] SOC critico: %.1f %%", globalTelemetry.battSoc);
        else if (lvl == 1) LOGW("[BATERIA] SOC baixo: %.1f %%", globalTelemetry.battSoc);
        else               LOGOK("[BATERIA] SOC acima de 30 %% (%.1f %%)", globalTelemetry.battSoc);
      }
    } else {
      battRejectCount++;
    }
  }
  xSemaphoreGive(systemMutex);
}

// ============================================================
// FUNÇÕES AUXILIARES CAN
// ============================================================
uint16_t calculateSpeedValue(int targetRpm) {
  int32_t val = (int32_t)targetRpm + 32000;
  return (uint16_t)constrain(val, 0, 65535);
}

uint16_t calculateCurrentValue(float targetAmps) {
  int32_t val = (int32_t)((targetAmps + 3200.0f) * 10.0f);
  return (uint16_t)constrain(val, 0, 65535);
}

void sendTcmsStateToEcu() {
  twai_message_t tx_msg = {};
  tx_msg.identifier = CAN_ID_TCMS_CMD;
  tx_msg.extd = 1;
  tx_msg.rtr  = 0;
  tx_msg.data_length_code = 8;

  tx_msg.data[0] = relayState[RELAY_DC]     ? 1 : 0;
  tx_msg.data[1] = relayState[RELAY_KEY]    ? 1 : 0;
  tx_msg.data[2] = relayState[RELAY_LIGHTS] ? 1 : 0;
  tx_msg.data[3] = relayState[RELAY_HORN]   ? 1 : 0;
  tx_msg.data[4] = eStopActive              ? 1 : 0;
  tx_msg.data[5] = motorEnabled()           ? 1 : 0;
  tx_msg.data[6] = stopPressed              ? 1 : 0;   // matriz mostra PARAR
  tx_msg.data[7] = 0;

  canSend(tx_msg, "estado para a ECU");
  lastEcuCmdSend = millis();
  ecuCmdDirty = false;
}

void sendMatrixTextToEcu() {
  int len = strlen(matrixText);
  uint8_t total = (uint8_t)((len + MATRIX_CHUNK - 1) / MATRIX_CHUNK);
  if (total == 0) total = 1;            

  for (uint8_t idx = 0; idx < total; idx++) {
    twai_message_t tx_msg = {};
    tx_msg.identifier = CAN_ID_MATRIX_TXT;
    tx_msg.extd = 1;
    tx_msg.rtr  = 0;
    tx_msg.data_length_code = 8;
    tx_msg.data[0] = matrixSeq;
    tx_msg.data[1] = idx;
    tx_msg.data[2] = total;
    for (int i = 0; i < MATRIX_CHUNK; i++) {
      int p = idx * MATRIX_CHUNK + i;
      tx_msg.data[3 + i] = (p < len) ? (uint8_t)matrixText[p] : 0;
    }
    canSend(tx_msg, "texto da matriz");
  }

  lastMatrixSend = millis();
  matrixDirty = false;
}

void handleEcuLogFrame(const twai_message_t &rx) {
  if (rx.data_length_code < 2) return;
  uint8_t code  = rx.data[0];
  uint8_t level = rx.data[1];
  uint16_t p1 = (rx.data_length_code >= 4) ? (rx.data[2] | (rx.data[3] << 8)) : 0;
  uint16_t p2 = (rx.data_length_code >= 6) ? (rx.data[4] | (rx.data[5] << 8)) : 0;

  switch (code) {
    case 1:  vcuLog(SRC_ECU, level, "[SISTEMA] ECU arrancou. CAN e matriz prontos."); break;
    case 2:  vcuLog(SRC_ECU, level, "[CAN] ECU: falha ao iniciar o driver TWAI."); break;
    case 3:  vcuLog(SRC_ECU, level, "[DADOS] ECU: sem blocos validos do SmartShunt ha %u s. Ve o cabo VE.Direct.", p1); break;
    case 4:  vcuLog(SRC_ECU, level, "[DADOS] ECU: SmartShunt a responder outra vez."); break;
    case 5:  vcuLog(SRC_ECU, level, "[DADOS] ECU: %u blocos VE.Direct com checksum invalido. Ruido na linha serie.", p1); break;
    case 6:  vcuLog(SRC_ECU, level, "[DADOS] ECU: campo SOC nunca recebido do shunt (V/I/P chegam bem)."); break;
    case 7:  vcuLog(SRC_ECU, level, "[CAN] ECU: %u transmissoes falhadas. O TCMS esta a ouvir?", p1); break;
    case 8:  vcuLog(SRC_ECU, level, "[MATRIZ] ECU: texto novo aplicado, %u caracteres.", p1); break;
    case 9:  vcuLog(SRC_ECU, level, "[MATRIZ] ECU: mensagem %u incompleta, pedacos em falta (mascara 0x%02X).", p1, p2); break;
    case 10: vcuLog(SRC_ECU, level, "[BUFFER] ECU: buffer serie quase cheio (%u bytes). Blocos podem perder-se.", p1); break;
    case 11: vcuLog(SRC_ECU, level, "[DADOS] ECU: bloco recebido mas sem nenhum campo util."); break;
    case 12: vcuLog(SRC_ECU, level, p1 ? "[MATRIZ] ECU: a mostrar PARAR a vermelho."
                                       : "[MATRIZ] ECU: PARAR retirado, texto normal de volta."); break;
    default: vcuLog(SRC_ECU, level, "[ECU] codigo desconhecido %u (%u, %u)", code, p1, p2); break;
  }
}

void watchControllerData(int c) {
  mcu_telemetry_t &m = globalTelemetry.mcu[c];
  const char *name = controllers[c].name;

  if (m.errRaw != lastErrRaw[c]) {
    uint32_t apareceram = m.errRaw & ~lastErrRaw[c];
    uint32_t sairam     = lastErrRaw[c] & ~m.errRaw;
    for (int b = 0; b < ERROR_COUNT; b++) {
      if (apareceram & (1UL << b)) LOGE("[MOTOR] %s ERRO: %s", name, ERROR_NAMES[b]);
      if (sairam     & (1UL << b)) LOGOK("[MOTOR] %s erro resolvido: %s", name, ERROR_NAMES[b]);
    }
    lastErrRaw[c] = m.errRaw;
  }

  if (m.running != wasRunning[c]) {
    wasRunning[c] = m.running;
    LOGI("[MOTOR] %s: %s", name, m.running ? "RUNNING" : "HALTED");
  }

  int lvl = (m.ctrlTemp > 80) ? 2 : (m.ctrlTemp > 60 ? 1 : 0);
  if (lvl != tempLevelCtrl[c]) {
    tempLevelCtrl[c] = lvl;
    if (lvl == 2)      LOGE("[TEMPERATURA] %s controlador a %d C: acima do limite.", name, m.ctrlTemp);
    else if (lvl == 1) LOGW("[TEMPERATURA] %s controlador a %d C: a aquecer.", name, m.ctrlTemp);
    else               LOGOK("[TEMPERATURA] %s controlador normalizou (%d C).", name, m.ctrlTemp);
  }

  lvl = (m.motorTemp > 100) ? 2 : (m.motorTemp > 80 ? 1 : 0);
  if (lvl != tempLevelMotor[c]) {
    tempLevelMotor[c] = lvl;
    if (lvl == 2)      LOGE("[TEMPERATURA] %s motor a %d C: acima do limite.", name, m.motorTemp);
    else if (lvl == 1) LOGW("[TEMPERATURA] %s motor a %d C: a aquecer.", name, m.motorTemp);
    else               LOGOK("[TEMPERATURA] %s motor normalizou (%d C).", name, m.motorTemp);
  }
}

void checkCanAlerts() {
  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) != ESP_OK) return;

  if (alerts & TWAI_ALERT_BUS_OFF) {
    LOGE("[CAN] BUS-OFF: demasiados erros, o no saiu do barramento. A recuperar...");
    LOGE("[CAN] Verifica terminacao (120 ohm so nas 2 pontas), massa comum e bitrate.");
    twai_initiate_recovery();
  }
  if (alerts & TWAI_ALERT_BUS_RECOVERED) {
    LOGOK("[CAN] Barramento recuperado. A reiniciar o driver.");
    twai_start();          
  }
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
    LOGW("[BUFFER] Fila de rececao CAN cheia: tramas perdidas. O canTask esta atrasado.");
  }
  if (alerts & TWAI_ALERT_TX_FAILED) {
    LOGE("[CAN] Transmissao falhada: ninguem confirmou a trama (ACK). Ha alguem no barramento?");
  }
  if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
    LOGW("[CAN] Contadores de erro acima do limiar. O barramento esta a degradar-se.");
  }
  if (alerts & TWAI_ALERT_ERR_PASS) {
    LOGE("[CAN] Estado ERROR-PASSIVE: este no ja quase nao e ouvido pelos outros.");
  }
  if (alerts & TWAI_ALERT_BUS_ERROR) {
    canBusErrCount++;
    if (canBusErrCount % 50 == 0 && millis() - lastCanErrReport > 5000) {
      lastCanErrReport = millis();
      LOGW("[CAN] %u erros de bit/forma acumulados. Ruido ou cablagem.", canBusErrCount);
    }
  }
}

void reportHealth() {
  if (millis() - lastHealthReport < HEALTH_REPORT_MS) return;
  lastHealthReport = millis();

  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK) {
    if (st.tx_error_counter > 0 || st.rx_error_counter > 0 ||
        st.rx_missed_count > 0 || st.arb_lost_count > 0) {
      LOGW("[CAN] TEC=%u REC=%u perdidas=%u arbitragem=%u",
           st.tx_error_counter, st.rx_error_counter,
           st.rx_missed_count, st.arb_lost_count);
    }
  }
  if (canTxFailCount > 0) {
    LOGW("[CAN] %u transmissoes falhadas desde o arranque.", canTxFailCount);
  }
  if (espNowFailCount > 0) {
    LOGW("[ESP-NOW] %u envios falhados. O COMANDO pode estar longe ou desligado.", espNowFailCount);
  }
  if (battRejectCount > 0) {
    LOGW("[DADOS] %u valores invalidos da ECU descartados (NaN/infinito).", battRejectCount);
  }
  if (logDropped > 0) {
    LOGW("[BUFFER] %u linhas de log descartadas: a fila encheu.", logDropped);
    logDropped = 0;
  }
}

bool canSend(twai_message_t &msg, const char *what) {
  esp_err_t r = twai_transmit(&msg, pdMS_TO_TICKS(10));
  if (r != ESP_OK) {
    canTxFailCount++;
    if (canTxFailCount <= 3 || canTxFailCount % 100 == 0) {
      LOGE("[CAN] Falha ao enviar %s (ID 0x%08X, erro %d)",
           what, (unsigned)msg.identifier, (int)r);
    }
    return false;
  }
  return true;
}

void sendHandshakeResponse(Controller &ctrl) {
  twai_message_t tx_msg = {};
  tx_msg.identifier = ctrl.canTxId;
  tx_msg.extd = 1;
  tx_msg.data_length_code = 8;
  memset(tx_msg.data, 0xAA, 8);
  canSend(tx_msg, "resposta de handshake");
}

void sendVcuCommand(Controller &ctrl) {
  bool allowed = !eStopActive && motorEnabled();

  float currentToSet = 0.0f;
  int   rpmToSet     = 0;
  if (allowed) {
    currentToSet = (ctrl.targetRpm < 0) ? -abs(ctrl.targetAmps) : abs(ctrl.targetAmps);
    rpmToSet     = ctrl.targetRpm;
  }

  twai_message_t tx_msg = {};
  tx_msg.identifier = ctrl.canTxId;
  tx_msg.extd = 1;
  tx_msg.data_length_code = 8;

  uint16_t currentVal = calculateCurrentValue(currentToSet);
  uint16_t speedVal   = calculateSpeedValue(rpmToSet);

  tx_msg.data[0] = currentVal & 0xFF;
  tx_msg.data[1] = (currentVal >> 8) & 0xFF;
  tx_msg.data[2] = speedVal & 0xFF;
  tx_msg.data[3] = (speedVal >> 8) & 0xFF;
  tx_msg.data[4] = allowed ? 0x01 : 0x00;   
  tx_msg.data[5] = 0x00;
  tx_msg.data[6] = 0x00;
  tx_msg.data[7] = ctrl.lifeSignal++;

  canSend(tx_msg, "comando de motor");
}

// ============================================================
// GESTÃO DE COMANDOS ESP-NOW
// ============================================================
void applyControlCommand(const espnow_message_t &msg) {
  xSemaphoreTake(systemMutex, portMAX_DELAY);
  
  if (msg.command == 'P') { 
    eStopActive = true;
    stopPressed = true;
    ecuCmdDirty = true;          // a matriz mostra PARAR ja, sem esperar 200 ms
    stopMotorTargets();
    LOGE("[SEGURANCA] E-STOP ativado: alvos a zero e enable a 0");
  } 
  else if (msg.command == 'R') { 
    if (motorEnabled()) {
      eStopActive = false;
      stopPressed = false;
      ecuCmdDirty = true;        // a matriz volta ao texto normal
      LOGOK("[SEGURANCA] E-STOP libertado: controlo retomado");
    } else {
      LOGW("[SEGURANCA] Resume recusado: faltam DC Contactor e/ou Key Switch");
    }
  } 
  else if (msg.command == 'T') { 
    if (!eStopActive && motorEnabled()) {
      controllers[0].targetRpm = msg.targetRpm;
      controllers[1].targetRpm = msg.targetRpm;
    } else {
      stopMotorTargets();
    }
  } 
  else if (msg.command == 'A') { 
    controllers[0].targetAmps = msg.targetAmps;
    controllers[1].targetAmps = msg.targetAmps;
  }
  else if (msg.command == 'L') { 
    setRelayRequest(msg.relayId, msg.relayState);
  }
  else if (msg.command == 'M') { 
    strncpy(matrixText, msg.matrixText, MATRIX_TEXT_MAX);
    matrixText[MATRIX_TEXT_MAX] = '\0';
    matrixSeq++;                 
    matrixDirty = true;
    LOGI("[MATRIZ] texto novo: \"%s\" (%d caracteres)", matrixText, (int)strlen(matrixText));
  }

  xSemaphoreGive(systemMutex);
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(espnow_message_t)) {
    static unsigned long lastWarn = 0;
    if (millis() - lastWarn > 5000) {
      lastWarn = millis();
      LOGE("[ESP-NOW] Pacote de %d bytes rejeitado (esperado %d). As structs do TCMS e do COMANDO estao diferentes: grava os dois.",
           len, (int)sizeof(espnow_message_t));
    }
    return;
  }

  espnow_message_t msg;
  memcpy(&msg, data, sizeof(msg));

  if (!senderKnown || memcmp(senderMac, info->src_addr, 6) != 0) {
    memcpy(senderMac, info->src_addr, 6);
    if (!esp_now_is_peer_exist(senderMac)) {
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, senderMac, 6);
      peerInfo.channel = WIFI_CHANNEL;
      esp_now_add_peer(&peerInfo);
    }
    senderKnown = true;
    LOGOK("[ESP-NOW] Posto de comando emparelhado: %02X:%02X:%02X:%02X:%02X:%02X",
          senderMac[0], senderMac[1], senderMac[2], senderMac[3], senderMac[4], senderMac[5]);
  }

  if (msg.type == MSG_PING) {
    espnow_message_t reply = {};
    reply.type = MSG_PONG;
    strncpy(reply.text, "Adeus", sizeof(reply.text));
    esp_now_send(senderMac, (uint8_t *)&reply, sizeof(reply));
  } 
  else if (msg.type == MSG_CONTROL) {
    applyControlCommand(msg);
  }
}

// ============================================================
// TASK 1 (CORE 1): LÓGICA CAN
// ============================================================
void canTask(void *pvParameters) {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  g_config.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                            TWAI_ALERT_ERR_PASS | TWAI_ALERT_ABOVE_ERR_WARN |
                            TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_FAILED |
                            TWAI_ALERT_BUS_ERROR;

  esp_err_t instErr = twai_driver_install(&g_config, &t_config, &f_config);
  if (instErr != ESP_OK) {
    LOGE("[CAN] FALHA ao instalar o driver TWAI (erro %d). Sem CAN nao ha motores.", (int)instErr);
  } else {
    esp_err_t startErr = twai_start();
    if (startErr != ESP_OK) {
      LOGE("[CAN] FALHA ao arrancar o driver TWAI (erro %d).", (int)startErr);
    } else {
      LOGOK("[CAN] Driver TWAI iniciado a 250 kbit/s (TX GPIO%d, RX GPIO%d)", CAN_TX_PIN, CAN_RX_PIN);
    }
  }

  twai_message_t rx_msg;
  for (;;) {
    checkCanAlerts();

    while (twai_receive(&rx_msg, 0) == ESP_OK) {
      int telIdx = mcuIndexMsg1(rx_msg.identifier);
      if (telIdx < 0) telIdx = mcuIndexMsg2(rx_msg.identifier);

      int hsIdx = -1;
      if (rx_msg.data_length_code == 8) {
        int cand = mcuIndexMsg1(rx_msg.identifier);
        if (cand >= 0) {
          bool all55 = true;
          for (int i = 0; i < 8; i++) {
            if (rx_msg.data[i] != 0x55) { all55 = false; break; }
          }
          if (all55) hsIdx = cand;
        }
      }

      if (rx_msg.identifier == CAN_ID_ECU_LOG) {
        handleEcuLogFrame(rx_msg);
      }
      else if (hsIdx < 0) {
        parseControllerTelemetry(rx_msg);
      }

      xSemaphoreTake(systemMutex, portMAX_DELAY);
      if (telIdx >= 0) controllers[telIdx].lastRxTime = millis();

      if (hsIdx >= 0 && controllers[hsIdx].state == STATE_WAIT_HANDSHAKE) {
        sendHandshakeResponse(controllers[hsIdx]);
        controllers[hsIdx].state = STATE_RUNNING;
        controllers[hsIdx].lastControlSend = millis();
        LOGOK("[MOTOR] %s online: handshake concluido", controllers[hsIdx].name);
      }
      xSemaphoreGive(systemMutex);
    }

    xSemaphoreTake(systemMutex, portMAX_DELAY);

    if (hornOffAt != 0 && (long)(millis() - hornOffAt) >= 0) {
      writeRelay(RELAY_HORN, false);
      hornOffAt = 0;
    }

    updateAutoLights();

    for (int i = 0; i < NUM_MCU; i++) {
      if (controllers[i].state == STATE_RUNNING) watchControllerData(i);
    }
    reportHealth();

    if (ecuCmdDirty || (millis() - lastEcuCmdSend >= ECU_CMD_INTERVAL_MS)) {
      sendTcmsStateToEcu();
    }

    if (matrixDirty || (millis() - lastMatrixSend >= MATRIX_REPEAT_MS)) {
      sendMatrixTextToEcu();
    }

    for (int i = 0; i < 2; i++) {
      if (controllers[i].state == STATE_RUNNING) {
        if (millis() - controllers[i].lastRxTime > RX_TIMEOUT_MS) {
          controllers[i].state = STATE_WAIT_HANDSHAKE;
          LOGE("[MOTOR] %s offline: %d ms sem telemetria. Verifica cablagem CAN e alimentacao.", controllers[i].name, RX_TIMEOUT_MS);
          continue;
        }
        if (millis() - controllers[i].lastControlSend >= CONTROL_INTERVAL_MS) {
          controllers[i].lastControlSend = millis();
          sendVcuCommand(controllers[i]);
        }
      }
    }
    xSemaphoreGive(systemMutex);

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ============================================================
// TASK 2 (CORE 0): ESP-NOW (Telemetria)
// ============================================================
void espNowTask(void *pvParameters) {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onDataRecv);
    LOGOK("[ESP-NOW] Iniciado no canal %d. MAC deste TCMS: %s",
          WIFI_CHANNEL, WiFi.macAddress().c_str());
  } else {
    LOGE("[ESP-NOW] FALHA ao iniciar. O posto de comando nao vai conseguir falar com o TCMS.");
  }

  unsigned long lastTelemetry = 0;
  for (;;) {
    if (senderKnown && (millis() - lastTelemetry >= TELEMETRY_INTERVAL_MS)) {
      lastTelemetry = millis();

      xSemaphoreTake(systemMutex, portMAX_DELAY);
      globalTelemetry.type = MSG_TELEMETRY;
      globalTelemetry.relays = (relayState[RELAY_LIGHTS] ? 0x01 : 0) |
                               (relayState[RELAY_DC]     ? 0x02 : 0) |
                               (relayState[RELAY_KEY]    ? 0x04 : 0) |
                               (relayState[RELAY_HORN]   ? 0x08 : 0);
      globalTelemetry.motorEnabled = motorEnabled();

      for (int i = 0; i < NUM_MCU; i++) {
        globalTelemetry.mcu[i].online = (controllers[i].state == STATE_RUNNING);
      }
      if (lastBattRx == 0 || millis() - lastBattRx > BATT_TIMEOUT_MS) {
        globalTelemetry.battValid = false;   
        if (battWasValid) {
          battWasValid = false;
          LOGE("[BATERIA] Sem dados da ECU ha mais de %d ms. ECU desligada ou CAN cortado.",
               BATT_TIMEOUT_MS);
        }
      } else if (!battWasValid) {
        battWasValid = true;
        LOGOK("[BATERIA] ECU a responder: %.2f V, %.2f A, SOC %.1f %%",
              globalTelemetry.battVoltage, globalTelemetry.battCurrent, globalTelemetry.battSoc);
      }
      espnow_message_t msgToSend = globalTelemetry;
      xSemaphoreGive(systemMutex);

      if (esp_now_send(senderMac, (uint8_t *)&msgToSend, sizeof(msgToSend)) != ESP_OK) {
        espNowFailCount++;
      }
    }

    if (senderKnown) {
      log_entry_t e;
      for (int k = 0; k < 4 && xQueueReceive(logQueue, &e, 0) == pdTRUE; k++) {
        espnow_message_t lm = {};
        lm.type     = MSG_LOG;
        lm.logSrc   = e.src;
        lm.logLevel = e.level;
        strncpy(lm.logText, e.text, LOG_TEXT_MAX);
        if (esp_now_send(senderMac, (uint8_t *)&lm, sizeof(lm)) != ESP_OK) espNowFailCount++;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ============================================================
// SETUP & LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LIGHTS_PIN, OUTPUT);
  pinMode(HORN_PIN, OUTPUT);
  pinMode(DC_CONTACTOR_PIN, OUTPUT);
  pinMode(KEY_SWITCH_PIN, OUTPUT);
  writeRelay(RELAY_DC, false);
  writeRelay(RELAY_KEY, false);
  writeRelay(RELAY_LIGHTS, false);
  writeRelay(RELAY_HORN, false);

  systemMutex = xSemaphoreCreateMutex();
  logQueue    = xQueueCreate(LOG_QUEUE_LEN, sizeof(log_entry_t));

  if (!systemMutex) Serial.println("[SISTEMA] FALHA ao criar o mutex!");
  if (!logQueue)    Serial.println("[SISTEMA] FALHA ao criar a fila de log!");

  LOGOK("[SISTEMA] TCMS a arrancar. Reles todos desligados, E-STOP ativo.");
  LOGI("[SISTEMA] Tamanho da mensagem ESP-NOW: %d bytes (tem de bater certo com o COMANDO)",
       (int)sizeof(espnow_message_t));

  xTaskCreatePinnedToCore(espNowTask, "ESPNOW_Task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(canTask, "CAN_Task", 4096, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);
}