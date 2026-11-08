#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

// ============================================================
// CONFIGURAÇÃO
// ============================================================
uint8_t receiverMac[] = {0xDC, 0xB4, 0xD9, 0x17, 0x1B, 0xA4 };
#define WIFI_CHANNEL 1
#define HEARTBEAT_INTERVAL_MS 2000

#define RELAY_LIGHTS 0
#define RELAY_DC     1
#define RELAY_KEY    2
#define RELAY_HORN   3

// Texto da matriz de LEDs. TEM de ser igual no TCMS e na ECU.
#define MATRIX_TEXT_MAX 32

// ============================================================
// PROTOCOLO DE MENSAGENS
// ============================================================
typedef enum { MSG_PING = 0, MSG_PONG = 1, MSG_TELEMETRY = 2, MSG_CONTROL = 3,
               MSG_LOG = 4 } MsgType;

#define LOG_TEXT_MAX 72
#define LVL_INFO 0
#define LVL_OK   1
#define LVL_WARN 2
#define LVL_ERR  3
#define SRC_TCMS 0
#define SRC_ECU  1
#define SRC_CMD  2

// ATENÇÃO: esta struct tem de ser EXATAMENTE igual nos dois ESP32.
#define NUM_MCU 2

// Telemetria de UM controlador. Igual no TCMS e no COMANDO.
typedef struct __attribute__((packed)) {
  float    busVoltage;
  float    busCurrent;
  float    phaseCurrent;
  int16_t  speedActual;
  int16_t  ctrlTemp;
  int16_t  motorTemp;
  uint32_t errRaw;
  bool     running;
  bool     speedMode;
  bool     online;
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

  // Bateria (ECU / Smart Shunt, recebida por CAN no TCMS)
  float    battVoltage;
  float    battCurrent;
  float    battPower;
  float    battSoc;      // já em %
  bool     battValid;

  // Texto para a matriz de LEDs (comando 'M')
  char     matrixText[MATRIX_TEXT_MAX + 1];

  // Uma linha de diagnostico (so preenchida quando type == MSG_LOG)
  uint8_t  logSrc;
  uint8_t  logLevel;
  char     logText[LOG_TEXT_MAX + 1];
} espnow_message_t;

espnow_message_t incomingMsg;
espnow_message_t outgoingMsg;
unsigned long lastHeartbeat = 0;
uint8_t  relayMask   = 0;      // último estado conhecido dos relés
bool     motorArmed  = false;
char     currentMatrixText[MATRIX_TEXT_MAX + 1] = "ISTrain";
unsigned long lastTelemetryRx = 0;      // ultima telemetria vinda do TCMS
bool     tcmsAlive = false;
uint32_t espNowTxFail = 0;
#define TCMS_SILENT_MS 1500

#define ERROR_COUNT 22
const char* ERROR_NAMES[ERROR_COUNT] = {
  "Overcurrent", "Overload", "Overvoltage", "Undervoltage",
  "Controller Overheat", "Motor Overheat", "Motor Stalled", "Motor Out of phase",
  "Motor Sensor", "Motor AUX Sensor", "Encoder Misaligned", "Anti-Runaway Engaged",
  "Main Accelerator", "AUX Accelerator", "Pre-charge", "DC Contactor",
  "Power valve", "Current Sensor", "Auto-tune", "RS485", "CAN", "Software"
};

// ============================================================
// SERVIDOR WEB & WEBSOCKET
// ============================================================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>ISTrain Dashboard — EZkontrol</title>
<style>
  :root {
    --bg: #080d0f; --panel: #0f161a; --panel-border: #203039;
    --grid-line: #141e23; --ink: #e0eef2; --ink-dim: #7d9099;
    --ink-faint: #61757e; --accent: #19d3e6; --accent-dim: #0e7f8c;
    --warn: #f5a623; --danger: #ef3f4f; --ok: #3fef8a;
    --mono: 'IBM Plex Mono','SF Mono','Consolas',monospace;
    --display: 'Barlow Condensed','Arial Narrow',sans-serif;
    --control-bar-h: 100px;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  html, body { height: 100%; }
  body {
    background: var(--bg); color: var(--ink); font-family: var(--mono);
    background-image: linear-gradient(var(--grid-line) 1px,transparent 1px),linear-gradient(90deg,var(--grid-line) 1px,transparent 1px);
    background-size: 28px 28px;
    padding: 16px 16px calc(var(--control-bar-h) + 24px) 16px;
    user-select: none; -webkit-user-select: none;
    -webkit-tap-highlight-color: transparent;
  }
  .frame { max-width: 1440px; margin: 0 auto; }

  .header { display:flex; justify-content:space-between; align-items:flex-end; border-bottom:2px solid var(--accent-dim); padding-bottom:12px; margin-bottom:14px; flex-wrap:wrap; gap:10px; }
  .header-title { font-family:var(--display); font-weight:700; font-size:30px; text-transform:uppercase; }
  .header-title span { color:var(--accent); }
  .header-sub { font-size:11.5px; color:var(--ink-faint); letter-spacing:1.5px; text-transform:uppercase; margin-top:3px; }
  .conn-bar { display:flex; align-items:center; gap:10px; }
  .conn-dot { width:12px; height:12px; border-radius:50%; background:var(--ink-faint); transition:all 0.3s; }
  .conn-dot.live { background:var(--ok); box-shadow:0 0 8px var(--ok); }
  .conn-dot.connecting { background:var(--warn); animation:pulse 1s infinite; }
  .conn-dot.error { background:var(--danger); box-shadow:0 0 8px var(--danger); }
  @keyframes pulse { 0%,100%{opacity:1}50%{opacity:.35} }
  .conn-label { font-size:13px; text-transform:uppercase; letter-spacing:1px; color:var(--ink-dim); }
  button {
    font-family:var(--mono); font-size:13px; text-transform:uppercase; letter-spacing:1px;
    background:transparent; color:var(--accent); border:1px solid var(--accent-dim);
    padding:8px 16px; cursor:pointer; transition:all 0.15s; border-radius: 4px;
    touch-action: manipulation; -webkit-tap-highlight-color: transparent;
  }
  button:hover:not(:disabled) { background:var(--accent); color:var(--bg); }
  button:disabled { color:var(--ink-faint); border-color:var(--panel-border); cursor:not-allowed; }
  input[type=number], input[type=text] {
    font-family:var(--mono); font-size:15px; background:var(--bg); color:var(--ink);
    border:1px solid var(--panel-border); padding:8px 10px; width:100%; border-radius:4px;
  }
  input[type=number]:focus, input[type=text]:focus { outline:none; border-color:var(--accent-dim); }
  input[type=text]::placeholder { color:var(--ink-faint); }

  .status-strip { display:grid; grid-template-columns:repeat(5,1fr); gap:1px; background:var(--panel-border); border:1px solid var(--panel-border); margin-bottom:14px; }
  .status-cell { background:var(--panel); padding:10px 14px; }
  .status-cell-label { font-size:13px; color:var(--ink-faint); text-transform:uppercase; letter-spacing:1px; margin-bottom:3px; }
  .status-cell-value { font-family:var(--display); font-size:21px; font-weight:600; text-transform:uppercase; }
  .status-cell-value.ok { color:var(--ok); } .status-cell-value.warn { color:var(--warn); }
  .status-cell-value.danger { color:var(--danger); } .status-cell-value.idle { color:var(--ink-faint); }

  .main-layout { display:grid; grid-template-columns: 1fr 400px; gap: 14px; align-items: start; }
  @media (max-width: 1000px) { .main-layout { grid-template-columns: 1fr; } }
  .left-col { min-width: 0; }

  /* ---------- PAINEL DE POTÊNCIA ---------- */
  .power-panel { background:var(--panel); border:1px solid var(--panel-border); padding:12px 14px; margin-bottom:14px; }
  .power-grid { display:grid; grid-template-columns:repeat(4,1fr); gap:10px; margin-top:10px; }
  @media(max-width:900px){ .power-grid{grid-template-columns:repeat(2,1fr)} }
  @media(max-width:520px){ .power-grid{grid-template-columns:1fr} }
  .power-btn {
    display:flex; flex-direction:column; align-items:flex-start; gap:4px;
    padding:12px 14px; border:1px solid var(--panel-border); background:var(--bg);
    text-align:left; border-radius:4px; transition:all .15s; letter-spacing:0;
  }
  .power-btn:hover:not(:disabled) { background:var(--bg); border-color:var(--accent-dim); }
  .power-btn:disabled { opacity:.35; cursor:not-allowed; }
  .pb-step { font-size:11px; letter-spacing:1.2px; color:var(--ink-faint); text-transform:uppercase; }
  .pb-name { font-family:var(--display); font-size:21px; font-weight:700; text-transform:uppercase; color:var(--ink); line-height:1.1; }
  .pb-state { font-size:12px; letter-spacing:1px; color:var(--ink-faint); text-transform:uppercase; }
  .power-btn.on { border-color:var(--ok); background:rgba(63,239,138,.07); }
  .power-btn.on .pb-state { color:var(--ok); }
  .power-btn.on.is-lights { border-color:var(--warn); background:rgba(245,166,35,.08); }
  .power-btn.on.is-lights .pb-state { color:var(--warn); }
  .power-btn.on.is-horn { border-color:var(--ok); background:rgba(63,239,138,.10); }
  .power-btn.on.is-horn .pb-state { color:var(--ok); }
  .power-hint { margin-top:10px; font-size:13px; line-height:1.5; color:var(--ink-faint); border-left:2px solid var(--panel-border); padding-left:9px; }
  .power-hint.blocked { color:var(--warn); border-color:var(--warn); }
  .power-hint.ready { color:var(--ok); border-color:var(--ok); }

  /* ---------- MOSTRADORES ---------- */
  .gauge-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:10px; margin-bottom:14px; }
  @media(max-width:760px){.gauge-grid{grid-template-columns:repeat(2,1fr)}.status-strip{grid-template-columns:repeat(2,1fr)}}
  .gauge { background:var(--panel); border:1px solid var(--panel-border); padding:12px 14px; position:relative; overflow:hidden; cursor:pointer; transition:background 0.15s,border-color 0.15s; touch-action:manipulation; }
  .gauge:hover { background:#141c21; border-color:var(--accent-dim); }
  .gauge.pinned { border-color: var(--accent); background:#121a20; }
  .gauge::before { content:''; position:absolute; top:0;left:0;width:3px;height:100%;background:var(--accent-dim); }
  .gauge.pinned::before { background: var(--accent); }
  .gauge-pin-indicator { position:absolute;top:11px;right:11px;font-size:10.5px;letter-spacing:1px;text-transform:uppercase;color:var(--ink-faint); }
  .gauge.pinned .gauge-pin-indicator { color: var(--accent); }
  .gauge-label { font-size:13px;color:var(--ink-faint);text-transform:uppercase;letter-spacing:1.1px;margin-bottom:6px; }
  .gauge-value { font-family:var(--display);font-size:30px;font-weight:600;color:var(--ink);line-height:1;display:flex;align-items:baseline;gap:5px; }
  .gauge-unit { font-family:var(--mono);font-size:13px;color:var(--ink-dim);font-weight:400; }
  .gauge-bar-track { height:5px;background:var(--grid-line);margin-top:8px;position:relative; }
  .gauge-bar-fill { height:100%;background:var(--accent);transition:width 0.2s; }
  .gauge-bar-fill.warn { background:var(--warn); } .gauge-bar-fill.danger { background:var(--danger); }
  .gauge-bar-zero { position:absolute;top:-3px;width:1px;height:11px;background:var(--ink-faint); }
  .gauge-grid.offline .gauge-value { opacity:.3; }
  .gauge.stale .gauge-value { opacity:.3; }

  .section-title { display:flex;align-items:baseline;gap:10px;font-size:13px;color:var(--ink-dim);
    text-transform:uppercase;letter-spacing:1.4px;margin:0 0 8px 0;border-bottom:1px solid var(--panel-border);padding-bottom:6px; }
  .section-title .name { font-family:var(--display); font-size:19px; font-weight:700; color:var(--ink); letter-spacing:1px; }
  .section-title .tag { font-size:12px;letter-spacing:1px; }
  .section-title .tag.live { color:var(--ok); } .section-title .tag.dead { color:var(--danger); }
  .batt-grid { grid-template-columns:repeat(4,1fr); }
  @media(max-width:760px){ .batt-grid{grid-template-columns:repeat(2,1fr)} }

  .settings-panel { background:var(--panel); border:1px solid var(--panel-border); padding:12px 14px; margin-bottom:14px; }
  .settings-grid { display:grid; grid-template-columns:1fr 1fr; gap:12px; margin-top:10px; }
  @media(max-width:600px){.settings-grid{grid-template-columns:1fr}}
  .settings-field-label { font-size:12.5px; color:var(--ink-faint); text-transform:uppercase; letter-spacing:1px; margin-bottom:5px; }
  .settings-field-row { display:flex; gap:6px; }
  .settings-field-row input { flex:1; }
  .settings-field-row button { flex:0 0 auto; padding:8px 14px; }

  .lower-grid { display:grid;grid-template-columns:1fr 1fr;gap:12px; }
  @media(max-width:760px){.lower-grid{grid-template-columns:1fr}}
  .panel { background:var(--panel);border:1px solid var(--panel-border); }
  .panel-head { padding:9px 14px;border-bottom:1px solid var(--panel-border);font-size:13px;text-transform:uppercase;letter-spacing:1.3px;color:var(--ink-dim);display:flex;justify-content:space-between;align-items:center;gap:8px; }
  .panel-body { padding:12px 14px; }
  .error-empty { color:var(--ink-faint);font-size:14px; }
  .error-row { display:flex;align-items:center;gap:8px;padding:6px 0;font-size:14px;color:var(--danger);border-bottom:1px solid var(--grid-line); }
  .error-row:last-child{border-bottom:none} .error-row::before{content:'▸'}

  /* ---------- LOG ---------- */
  .log-panel { border-color: var(--accent-dim); }
  .log-box {
    height: 340px; overflow-y: auto; font-size:12px; line-height:1.65; color:var(--ink-dim);
    -webkit-overflow-scrolling: touch; touch-action: pan-y;
    user-select: text; -webkit-user-select: text; cursor: text;
    overscroll-behavior: contain;
  }
  .log-box::-webkit-scrollbar{width:9px}
  .log-box::-webkit-scrollbar-track{background:var(--bg)}
  .log-box::-webkit-scrollbar-thumb{background:var(--panel-border);border-radius:5px}
  .log-box::-webkit-scrollbar-thumb:hover{background:var(--accent-dim)}
  .log-line{white-space:pre-wrap;word-break:break-word}
  .log-line.t-error{color:var(--danger)} .log-line.t-warn{color:var(--warn)} .log-line.t-ok{color:var(--ok)} .log-line.t-cmd{color:var(--accent); font-weight:bold;}
  .log-paused { font-size:11.5px; color:var(--warn); letter-spacing:1px; }
  .log-jump { font-size:11px; padding:5px 10px; }
  .log-filters { display:flex; gap:5px; padding:8px 14px 0 14px; }
  .log-filters button { flex:1; font-size:10.5px; padding:5px 6px; letter-spacing:.5px; }
  .log-filters button.active { background:var(--accent); color:var(--bg); border-color:var(--accent); }
  .log-src { display:inline-block; min-width:38px; font-size:10px; font-weight:bold;
             color:var(--ink-faint); letter-spacing:.5px; }
  .log-line.t-error .log-src { color:var(--danger); }
  .log-line.t-warn  .log-src { color:var(--warn); }
  .log-line.t-ok    .log-src { color:var(--ok); }
  .log-box.only-warn .log-line:not(.t-warn):not(.t-error) { display:none; }
  .log-box.only-err  .log-line:not(.t-error) { display:none; }

  .right-col { display:flex; flex-direction:column; gap:12px; position: sticky; top: 16px;
               max-height: calc(100vh - var(--control-bar-h) - 32px); overflow-y: auto; overscroll-behavior: contain; }
  .right-col::-webkit-scrollbar{width:9px}
  .right-col::-webkit-scrollbar-thumb{background:var(--panel-border);border-radius:5px}
  @media (max-width: 1000px) { .right-col { position: static; max-height: none; overflow: visible; } }

  .chart-placeholder {
    background:var(--panel); border:1px dashed var(--panel-border); padding: 20px 16px;
    text-align:center; color:var(--ink-faint); font-size:13px; line-height:1.6;
  }
  .chart-card { background:var(--panel); border:1px solid var(--accent-dim); }
  .chart-card-head { display:flex; justify-content:space-between; align-items:center; padding:10px 14px; border-bottom:1px solid var(--panel-border); gap:8px; }
  .chart-card-title { font-family:var(--display); font-size:18px; font-weight:600; text-transform:uppercase; }
  .chart-card-close { background:transparent;border:1px solid var(--panel-border);color:var(--ink-dim);width:30px;height:30px;font-size:15px;line-height:1;padding:0;cursor:pointer;flex:0 0 auto; }
  .chart-card-close:hover { border-color:var(--danger); color:var(--danger); background:transparent; }
  .chart-card-body { padding: 12px 14px; }
  .modal-stats-row{display:grid;grid-template-columns:repeat(4,1fr);gap:1px;background:var(--panel-border);border:1px solid var(--panel-border);margin-bottom:10px}
  .modal-stat{background:var(--bg);padding:7px 8px}
  .modal-stat-label{font-size:11px;color:var(--ink-faint);text-transform:uppercase;letter-spacing:.5px;margin-bottom:2px}
  .modal-stat-value{font-family:var(--display);font-size:16px;font-weight:600;color:var(--ink)}
  .chart-wrap{background:var(--bg);border:1px solid var(--panel-border);padding:8px}
  .chart-wrap canvas{display:block;width:100%}
  .modal-range-row{display:flex;gap:6px;margin-top:10px}
  .range-btn{flex:1;font-size:11px;padding:8px}
  .range-btn.active{background:var(--accent);color:var(--bg);border-color:var(--accent)}
  .modal-empty{text-align:center;color:var(--ink-faint);font-size:12px;padding:30px 10px}

  /* ---------- BARRA DE CONTROLO ---------- */
  .control-bar {
    position: fixed; left:0; right:0; bottom:0; height: var(--control-bar-h);
    background: #0b1216; border-top: 2px solid var(--accent-dim);
    display:flex; align-items:center; gap:14px; padding: 0 16px; z-index: 80;
  }
  .control-bar-slider-section { flex:1; min-width:0; }
  .control-bar-top { display:flex; justify-content:space-between; align-items:baseline; margin-bottom:8px; }
  .control-bar-label { font-size:13px; color:var(--ink-faint); text-transform:uppercase; letter-spacing:1.3px; }
  .control-bar-current { font-family:var(--display); font-size:28px; font-weight:700; color:var(--accent); }
  .control-bar-current .unit { font-family:var(--mono); font-size:14px; color:var(--ink-dim); margin-left:3px; }
  .control-bar-current.stopped { color: var(--ink-faint); }

  .key-row { display:flex; align-items:center; gap:10px; }
  .key-badge {
    font-family:var(--display); font-size:16px; font-weight:700; letter-spacing:1px;
    border:2px solid var(--panel-border); color:var(--ink-faint); padding:12px 14px;
    text-align:center; border-radius:4px; transition:border-color .08s,color .08s,background .08s;
    min-width:150px; cursor:pointer;
    touch-action:none; -webkit-tap-highlight-color:transparent; -webkit-user-select:none; user-select:none;
  }
  .key-badge.active {
    border-color:var(--accent); color:var(--accent); background:rgba(25,211,230,0.12);
    box-shadow:0 0 14px rgba(25,211,230,0.28);
  }
  .key-badge.locked { opacity:.4; cursor:not-allowed; }
  .dir-badge {
    flex:1; text-align:center; font-family:var(--display); font-size:17px; font-weight:700;
    letter-spacing:2px; color:var(--ink-faint); text-transform:uppercase;
  }
  .dir-badge.fwd { color:var(--ok); } .dir-badge.rev { color:var(--warn); }

  .control-bar-actions { flex: 0 0 auto; display:flex; gap:10px; align-items:center; }
  .btn-brake {
    padding:0 22px; height:56px; font-size:15px; font-weight:700; letter-spacing:1.2px;
    background:rgba(245,166,35,0.10); color:var(--warn); border:2px solid var(--warn);
    cursor:pointer; font-family:var(--mono); text-transform:uppercase; white-space:nowrap;
    transition:background .1s, color .1s, box-shadow .1s;
  }
  .btn-brake:hover { background:var(--warn); color:#1a1200; }
  .btn-brake .kbd {
    display:inline-block; margin-left:8px; padding:1px 6px; font-size:11px;
    border:1px solid currentColor; border-radius:3px; opacity:.8;
  }
  .btn-brake.fired { background:var(--warn); color:#1a1200; box-shadow:0 0 20px rgba(245,166,35,.6); }
  .btn-resume {
    padding:0 18px; height:56px; font-size:13px; background:transparent;
    color:var(--ok); border:1px solid var(--ok); cursor:pointer;
    font-family:var(--mono); text-transform:uppercase; transition:all 0.15s;
  }
  .btn-resume:hover:not(:disabled) { background:var(--ok); color:var(--bg); }
  .btn-resume:disabled { color:var(--ink-faint); border-color:var(--panel-border); cursor:not-allowed; }
  .btn-estop {
    padding:0 26px; height:56px; font-size:16px; font-weight:700; letter-spacing:1.5px;
    background:var(--danger); color:#fff; border:2px solid #ff6070;
    cursor:pointer; text-transform:uppercase; font-family:var(--mono);
    transition:all 0.1s; white-space:nowrap;
  }
  .btn-estop:hover { background:#ff1020; box-shadow:0 0 20px var(--danger); }
  /* Parado: pisca a vermelho com "PARAR", alternando cheio e escuro */
  .btn-estop.armed { animation: estop-blink 0.8s infinite; }
  @keyframes estop-blink {
    0%, 49.9% { background:var(--danger); color:#fff; border-color:#ff6070; box-shadow:0 0 22px var(--danger); }
    50%, 100% { background:#1a0a0a; color:var(--danger); border-color:var(--danger); box-shadow:none; }
  }

  .estop-overlay {
    display:none; position:fixed; inset:0; bottom: var(--control-bar-h);
    background:rgba(239,63,79,0.12); border:3px solid var(--danger);
    pointer-events:none; z-index:50; animation:blink-border 0.8s infinite;
  }
  @keyframes blink-border { 0%,100%{opacity:1}50%{opacity:0.3} }
  .estop-overlay.active { display:block; }

  @media (pointer: coarse) {
    :root { --control-bar-h: 124px; }
    .key-badge { padding:18px 16px; font-size:18px; min-width:160px; }
    .power-btn { padding:16px 14px; }
    .gauge { padding:16px 14px; }
    button { padding:12px 18px; }
    .btn-estop, .btn-resume, .btn-brake { height:64px; }
  }
  @media (max-width: 640px) {
    :root { --control-bar-h: 180px; }
    .control-bar { flex-wrap: wrap; padding: 8px 12px; }
    .control-bar-slider-section { flex: 1 1 100%; order: 1; }
    .control-bar-actions { flex: 1 1 100%; order: 2; justify-content: stretch; }
    .btn-estop, .btn-resume, .btn-brake { flex:1; padding:0 8px; }
    .key-badge { min-width:0; flex:1; }
  }
</style>
</head>
<body>

<div class="estop-overlay" id="estopOverlay"></div>

<div class="frame">

  <div class="header">
    <div>
      <div class="header-title">ISTrain <span>DASHBOARD</span></div>
      <div class="header-sub">EZkontrol MCU ×2 ↔ ESP32-S3 · CAN 250K · WiFi</div>
    </div>
    <div style="display:flex;align-items:center;gap:12px">
      <div class="conn-bar">
        <div class="conn-dot" id="connDot"></div>
        <div class="conn-label" id="connLabel">DESLIGADO</div>
      </div>
    </div>
  </div>

  <div class="status-strip">
    <div class="status-cell"><div class="status-cell-label">Sistema</div><div class="status-cell-value danger" id="armStatus">BLOQUEADO</div></div>
    <div class="status-cell"><div class="status-cell-label">Controlador 1</div><div class="status-cell-value idle" id="mcuStatus0">OFFLINE</div></div>
    <div class="status-cell"><div class="status-cell-label">Controlador 2</div><div class="status-cell-value idle" id="mcuStatus1">OFFLINE</div></div>
    <div class="status-cell"><div class="status-cell-label">Erros Ativos</div><div class="status-cell-value ok" id="errCount">0</div></div>
    <div class="status-cell"><div class="status-cell-label">Erros CAN</div><div class="status-cell-value idle" id="canErrCount">0</div></div>
  </div>

  <div class="main-layout">
    <div class="left-col">

      <div class="power-panel">
        <div class="gauge-label">Sequência de Arranque — Relés do TCMS</div>
        <div class="power-grid">
          <button class="power-btn" id="btnDc">
            <span class="pb-step">Passo 1 · GPIO 17</span>
            <span class="pb-name">DC Contactor</span>
            <span class="pb-state" id="stDc">OFF — SEM POTÊNCIA</span>
          </button>
          <button class="power-btn" id="btnKey" disabled>
            <span class="pb-step">Passo 2 · GPIO 8</span>
            <span class="pb-name">Key Switch</span>
            <span class="pb-state" id="stKey">OFF — BLOQUEADO</span>
          </button>
          <button class="power-btn is-lights" id="btnLightsPanel" disabled>
            <span class="pb-step">Auto · GPIO 7 · pelas RPM</span>
            <span class="pb-name">Luzes</span>
            <span class="pb-state" id="stLights">OFF — BLOQUEADO</span>
          </button>
          <button class="power-btn is-horn" id="btnHornPanel" disabled>
            <span class="pb-step">Auxiliar · GPIO 15 · 0,5 s</span>
            <span class="pb-name">Buzina</span>
            <span class="pb-state" id="stHorn">EM ESPERA — BLOQUEADA</span>
          </button>
        </div>
        <div class="power-hint blocked" id="powerHint">
          DC Contactor desligado: nada funciona. Liga primeiro o DC Contactor, depois o Key Switch.
        </div>
      </div>

      <!-- As duas secções dos controladores são geradas em JS -->
      <div id="mcuSections"></div>

      <div class="section-title">
        <span class="name">Bateria</span>
        <span>Smart Shunt · ECU → CAN → TCMS</span>
        <span class="tag dead" id="battTag">SEM DADOS</span>
      </div>
      <div class="gauge-grid batt-grid" id="battGrid"></div>

      <div class="settings-panel">
        <div class="gauge-label">Limites de Segurança — aplicados aos dois controladores</div>
        <div class="settings-grid">
          <div>
            <div class="settings-field-label">Velocidade Máxima (RPM)</div>
            <div class="settings-field-row">
              <input type="number" id="inputMaxRpm" value="1000" min="0" max="8000" step="10">
              <button id="btnApplyMax">Aplicar</button>
            </div>
          </div>
          <div>
            <div class="settings-field-label">Limite de Corrente (A)</div>
            <div class="settings-field-row">
              <input type="number" id="inputAmps" value="30" min="0" max="200" step="1">
              <button id="btnApplyAmps">Aplicar</button>
            </div>
          </div>
        </div>
      </div>

      <div class="settings-panel">
        <div class="gauge-label">Matriz de LEDs — 32 × 8</div>
        <div class="settings-field-label" style="margin-top:10px">Texto a mostrar — máx. 32 caracteres, sem acentos</div>
        <div class="settings-field-row">
          <input type="text" id="inputMatrix" maxlength="32" placeholder="ISTrain" autocomplete="off" spellcheck="false">
          <button id="btnMatrix">Enviar</button>
        </div>
        <div class="power-hint" id="matrixHint">A mostrar: ISTrain</div>
      </div>

      <div class="lower-grid">
        <div class="panel">
          <div class="panel-head"><span>Erros — Controlador 1</span><span id="errPanelCount0">0 ativos</span></div>
          <div class="panel-body"><div id="errorList0"><div class="error-empty">Sem erros reportados.</div></div></div>
        </div>
        <div class="panel">
          <div class="panel-head"><span>Erros — Controlador 2</span><span id="errPanelCount1">0 ativos</span></div>
          <div class="panel-body"><div id="errorList1"><div class="error-empty">Sem erros reportados.</div></div></div>
        </div>
      </div>
    </div>

    <div class="right-col" id="rightCol">
      <div class="panel log-panel">
        <div class="panel-head">
          <span>Log</span>
          <span style="display:flex;align-items:center;gap:8px">
            <span class="log-paused" id="logPaused" style="display:none">PAUSA</span>
            <button class="log-jump" id="btnLogBottom">▼ Fim</button>
            <span id="logCount">0 linhas</span>
          </span>
        </div>
        <div class="log-filters">
          <button class="active" data-filter="all">Tudo</button>
          <button data-filter="warn">Avisos +</button>
          <button data-filter="err">Só erros</button>
        </div>
        <div class="panel-body">
          <div class="log-box" id="logBox"><div class="log-line" style="color:var(--ink-faint)">A ligar ao ESP32...</div></div>
        </div>
      </div>

      <div class="chart-placeholder" id="chartPlaceholder">
        Toca em qualquer valor à esquerda<br>para abrir o gráfico aqui, por baixo do log.<br>
        Podes ter vários abertos ao mesmo tempo.
      </div>
    </div>
  </div>
</div>

<div class="control-bar">
  <div class="control-bar-slider-section">
    <div class="control-bar-top">
      <div class="control-bar-label">Velocidade Alvo — prime e mantém · X trava</div>
      <div class="control-bar-current" id="targetDisplay">0<span class="unit">RPM</span></div>
    </div>
    <div class="key-row">
      <div class="key-badge locked" id="keyS" role="button" tabindex="0" aria-label="Trás">◄ S — TRÁS</div>
      <div class="dir-badge" id="dirBadge">PARADO</div>
      <div class="key-badge locked" id="keyW" role="button" tabindex="0" aria-label="Frente">FRENTE — W ►</div>
    </div>
  </div>
  <div class="control-bar-actions">
    <button class="btn-brake" id="btnBrake" title="Travar: alvo a 0 RPM (tecla X)">■ Travar<span class="kbd">X</span></button>
    <button class="btn-resume" id="btnResume" disabled>▶ Retomar</button>
    <button class="btn-estop" id="btnEstop">⚠ STOP</button>
  </div>
</div>

<script>
(() => {
  const $ = id => document.getElementById(id);
  const els = {
    connDot:$('connDot'), connLabel:$('connLabel'),
    armStatus:$('armStatus'), mcuStatus:[$('mcuStatus0'), $('mcuStatus1')],
    errCount:$('errCount'), canErrCount:$('canErrCount'),
    mcuSections:$('mcuSections'), battGrid:$('battGrid'), battTag:$('battTag'),
    errorList:[$('errorList0'), $('errorList1')],
    errPanelCount:[$('errPanelCount0'), $('errPanelCount1')],
    logBox:$('logBox'), logCount:$('logCount'), logPaused:$('logPaused'), btnLogBottom:$('btnLogBottom'),
    btnEstop:$('btnEstop'), btnResume:$('btnResume'), btnBrake:$('btnBrake'),
    btnDc:$('btnDc'), btnKey:$('btnKey'), btnLightsPanel:$('btnLightsPanel'),
    btnHornPanel:$('btnHornPanel'), stHorn:$('stHorn'),
    stDc:$('stDc'), stKey:$('stKey'), stLights:$('stLights'), powerHint:$('powerHint'),
    targetDisplay:$('targetDisplay'), keyW:$('keyW'), keyS:$('keyS'), dirBadge:$('dirBadge'),
    inputMaxRpm:$('inputMaxRpm'), btnApplyMax:$('btnApplyMax'),
    inputAmps:$('inputAmps'), btnApplyAmps:$('btnApplyAmps'),
    inputMatrix:$('inputMatrix'), btnMatrix:$('btnMatrix'), matrixHint:$('matrixHint'),
    estopOverlay:$('estopOverlay'),
    rightCol:$('rightCol'), chartPlaceholder:$('chartPlaceholder'),
  };

  const NUM_MCU = 2;

  // ---------------- DEFINIÇÃO DOS MOSTRADORES ----------------
  // Uma única tabela descreve cada grandeza: rótulo, unidade, casas decimais,
  // escala da barra e limiares de cor. As duas secções de controlador e os
  // gráficos saem todos daqui, por isso não há valores repetidos pelo código.
  const MCU_METRICS = [
    {key:'busVoltage',   label:'Tensão BUS',        unit:'V',   dec:1, min:0,     max:100},
    {key:'busCurrent',   label:'Corrente BUS',      unit:'A',   dec:1, min:-100,  max:100,  centered:true},
    {key:'phaseCurrent', label:'Corrente Fase',     unit:'A',   dec:1, min:-150,  max:150,  centered:true},
    {key:'speed',        label:'Velocidade',        unit:'RPM', dec:0, min:-4000, max:4000, centered:true},
    {key:'ctrlTemp',     label:'Temp. Controlador', unit:'°C',  dec:0, min:-40,   max:120,  hiWarn:60, hiDanger:80},
    {key:'motorTemp',    label:'Temp. Motor',       unit:'°C',  dec:0, min:-40,   max:150,  hiWarn:80, hiDanger:100},
  ];
  const BATT_METRICS = [
    {key:'battVoltage', label:'Tensão Bateria',   unit:'V', dec:2, min:0,      max:100},
    {key:'battCurrent', label:'Corrente Bateria', unit:'A', dec:2, min:-200,   max:200,   centered:true},
    {key:'battPower',   label:'Potência',         unit:'W', dec:0, min:-10000, max:10000, centered:true},
    {key:'battSoc',     label:'SOC',              unit:'%', dec:1, min:0,      max:100,   loWarn:30, loDanger:15},
  ];

  const METRICS = {};   // id -> definição (inclui chartLabel)
  const history = {};   // id -> [{t,v}]

  function gaugeHtml(id, m, chartLabel){
    const zero = m.centered ? '<div class="gauge-bar-zero" style="left:50%"></div>' : '';
    const ml   = m.centered ? ';margin-left:50%' : '';
    return `<div class="gauge" data-metric="${id}">
      <span class="gauge-pin-indicator">gráfico</span>
      <div class="gauge-label">${m.label}</div>
      <div class="gauge-value"><span data-val="${id}">—</span><span class="gauge-unit">${m.unit}</span></div>
      <div class="gauge-bar-track">${zero}<div class="gauge-bar-fill" data-bar="${id}" style="width:0%${ml}"></div></div>
    </div>`;
  }

  function registerMetric(id, m, chartLabel){
    METRICS[id] = Object.assign({}, m, {chartLabel});
    history[id] = [];
  }

  // Secções dos dois controladores
  let html = '';
  for(let c = 0; c < NUM_MCU; c++){
    html += `<div class="section-title">
      <span class="name">Controlador ${c+1}</span>
      <span>EZkontrol MCU · nó 0x${c===0?'EF':'F0'}</span>
      <span class="tag dead" data-mcutag="${c}">OFFLINE</span>
    </div>
    <div class="gauge-grid offline" data-mcugrid="${c}">`;
    MCU_METRICS.forEach(m=>{
      const id = 'm'+c+'_'+m.key;
      registerMetric(id, m, 'CTRL '+(c+1)+' · '+m.label);
      html += gaugeHtml(id, m);
    });
    html += '</div>';
  }
  els.mcuSections.innerHTML = html;

  // Secção da bateria
  let bhtml = '';
  BATT_METRICS.forEach(m=>{
    registerMetric(m.key, m, m.label);
    bhtml += gaugeHtml(m.key, m);
  });
  els.battGrid.innerHTML = bhtml;
  document.querySelectorAll('#battGrid .gauge').forEach(g=>g.classList.add('stale'));

  const mcuTags  = [...document.querySelectorAll('[data-mcutag]')];
  const mcuGrids = [...document.querySelectorAll('[data-mcugrid]')];
  const valEls = {}, barEls = {};
  Object.keys(METRICS).forEach(id=>{
    valEls[id] = document.querySelector(`[data-val="${id}"]`);
    barEls[id] = document.querySelector(`[data-bar="${id}"]`);
  });

  // ---------------- ESTADO ----------------
  let targetRpm = 0;
  let isStopped = true;
  let wsConnected = false;
  const relays = { lights:false, dc:false, key:false, horn:false };
  let motorReady = false;
  const errCounts = [0, 0];

  // ---------------- HISTÓRICO ----------------
  const RETENTION = 30*60*1000;
  function pushHistory(id, v){
    const a = history[id]; if(!a) return;
    const now = Date.now();
    a.push({t:now, v});
    const cut = now - RETENTION;
    while(a.length && a[0].t < cut) a.shift();
  }
  function clampPct(v, m){ return Math.max(0, Math.min(100, ((v-m.min)/(m.max-m.min))*100)); }
  function setCenteredBar(el, v, m){
    const h = Math.max(Math.abs(m.min), Math.abs(m.max));
    const p = Math.max(-50, Math.min(50, (v/h)*50));
    if(p >= 0){ el.style.marginLeft='50%'; el.style.width=p+'%'; }
    else { el.style.marginLeft=(50+p)+'%'; el.style.width=(-p)+'%'; }
  }
  function setMetric(id, value){
    const m = METRICS[id]; if(!m || value===undefined || value===null) return;
    const v = Number(value); if(!isFinite(v)) return;
    valEls[id].textContent = v.toFixed(m.dec);
    pushHistory(id, v);
    const bar = barEls[id];
    if(m.centered) setCenteredBar(bar, v, m);
    else bar.style.width = clampPct(v, m)+'%';
    let cls = '';
    if(m.hiDanger !== undefined) cls = v > m.hiDanger ? ' danger' : (v > m.hiWarn ? ' warn' : '');
    if(m.loDanger !== undefined) cls = v < m.loDanger ? ' danger' : (v < m.loWarn ? ' warn' : '');
    bar.className = 'gauge-bar-fill'+cls;
  }

  // ---------------- WEBSOCKET ----------------
  let ws=null, logLines=0, canErrTotal=0;

  function connectWs(){
    ws = new WebSocket(`ws://${location.hostname}/ws`);
    setConn('connecting');
    ws.onopen = ()=>{ wsConnected=true; setConn('live'); addLog('WebSocket ligado.','t-ok'); };
    ws.onclose = ()=>{
      wsConnected=false; setConn('idle');
      addLog('Ligação perdida. A tentar religar...','t-warn');
      motorReady=false; setEstopUI(true);
      for(let c=0;c<NUM_MCU;c++) setMcuOnline(c, false);
      refreshPowerUI();
      setTimeout(connectWs,1500);
    };
    ws.onerror = ()=>{ setConn('error'); };
    ws.onmessage = ev => { try{ handleJson(JSON.parse(ev.data)); }catch(e){} };
  }
  function send(obj){ if(ws && wsConnected) ws.send(JSON.stringify(obj)); }
  setInterval(()=>{ if(wsConnected) send({cmd:'ping'}); }, 400);

  function setConn(s){
    const m={idle:'DESLIGADO',connecting:'A LIGAR...',live:'LIGADO',error:'ERRO'};
    els.connDot.className='conn-dot '+s;
    els.connLabel.textContent=m[s]||s;
    updateResumeBtn();
  }

  // ---------------- MENSAGENS DO ESP32 ----------------
  function handleJson(o){
    switch(o.type){
      case 'boot':
        addLog(`[BOOT] reset=${o.reset_ok} bitrate=${o.bitrate_ok}`, o.reset_ok&&o.bitrate_ok?'t-ok':'t-error');
        break;
      case 'log':
        // Vem do TCMS, da ECU ou do proprio posto de comando
        addLog(o.text, LEVEL_CLASS[o.level|0] || '', o.src || 'TCMS');
        break;
      case 'can_error':
        canErrTotal++;
        els.canErrCount.textContent=canErrTotal;
        els.canErrCount.className='status-cell-value danger';
        addLog('[CAN_ERROR] código '+o.code,'t-error');
        break;
      case 'relays':
        applyRelayStates(o);
        break;
      case 'matrix':
        els.matrixHint.textContent = (o.text && o.text.length) ? 'A mostrar: '+o.text : 'Matriz apagada.';
        break;
      case 'batt':
        applyBattery(o);
        break;
      case 'mcu':
        applyMcu(o);
        break;
    }
  }

  // ---------------- TELEMETRIA DOS CONTROLADORES ----------------
  function setMcuOnline(c, online){
    mcuTags[c].textContent = online ? 'A RECEBER' : 'OFFLINE';
    mcuTags[c].className = 'tag ' + (online ? 'live' : 'dead');
    mcuGrids[c].classList.toggle('offline', !online);
    if(!online){
      els.mcuStatus[c].textContent = 'OFFLINE';
      els.mcuStatus[c].className = 'status-cell-value idle';
    }
  }

  function applyMcu(o){
    const c = o.idx|0;
    if(c < 0 || c >= NUM_MCU) return;

    setMcuOnline(c, !!o.online);
    if(o.online){
      els.mcuStatus[c].textContent = o.running ? 'RUNNING' : 'HALTED';
      els.mcuStatus[c].className = 'status-cell-value ' + (o.running ? 'ok' : 'warn');
    }

    MCU_METRICS.forEach(m=> setMetric('m'+c+'_'+m.key, o[m.key]));
    renderErrors(c, o.errors||[]);
    refreshPinnedCharts();
  }

  function renderErrors(c, errors){
    errCounts[c] = errors.length;
    const total = errCounts[0] + errCounts[1];
    els.errCount.textContent = total;
    els.errCount.className = 'status-cell-value ' + (total>0 ? 'danger' : 'ok');
    els.errPanelCount[c].textContent = errors.length+' ativos';
    els.errorList[c].innerHTML = errors.length===0
      ? '<div class="error-empty">Sem erros reportados.</div>'
      : errors.map(e=>`<div class="error-row">${e}</div>`).join('');
  }

  // ---------------- BATERIA ----------------
  function applyBattery(o){
    const battGauges = document.querySelectorAll('#battGrid .gauge');
    if(!o.valid){
      els.battTag.textContent='SEM DADOS DA ECU';
      els.battTag.className='tag dead';
      battGauges.forEach(g=>g.classList.add('stale'));
      return;
    }
    els.battTag.textContent='A RECEBER';
    els.battTag.className='tag live';
    battGauges.forEach(g=>g.classList.remove('stale'));

    setMetric('battVoltage', o.voltage);
    setMetric('battCurrent', o.current);
    setMetric('battPower',   o.power);
    setMetric('battSoc',     o.soc);
    refreshPinnedCharts();
  }

  // ---------------- LOG ----------------
  // Só se salta para o fim se o utilizador já lá estava. Assim dá para
  // ler tranquilamente uma linha antiga sem o log a puxar para baixo.
  let logStick = true;
  function atBottom(){
    const b = els.logBox;
    return b.scrollHeight - b.scrollTop - b.clientHeight < 24;
  }
  els.logBox.addEventListener('scroll', ()=>{
    logStick = atBottom();
    els.logPaused.style.display = logStick ? 'none' : 'inline';
  });
  document.querySelectorAll('.log-filters button').forEach(b=>{
    b.addEventListener('click', ()=>{
      document.querySelectorAll('.log-filters button').forEach(x=>x.classList.remove('active'));
      b.classList.add('active');
      const f = b.dataset.filter;
      els.logBox.classList.toggle('only-warn', f==='warn');
      els.logBox.classList.toggle('only-err',  f==='err');
      els.logBox.scrollTop = els.logBox.scrollHeight;
    });
  });

  els.btnLogBottom.addEventListener('click', ()=>{
    els.logBox.scrollTop = els.logBox.scrollHeight;
    logStick = true;
    els.logPaused.style.display = 'none';
  });

  const LEVEL_CLASS = ['', 't-ok', 't-warn', 't-error'];

  function addLog(text, cls, src){
    const d=document.createElement('div');
    d.className='log-line'+(cls?' '+cls:'');
    const hora = new Date().toLocaleTimeString('pt-PT');
    const tag = document.createElement('span');
    tag.className = 'log-src';
    tag.textContent = src || 'WEB';
    d.appendChild(document.createTextNode(hora + ' '));
    d.appendChild(tag);
    d.appendChild(document.createTextNode(' ' + text));
    els.logBox.appendChild(d);
    logLines++;
    while(els.logBox.children.length>400) els.logBox.removeChild(els.logBox.firstChild);
    if(logStick) els.logBox.scrollTop = els.logBox.scrollHeight;
    els.logCount.textContent = logLines+' linhas';
  }

  // ---------------- RELÉS ----------------
  const RELAY_LIGHTS=0, RELAY_DC=1, RELAY_KEY=2, RELAY_HORN=3;
  function sendRelay(id, state){ send({cmd:'relay', id:id, state:state}); }

  function applyRelayStates(o){
    const wasReady = motorReady;
    relays.lights = !!o.lights;
    relays.dc     = !!o.dc;
    relays.key    = !!o.key;
    relays.horn   = !!o.horn;
    motorReady    = !!o.motorEnabled;
    if(wasReady && !motorReady){
      targetRpm = 0;
      setEstopUI(true);
      addLog('⚠ Interlock aberto: motor desarmado','t-error');
    }
    refreshPowerUI();
  }

  function refreshPowerUI(){
    els.btnDc.classList.toggle('on', relays.dc);
    els.stDc.textContent = relays.dc ? 'ON — POTÊNCIA PRESENTE' : 'OFF — SEM POTÊNCIA';
    els.btnDc.disabled = !wsConnected;

    els.btnKey.classList.toggle('on', relays.key);
    els.btnKey.disabled = !wsConnected || !relays.dc;
    els.stKey.textContent = relays.key ? 'ON — MOTOR ARMADO'
                          : (relays.dc ? 'OFF — PRONTO A LIGAR' : 'OFF — BLOQUEADO');

    els.btnLightsPanel.classList.toggle('on', relays.lights);
    els.btnLightsPanel.disabled = !wsConnected || !relays.dc;
    els.stLights.textContent = relays.lights ? 'ON — AUTO / MANUAL'
                             : (relays.dc ? 'OFF — AUTO / MANUAL' : 'OFF — BLOQUEADO');

    els.btnHornPanel.classList.toggle('on', relays.horn);
    els.btnHornPanel.disabled = !wsConnected || !relays.dc || relays.horn;
    els.stHorn.textContent = relays.horn ? 'A TOCAR'
                           : (relays.dc ? 'PRONTA — TOCA PARA BUZINAR' : 'EM ESPERA — BLOQUEADA');

    els.armStatus.textContent = motorReady ? 'ARMADO' : 'BLOQUEADO';
    els.armStatus.className = 'status-cell-value ' + (motorReady ? 'ok' : 'danger');

    if(!relays.dc){
      els.powerHint.className='power-hint blocked';
      els.powerHint.textContent='DC Contactor desligado: nada funciona. Liga primeiro o DC Contactor, depois o Key Switch.';
    } else if(!relays.key){
      els.powerHint.className='power-hint blocked';
      els.powerHint.textContent='Falta o Key Switch. O motor só arranca com DC Contactor e Key Switch ligados.';
    } else {
      els.powerHint.className='power-hint ready';
      els.powerHint.textContent='DC Contactor + Key Switch ON. Carrega em Retomar para sair do E-STOP e usar FRENTE/TRÁS.';
    }
    updateResumeBtn();
    updateTargetUI();
  }

  els.btnDc.addEventListener('click', ()=>{
    const want = !relays.dc;
    sendRelay(RELAY_DC, want);
    addLog('[DC CONTACTOR] pedido -> '+(want?'ON':'OFF'),'t-cmd');
  });
  els.btnKey.addEventListener('click', ()=>{
    if(!relays.dc){ addLog('Key Switch bloqueado: liga o DC Contactor primeiro.','t-warn'); return; }
    const want = !relays.key;
    sendRelay(RELAY_KEY, want);
    addLog('[KEY SWITCH] pedido -> '+(want?'ON':'OFF'),'t-cmd');
  });
  function toggleLights(){
    if(!relays.dc){ addLog('Luzes bloqueadas: liga o DC Contactor primeiro.','t-warn'); return; }
    const want = !relays.lights;
    sendRelay(RELAY_LIGHTS, want);
    addLog('[LUZES] pedido -> '+(want?'HIGH':'LOW'),'t-cmd');
  }
  function pressHorn(){
    if(!relays.dc){ addLog('Buzina bloqueada: liga o DC Contactor primeiro.','t-warn'); return; }
    if(relays.horn) return;
    sendRelay(RELAY_HORN, true);
    addLog('[BUZINA] toque de 0,5 s','t-cmd');
  }
  els.btnHornPanel.addEventListener('click', pressHorn);
  els.btnLightsPanel.addEventListener('click', toggleLights);

  // ---------------- E-STOP / RETOMAR ----------------
  function updateResumeBtn(){ els.btnResume.disabled = !(isStopped && wsConnected && motorReady); }

  function setEstopUI(stopped){
    isStopped=stopped;
    els.estopOverlay.classList.toggle('active',stopped);
    els.btnEstop.textContent=stopped?'⚠ PARAR':'⚠ STOP';
    els.btnEstop.classList.toggle('armed',stopped);
    if(stopped){ targetRpm=0; releaseDir('w'); releaseDir('s'); }
    updateTargetUI();
    updateResumeBtn();
  }
  els.btnEstop.addEventListener('click',()=>{
    setEstopUI(true);
    send({cmd:'stop'});
    addLog('⚠ EMERGENCY STOP activado','t-error');
  });
  els.btnResume.addEventListener('click',()=>{
    if(!motorReady){ addLog('Retomar bloqueado: faltam DC Contactor e/ou Key Switch.','t-warn'); return; }
    setEstopUI(false);
    send({cmd:'resume'});
    addLog('▶ Controlo retomado','t-ok');
  });

  // ---------------- CONTROLO: TECLADO W/S + TOQUE ----------------
  let maxRpm = parseInt(els.inputMaxRpm.value,10) || 1000;
  const STEP = 10, TICK_MS = 120;   // 10 RPM por passo: velocidades baixas do rollout
  const keyState = { w:false, s:false };
  let tickTimer = null;

  function updateTargetUI(){
    els.targetDisplay.innerHTML = targetRpm + '<span class="unit">RPM</span>';
    const blocked = isStopped || !motorReady;
    els.targetDisplay.classList.toggle('stopped', blocked || targetRpm===0);
    els.keyW.classList.toggle('locked', blocked);
    els.keyS.classList.toggle('locked', blocked);
    if(targetRpm > 0){ els.dirBadge.textContent='FRENTE'; els.dirBadge.className='dir-badge fwd'; }
    else if(targetRpm < 0){ els.dirBadge.textContent='TRÁS'; els.dirBadge.className='dir-badge rev'; }
    else { els.dirBadge.textContent = motorReady ? 'PARADO' : 'BLOQUEADO'; els.dirBadge.className='dir-badge'; }
  }

  function tick(){
    if(isStopped || !motorReady) return;
    if(keyState.w && !keyState.s) targetRpm = Math.min(maxRpm, targetRpm + STEP);
    else if(keyState.s && !keyState.w) targetRpm = Math.max(-maxRpm, targetRpm - STEP);
    else return;
    updateTargetUI();
    send({cmd:'setTarget', value: targetRpm});
  }
  function startTick(){ if(!tickTimer){ tick(); tickTimer = setInterval(tick, TICK_MS); } }
  function stopTickIfIdle(){ if(!keyState.w && !keyState.s && tickTimer){ clearInterval(tickTimer); tickTimer=null; } }

  function pressDir(dir){
    if(dir==='w'){
      if(keyState.w) return;
      keyState.w = true; els.keyW.classList.add('active');
    } else {
      if(keyState.s) return;
      keyState.s = true; els.keyS.classList.add('active');
    }
    // As luzes não dependem da tecla: o TCMS comuta-as pelo sinal das RPM reais.
    startTick();
  }
  function releaseDir(dir){
    if(dir==='w'){ keyState.w = false; els.keyW.classList.remove('active'); }
    else { keyState.s = false; els.keyS.classList.remove('active'); }
    stopTickIfIdle();
  }
  function releaseAll(){ releaseDir('w'); releaseDir('s'); }

  function isTypingInField(){
    const a = document.activeElement;
    return a && (a.tagName==='INPUT' || a.tagName==='TEXTAREA');
  }

  window.addEventListener('keydown', e=>{
    if(isTypingInField()) return;
    // e.repeat: o teclado repete a tecla enquanto a manténs premida. Ignorar
    // essas repetições impede que, depois de travar com W ainda em baixo,
    // a locomotiva volte a acelerar sozinha. Para voltar a andar, larga e prime.
    if(e.key==='w'||e.key==='W'){ if(!e.repeat) pressDir('w'); e.preventDefault(); }
    if(e.key==='s'||e.key==='S'){ if(!e.repeat) pressDir('s'); e.preventDefault(); }
    if(e.key==='x'||e.key==='X'){ if(!e.repeat) brake(); e.preventDefault(); }
  });
  window.addEventListener('keyup', e=>{
    if(e.key==='w'||e.key==='W') releaseDir('w');
    if(e.key==='s'||e.key==='S') releaseDir('s');
  });
  window.addEventListener('blur', releaseAll);
  document.addEventListener('visibilitychange', ()=>{ if(document.hidden) releaseAll(); });

  function bindHold(el, dir){
    el.addEventListener('pointerdown', e=>{
      e.preventDefault();
      try{ el.setPointerCapture(e.pointerId); }catch(_){}
      pressDir(dir);
    });
    ['pointerup','pointercancel'].forEach(t=>{
      el.addEventListener(t, e=>{
        try{ el.releasePointerCapture(e.pointerId); }catch(_){}
        releaseDir(dir);
      });
    });
    el.addEventListener('contextmenu', e=>e.preventDefault());
    el.addEventListener('dragstart', e=>e.preventDefault());
  }
  bindHold(els.keyW,'w');
  bindHold(els.keyS,'s');
  window.addEventListener('pointerup', releaseAll);
  window.addEventListener('pointercancel', releaseAll);

  // ---------------- TRAVAGEM ----------------
  // Diferente do E-STOP: põe o alvo a 0 RPM mas deixa o sistema armado,
  // por isso não é preciso carregar em Retomar para voltar a andar.
  let brakeFlashTimer = null;
  function brake(){
    releaseAll();                       // larga W/S e os botões de toque
    const antes = targetRpm;
    targetRpm = 0;
    updateTargetUI();
    send({cmd:'brake'});
    addLog('■ TRAVAGEM: alvo de '+antes+' para 0 RPM','t-warn');

    els.btnBrake.classList.add('fired');
    clearTimeout(brakeFlashTimer);
    brakeFlashTimer = setTimeout(()=>els.btnBrake.classList.remove('fired'), 350);
  }
  els.btnBrake.addEventListener('click', brake);

  // ---------------- LIMITES E MATRIZ ----------------
  els.btnApplyMax.addEventListener('click', ()=>{
    const v = parseInt(els.inputMaxRpm.value,10);
    if(!isNaN(v) && v>=0){
      maxRpm = v;
      targetRpm = Math.max(-maxRpm, Math.min(maxRpm, targetRpm));
      updateTargetUI();
      send({cmd:'setMaxRpm', value:v});
      addLog('[LIMITE] velocidade máxima -> '+v+' RPM','t-cmd');
    }
  });
  els.btnApplyAmps.addEventListener('click', ()=>{
    const v = parseFloat(els.inputAmps.value);
    if(!isNaN(v) && v>=0){
      send({cmd:'setAmps', value:v});
      addLog('[LIMITE] corrente -> '+v+' A','t-cmd');
    }
  });

  function sendMatrixText(){
    const t = els.inputMatrix.value.trim();
    send({cmd:'matrixText', value:t});
    addLog('[MATRIZ] texto -> "'+t+'"','t-cmd');
    els.inputMatrix.blur();
  }
  els.btnMatrix.addEventListener('click', sendMatrixText);
  els.inputMatrix.addEventListener('keydown', e=>{
    if(e.key==='Enter'){ e.preventDefault(); sendMatrixText(); }
  });

  // ---------------- GRÁFICOS ----------------
  const pinned = {};

  function togglePinned(metric){
    const m = METRICS[metric];
    if(!m) return;
    if(pinned[metric]){
      pinned[metric].cardEl.remove();
      delete pinned[metric];
      document.querySelector(`.gauge[data-metric="${metric}"]`).classList.remove('pinned');
      updatePlaceholder();
      return;
    }
    const card=document.createElement('div');
    card.className='chart-card';
    card.innerHTML=`
      <div class="chart-card-head">
        <div class="chart-card-title">${m.chartLabel}</div>
        <button class="chart-card-close">×</button>
      </div>
      <div class="chart-card-body">
        <div class="modal-stats-row">
          <div class="modal-stat"><div class="modal-stat-label">Atual</div><div class="modal-stat-value" data-stat="current">—</div></div>
          <div class="modal-stat"><div class="modal-stat-label">Mín.</div><div class="modal-stat-value" data-stat="min">—</div></div>
          <div class="modal-stat"><div class="modal-stat-label">Máx.</div><div class="modal-stat-value" data-stat="max">—</div></div>
          <div class="modal-stat"><div class="modal-stat-label">Média</div><div class="modal-stat-value" data-stat="avg">—</div></div>
        </div>
        <div class="chart-wrap"><canvas height="180"></canvas></div>
        <div class="modal-empty" style="display:none">Ainda sem dados suficientes.</div>
        <div class="modal-range-row">
          <button class="range-btn active" data-range="60">1 min</button>
          <button class="range-btn" data-range="300">5 min</button>
          <button class="range-btn" data-range="0">Tudo</button>
        </div>
      </div>`;
    els.rightCol.appendChild(card);

    pinned[metric] = {
      cardEl: card,
      canvasEl: card.querySelector('canvas'),
      emptyEl: card.querySelector('.modal-empty'),
      statEls: {
        current: card.querySelector('[data-stat="current"]'),
        min: card.querySelector('[data-stat="min"]'),
        max: card.querySelector('[data-stat="max"]'),
        avg: card.querySelector('[data-stat="avg"]'),
      },
      range: 60,
    };

    card.querySelector('.chart-card-close').addEventListener('click',()=>togglePinned(metric));
    card.querySelectorAll('.range-btn').forEach(b=>{
      b.addEventListener('click',()=>{
        card.querySelectorAll('.range-btn').forEach(x=>x.classList.remove('active'));
        b.classList.add('active');
        pinned[metric].range=parseInt(b.dataset.range,10);
        renderChart(metric);
      });
    });

    document.querySelector(`.gauge[data-metric="${metric}"]`).classList.add('pinned');
    updatePlaceholder();
    renderChart(metric);
  }

  function updatePlaceholder(){
    els.chartPlaceholder.style.display = Object.keys(pinned).length===0 ? 'block' : 'none';
  }

  document.querySelectorAll('.gauge[data-metric]').forEach(g=>{
    g.addEventListener('click',()=> togglePinned(g.dataset.metric));
  });

  function refreshPinnedCharts(){ Object.keys(pinned).forEach(renderChart); }
  setInterval(refreshPinnedCharts,1000);

  function renderChart(metric){
    const p = pinned[metric];
    if(!p) return;
    const m = METRICS[metric];
    const arr = history[metric]||[];
    const now = Date.now();
    const pts = p.range>0 ? arr.filter(x=>x.t>=now-p.range*1000) : arr;

    if(pts.length<2){
      p.emptyEl.style.display='block'; p.canvasEl.style.display='none';
      p.statEls.current.textContent = pts.length?pts[pts.length-1].v.toFixed(m.dec):'—';
      p.statEls.min.textContent='—'; p.statEls.max.textContent='—'; p.statEls.avg.textContent='—';
      return;
    }
    p.emptyEl.style.display='none'; p.canvasEl.style.display='block';
    const vals=pts.map(x=>x.v);
    const vMin=Math.min(...vals), vMax=Math.max(...vals), vAvg=vals.reduce((a,b)=>a+b,0)/vals.length;
    p.statEls.current.textContent=vals[vals.length-1].toFixed(m.dec);
    p.statEls.min.textContent=vMin.toFixed(m.dec);
    p.statEls.max.textContent=vMax.toFixed(m.dec);
    p.statEls.avg.textContent=vAvg.toFixed(m.dec);
    drawChart(p.canvasEl, pts, vMin, vMax);
  }

  function drawChart(canvas, pts, vMin, vMax){
    const dpr=window.devicePixelRatio||1;
    const cw=canvas.clientWidth||330, ch=180;
    canvas.width=cw*dpr; canvas.height=ch*dpr; canvas.style.height=ch+'px';
    const ctx=canvas.getContext('2d'); ctx.scale(dpr,dpr); ctx.clearRect(0,0,cw,ch);
    const pL=46,pR=10,pT=10,pB=22,pw=cw-pL-pR,ph=ch-pT-pB;
    let rMin=vMin,rMax=vMax; if(rMin===rMax){rMin-=1;rMax+=1;}
    const mg=(rMax-rMin)*0.12; rMin-=mg; rMax+=mg;
    const tMin=pts[0].t, tMax=pts[pts.length-1].t, tSpan=Math.max(1,tMax-tMin);
    const xF=t=>pL+((t-tMin)/tSpan)*pw;
    const yF=v=>pT+ph-((v-rMin)/(rMax-rMin))*ph;
    ctx.strokeStyle='#1a262c'; ctx.fillStyle='#61757e'; ctx.font='11px monospace';
    ctx.textAlign='right'; ctx.textBaseline='middle';
    for(let i=0;i<=3;i++){const v=rMin+(rMax-rMin)*(i/3);const y=yF(v);ctx.beginPath();ctx.moveTo(pL,y);ctx.lineTo(pL+pw,y);ctx.lineWidth=1;ctx.stroke();ctx.fillText(v.toFixed(1),pL-5,y);}
    if(rMin<0&&rMax>0){const y=yF(0);ctx.strokeStyle='#2c3f47';ctx.beginPath();ctx.moveTo(pL,y);ctx.lineTo(pL+pw,y);ctx.lineWidth=1;ctx.stroke();}
    ctx.textAlign='center'; ctx.textBaseline='top';
    function fa(t){const s=Math.round((tMax-t)/1000);if(s<1)return'agora';if(s<60)return'-'+s+'s';return'-'+Math.round(s/60)+'min';}
    ctx.fillStyle='#61757e';
    ctx.fillText(fa(tMin),xF(tMin),pT+ph+5); ctx.fillText(fa(tMax),xF(tMax),pT+ph+5);
    ctx.beginPath(); ctx.moveTo(xF(pts[0].t),yF(pts[0].v));
    for(let i=1;i<pts.length;i++)ctx.lineTo(xF(pts[i].t),yF(pts[i].v));
    ctx.lineTo(xF(pts[pts.length-1].t),pT+ph); ctx.lineTo(xF(pts[0].t),pT+ph);
    ctx.closePath(); ctx.fillStyle='rgba(25,211,230,0.10)'; ctx.fill();
    ctx.beginPath(); ctx.moveTo(xF(pts[0].t),yF(pts[0].v));
    for(let i=1;i<pts.length;i++)ctx.lineTo(xF(pts[i].t),yF(pts[i].v));
    ctx.strokeStyle='#19d3e6'; ctx.lineWidth=1.8; ctx.stroke();
    const last=pts[pts.length-1];
    ctx.beginPath(); ctx.arc(xF(last.t),yF(last.v),3.5,0,Math.PI*2); ctx.fillStyle='#19d3e6'; ctx.fill();
  }

  // ---------------- ARRANQUE ----------------
  setEstopUI(true);
  for(let c=0;c<NUM_MCU;c++) setMcuOnline(c, false);
  refreshPowerUI();
  connectWs();
})();
</script>
</body>
</html>
)rawliteral";

// ============================================================
// ENVIO DE COMANDOS ESP-NOW
// ============================================================
void sendControlCommand(char cmd, int16_t rpm = 0, float amps = 0,
                        uint8_t relayId = 0, bool relayState = false) {
  outgoingMsg.type = MSG_CONTROL;
  outgoingMsg.command = cmd;
  outgoingMsg.targetRpm = rpm;
  outgoingMsg.targetAmps = amps;
  outgoingMsg.relayId = relayId;
  outgoingMsg.relayState = relayState;
  strncpy(outgoingMsg.text, "CTRL", sizeof(outgoingMsg.text));

  esp_now_send(receiverMac, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));
  Serial.printf("[ESP-NOW TX] Comando '%c' enviado.\n", cmd);
}

void sendRelayCommand(uint8_t id, bool on) {
  sendControlCommand('L', 0, 0, id, on);
  Serial.printf("[RELE] pedido id=%u -> %s\n", id, on ? "ON" : "OFF");
}

// Envia o estado atual dos relés para o(s) browser(s)
void pushRelayStateToWs(AsyncWebSocketClient *client = nullptr) {
  StaticJsonDocument<192> doc;
  doc["type"]         = "relays";
  doc["lights"]       = (relayMask & 0x01) != 0;
  doc["dc"]           = (relayMask & 0x02) != 0;
  doc["key"]          = (relayMask & 0x04) != 0;
  doc["horn"]         = (relayMask & 0x08) != 0;
  doc["motorEnabled"] = motorArmed;
  String out;
  serializeJson(doc, out);
  if (client) client->text(out);
  else        ws.textAll(out);
}

// ============================================================
// CONSOLA DE DIAGNOSTICO
// Guarda as ultimas linhas para as repetir a quem abrir a dashboard depois,
// senao perdiam-se todas as mensagens de arranque.
// ============================================================
#define LOG_HISTORY 40
struct LogLine { uint8_t src; uint8_t level; char text[LOG_TEXT_MAX + 1]; };
LogLine logHistory[LOG_HISTORY];
int logHistCount = 0, logHistNext = 0;

const char* srcName(uint8_t src) {
  switch (src) { case SRC_ECU: return "ECU"; case SRC_CMD: return "CMD"; default: return "TCMS"; }
}

void pushLogToWs(uint8_t src, uint8_t level, const char* text, AsyncWebSocketClient *client = nullptr) {
  StaticJsonDocument<192> doc;
  doc["type"]  = "log";
  doc["src"]   = srcName(src);
  doc["level"] = level;
  doc["text"]  = text;
  String out;
  serializeJson(doc, out);
  if (client) client->text(out);
  else        ws.textAll(out);
}

void rememberLog(uint8_t src, uint8_t level, const char* text) {
  LogLine &l = logHistory[logHistNext];
  l.src = src; l.level = level;
  strncpy(l.text, text, LOG_TEXT_MAX);
  l.text[LOG_TEXT_MAX] = '\0';
  logHistNext = (logHistNext + 1) % LOG_HISTORY;
  if (logHistCount < LOG_HISTORY) logHistCount++;
}

// Mensagem gerada pelo proprio posto de comando
void cmdLog(uint8_t level, const char *fmt, ...) {
  char buf[LOG_TEXT_MAX + 1];
  va_list ap; va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.println(buf);
  rememberLog(SRC_CMD, level, buf);
  pushLogToWs(SRC_CMD, level, buf);
}

void replayLogToClient(AsyncWebSocketClient *client) {
  int start = (logHistCount < LOG_HISTORY) ? 0 : logHistNext;
  for (int i = 0; i < logHistCount; i++) {
    LogLine &l = logHistory[(start + i) % LOG_HISTORY];
    pushLogToWs(l.src, l.level, l.text, client);
  }
}

// Ultimo texto pedido para a matriz, para o browser mostrar
void pushMatrixTextToWs(AsyncWebSocketClient *client = nullptr) {
  StaticJsonDocument<128> doc;
  doc["type"] = "matrix";
  doc["text"] = currentMatrixText;
  String out;
  serializeJson(doc, out);
  if (client) client->text(out);
  else        ws.textAll(out);
}

void sendMatrixText(const char* txt) {
  // So caracteres imprimiveis ASCII: a matriz nao desenha acentos nem controlo
  int n = 0;
  for (int i = 0; txt[i] != '\0' && n < MATRIX_TEXT_MAX; i++) {
    char c = txt[i];
    if (c >= 32 && c <= 126) currentMatrixText[n++] = c;
  }
  currentMatrixText[n] = '\0';

  memset(outgoingMsg.matrixText, 0, sizeof(outgoingMsg.matrixText));
  strncpy(outgoingMsg.matrixText, currentMatrixText, MATRIX_TEXT_MAX);

  sendControlCommand('M');
  cmdLog(LVL_INFO, "[MATRIZ] Pedido de texto: \"%s\"", currentMatrixText);
  pushMatrixTextToWs();
}

// ------------------------------------------------------------
// TRAVAGEM: alvo a 0 RPM nos dois motores, sem ativar o E-STOP.
// Reutiliza o comando 'T' que o TCMS ja conhece, por isso nao e preciso
// regravar o TCMS. Vai 3 vezes seguidas: se um pacote de radio se perder,
// o motor nao pode ficar com o alvo antigo.
// ------------------------------------------------------------
void sendBrake(const char *origem) {
  for (int i = 0; i < 3; i++) sendControlCommand('T', 0, 0);
  cmdLog(LVL_WARN, "[SEGURANCA] Travagem (%s): alvo a 0 RPM nos dois motores.", origem);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    StaticJsonDocument<320> doc;
    if (deserializeJson(doc, (char*)data)) {
      cmdLog(LVL_WARN, "[DADOS] Mensagem da dashboard ilegivel (JSON invalido ou grande demais).");
      return;
    }

    const char* cmd = doc["cmd"];
    if (!cmd) return;

    if (strcmp(cmd, "setTarget") == 0)      sendControlCommand('T', doc["value"].as<int16_t>(), 0);
    else if (strcmp(cmd, "brake") == 0)     sendBrake("dashboard");
    else if (strcmp(cmd, "stop") == 0)      sendControlCommand('P');
    else if (strcmp(cmd, "resume") == 0)    sendControlCommand('R');
    else if (strcmp(cmd, "setAmps") == 0)   sendControlCommand('A', 0, doc["value"].as<float>());
    else if (strcmp(cmd, "relay") == 0) {   // DC Contactor / Key Switch / Luzes
      uint8_t id = doc["id"].as<uint8_t>();
      if (id <= RELAY_HORN) sendRelayCommand(id, doc["state"].as<bool>());
    }
    else if (strcmp(cmd, "matrixText") == 0) {   // texto da matriz de LEDs
      const char* txt = doc["value"];
      if (txt) sendMatrixText(txt);
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    pushRelayStateToWs(client);   // estado atual dos relés
    pushMatrixTextToWs(client);   // texto atual da matriz
    replayLogToClient(client);    // o que se passou antes de abrires a pagina
    cmdLog(LVL_OK, "[WIFI] Dashboard ligada (cliente #%u, %u ligados).",
           client->id(), ws.count());
  } else if (type == WS_EVT_DISCONNECT) {
    cmdLog(LVL_INFO, "[WIFI] Dashboard desligada (restam %u).", ws.count());
  } else if (type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  }
}

// ============================================================
// CALLBACKS ESP-NOW
// ============================================================
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) return;
  espNowTxFail++;
  // So avisamos de vez em quando: a 400 ms por ping, isto encheria a consola
  static unsigned long lastWarn = 0;
  if (millis() - lastWarn > 5000) {
    lastWarn = millis();
    cmdLog(LVL_WARN, "[ESP-NOW] %u envios sem confirmacao do TCMS. Distancia ou canal?", espNowTxFail);
  }
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(incomingMsg)) {
    // O erro mais traicoeiro do sistema: structs diferentes nos dois firmwares.
    static unsigned long lastWarn = 0;
    if (millis() - lastWarn > 5000) {
      lastWarn = millis();
      cmdLog(LVL_ERR, "[ESP-NOW] Pacote de %d bytes rejeitado, esperado %d. Grava TCMS e COMANDO.",
             len, (int)sizeof(incomingMsg));
    }
    return;
  }
  memcpy(&incomingMsg, data, sizeof(incomingMsg));

  // Linha de diagnostico vinda do TCMS (ou da ECU, reencaminhada por ele)
  if (incomingMsg.type == MSG_LOG) {
    rememberLog(incomingMsg.logSrc, incomingMsg.logLevel, incomingMsg.logText);
    pushLogToWs(incomingMsg.logSrc, incomingMsg.logLevel, incomingMsg.logText);
    return;
  }

  if (incomingMsg.type == MSG_TELEMETRY) {
    lastTelemetryRx = millis();
    if (!tcmsAlive) {
      tcmsAlive = true;
      cmdLog(LVL_OK, "[ESP-NOW] Telemetria do TCMS a chegar outra vez.");
    }

    // Uma mensagem por controlador: a dashboard mostra-os lado a lado
    for (int c = 0; c < NUM_MCU; c++) {
      const mcu_telemetry_t &m = incomingMsg.mcu[c];

      // 22 erros possíveis: o array sozinho já passa dos 256 bytes
      StaticJsonDocument<1024> doc;
      doc["type"]         = "mcu";
      doc["idx"]          = c;
      doc["online"]       = m.online;
      doc["busVoltage"]   = m.busVoltage;
      doc["busCurrent"]   = m.busCurrent;
      doc["phaseCurrent"] = m.phaseCurrent;
      doc["speed"]        = m.speedActual;
      doc["ctrlTemp"]     = m.ctrlTemp;
      doc["motorTemp"]    = m.motorTemp;
      doc["running"]      = m.running;
      doc["speedMode"]    = m.speedMode;

      JsonArray errors = doc.createNestedArray("errors");
      for (int i = 0; i < ERROR_COUNT; i++) {
        if ((m.errRaw >> i) & 0x01) errors.add(ERROR_NAMES[i]);
      }

      String out;
      serializeJson(doc, out);
      ws.textAll(out);
    }

    // Estado REAL dos relés, confirmado pelo TCMS
    relayMask  = incomingMsg.relays;
    motorArmed = incomingMsg.motorEnabled;

    // Bateria / Smart Shunt
    StaticJsonDocument<192> batt;
    batt["type"]    = "batt";
    batt["valid"]   = incomingMsg.battValid;
    batt["voltage"] = incomingMsg.battVoltage;
    batt["current"] = incomingMsg.battCurrent;
    batt["power"]   = incomingMsg.battPower;
    batt["soc"]     = incomingMsg.battSoc;

    String outB;
    serializeJson(batt, outB);
    ws.textAll(outB);
    pushRelayStateToWs();
  }
}

void handleSerialInput() {
  if (Serial.available() > 0) {
    char key = toupper(Serial.read());
    // W/S ja nao mexem nas luzes: o TCMS comuta-as pelo sinal das RPM.
    // Ficam aqui L (toggle manual), D, K, B e P.
    if      (key == 'L') sendRelayCommand(RELAY_LIGHTS, !(relayMask & 0x01));
    else if (key == 'D') sendRelayCommand(RELAY_DC,  !(relayMask & 0x02));
    else if (key == 'K') sendRelayCommand(RELAY_KEY, !(relayMask & 0x04));
    else if (key == 'B') sendRelayCommand(RELAY_HORN, true);   // buzina
    else if (key == 'P') sendControlCommand('P');
    else if (key == 'X') sendBrake("monitor serie");
  }
}

// ============================================================
// SETUP & LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Modo misto: Access Point (para o telemóvel/PC) + Station (para ESP-NOW)
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("EZkontrol_Monitor", "12345678", WIFI_CHANNEL);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] FALHA ao iniciar. Reinicia a placa.");
    while (true) delay(1000);
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    cmdLog(LVL_ERR, "[ESP-NOW] Nao foi possivel registar o TCMS como par. MAC errado?");
  }

  // Configuração do Servidor Web
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();

  Serial.println("\nSender Pronto. IP do Monitor: 192.168.4.1");

  // Estas linhas ficam no historico e aparecem assim que a dashboard abrir
  cmdLog(LVL_OK, "[SISTEMA] Posto de comando pronto. Rede EZkontrol_Monitor, IP 192.168.4.1");
  cmdLog(LVL_INFO, "[ESP-NOW] Canal %d. Mensagem de %d bytes. TCMS esperado em %02X:%02X:%02X:%02X:%02X:%02X",
         WIFI_CHANNEL, (int)sizeof(espnow_message_t),
         receiverMac[0], receiverMac[1], receiverMac[2],
         receiverMac[3], receiverMac[4], receiverMac[5]);
}

void loop() {
  ws.cleanupClients(); // Mantém o WebSocket estável
  handleSerialInput();

  if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = millis();
    outgoingMsg.type = MSG_PING;
    esp_now_send(receiverMac, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));
  }

  // O TCMS devia mandar telemetria de 200 em 200 ms. Se calar, avisa.
  if (tcmsAlive && millis() - lastTelemetryRx > TCMS_SILENT_MS) {
    tcmsAlive = false;
    cmdLog(LVL_ERR, "[ESP-NOW] Sem telemetria do TCMS ha %d ms. Desligado, fora de alcance ou noutro canal.",
           TCMS_SILENT_MS);
  }
}
