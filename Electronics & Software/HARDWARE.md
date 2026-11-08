# Hardware ports


| Board | GPIO | Use |
| --- | --- | --- |
| TCMS | 4 / 5 | CAN TX / RX, 250 kbit/s |
| TCMS | 17 | DC contactor relay output |
| TCMS | 8 | Key-switch relay output |
| TCMS | 7 | Lights relay output |
| TCMS | 15 | Horn relay output; 250 ms pulse |
| ECU | 5 / 6 | CAN TX / RX, 250 kbit/s |
| ECU | 16 | UART2 RX from SmartShunt |
| ECU | 17 | UART2 TX configured, but no data transmitted |
| ECU | 4 | Data output to 32 × 8 NeoPixel matrix |
| COMANDO | — | No application GPIO inputs or outputs configured |

- **TCMS relays:** firmware writes LOW for ON and HIGH for OFF. Reported states are software commands, without contact feedback.
- **SmartShunt UART:** 19200 baud, 8N1, inverted input enabled for the configured optocoupler interface.
- **LED matrix:** GRB, 800 kHz, column zigzag layout, brightness 60/255.
- **Wireless:** COMANDO and TCMS use ESP-NOW on channel 1. COMANDO also provides browser Wi-Fi, HTTP port 80 and WebSocket `/ws`.
- **Serial:** all three sketches configure `Serial` at 115200 baud. COMANDO also accepts control commands there.


