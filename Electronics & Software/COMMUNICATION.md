# Communication overview

| Link | What travels across it | Timing in firmware |
| --- | --- | --- |
| Browser ↔ COMANDO | Operator requests; live measurements, states and logs | HTTP port 80; JSON over WebSocket `/ws` |
| COMANDO → TCMS | Speed, current limit, relays, stop/resume, display text | ESP-NOW channel 1, on request; ping every 2 s |
| TCMS → COMANDO | Motor/battery telemetry and logs | Telemetry every 200 ms once a radio sender is known; logs as queued |
| TCMS ↔ motor controllers | Handshake, speed/current/start commands; motor telemetry | CAN at 250 kbit/s; commands every 50 ms while the session is running |
| SmartShunt → ECU | Battery voltage, current, power and state of charge | UART at 19200 baud, as data arrives |
| ECU → TCMS | Battery measurements and diagnostics | Battery frames after an accepted UART block updates a useful field |
| TCMS → ECU | Relay/stop state and matrix text | State on change or every 200 ms; text on change or every 3 s |

Intervals are scheduling targets, without measured timing guarantees. ECU and COMANDO exchange information through TCMS.

## How a request reaches the locomotive

A browser action reaches COMANDO over WebSocket. COMANDO converts it to an ESP-NOW request. TCMS applies the request to relay outputs, motor targets or display text. Motor and battery information returns through TCMS and COMANDO to the browser. ECU receives display text in short CAN chunks and assembles it before showing it.

## Stop, brake and connection loss

- **STOP:** TCMS zeros speed targets and clears the motor start bit through a software stop latch. DC and key relay commands stay unchanged. ECU displays `PARAR` when it receives the operator-stop state.
- **Resume:** clears the stop latch only with DC and key software states on.
- **Brake:** COMANDO sends zero-speed requests three times. The current limit and permission to start remain unchanged.
- **Motor telemetry loss:** after more than 500 ms, TCMS stops periodic commands to that controller and waits for another handshake. Stored targets remain available after reconnection.
- **Browser or radio loss:** does not automatically stop traction. TCMS can continue sending the last target while motor telemetry remains available. Releasing a direction key also retains the last target.

