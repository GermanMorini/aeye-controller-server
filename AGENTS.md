# AGENTS.md - controller_server

## Objetivo del paquete
`controller_server` es el paquete destinado al control de actuadores (motor/freno/direccion) del robot autonomo. En este estado del repo conviven:
- un esqueleto de paquete ROS 2 `ament_python` (raiz del repo), y
- una implementacion funcional UART v2 en `controller_server/controller/` (libreria `rpy_esp32_comms`).

## Contexto de plataforma
- Robot: cuatriciclo autonomo con direccion Ackermann.
- Compute: Raspberry Pi 5.
- Sensores relevantes para la navegacion global: LiDAR + GPS (sin SLAM).
- ROS 2: Humble, workflow principal dentro de Docker (`/ros2_ws`).
- Workspace local host: `/home/gmorini/Documentos/codigo/ros2/workspace`.

## Estado real del repositorio (importante)
- `setup.py` y `package.xml` aun tienen metadata placeholder (`TODO`), pero ya exponen el ejecutable ROS2 `controller_server_node`.
- El codigo de control activo vive en:
  - `controller_server/rpy_esp32_comms/*.py`
- Tests funcionales del protocolo UART v2 viven en:
  - `controller_server/controller/tests/`
- En este checkout, `controller_server/controller/` aparece como contenido no trackeado en git (`git status --short`). Tratarlo como parte critica del trabajo actual antes de limpiar/mover archivos.

## Mapa rapido de codigo
- `controller_server/rpy_esp32_comms/controller.py`
  - `CommandState` con clamps de seguridad (`speed`, `steer`, `brake`) y `safe_reset()`.
- `controller_server/rpy_esp32_comms/protocol.py`
  - framing UART v2, `crc8_maxim`, encode/decode, parser incremental robusto (`EspFrameParser`).
- `controller_server/rpy_esp32_comms/transport.py`
  - `CommsClient` con hilos TX/RX, locks, estadisticas, apertura de `/dev/serial0`.
- `controller_server/rpy_esp32_comms/telemetry.py`
  - estructura `Telemetry`, decode de `status_flags`, `ControlSource`.
- `controller_server/rpy_esp32_comms/cli.py`
  - REPL manual (`drive`, `estop`, `speed`, `steer`, `brake`, `status`, `watch`, `log`, `quit`).
- `controller_server/controller/COMUNICACIONES_UART_V2.md`
  - contrato del protocolo y semantica de seguridad.

## Contrato de protocolo (no romper)
- Pi -> ESP32: 7 bytes, header `0xAA`, version en nibble alto (`2`), CRC-8 Dallas/Maxim.
- ESP32 -> Pi: 8 bytes, header `0x55`, CRC-8 Dallas/Maxim.
- Sentinels RX:
  - `speed_raw == 0xFFFF` -> `speed_mps = None`
  - `steer_raw == -32768` -> `steer_deg = None`
- Flags de comando actuales:
  - bit0: `ESTOP`
  - bit1: `DRIVE_EN`

Si cambias frame layout, version, CRC o flags, debes actualizar **en el mismo cambio**:
1. `protocol.py`
2. `COMUNICACIONES_UART_V2.md` y `README.md`
3. tests en `controller/tests/test_protocol.py`
4. firmware ESP32 compatible (coordinar fuera de este repo)

## Invariantes de seguridad
- Al iniciar `CommsClient.start()`, estado deseado se resetea a seguro.
- Al detener `CommsClient.stop()`, se envian 3 frames seguros antes de cerrar UART.
- TX Pi->ESP32 es periodico (default `50 Hz`); no introducir envios event-driven unicos sin fallback periodico.
- Mantener clamps:
  - `steer_pct` en `[-100, 100]`
  - `brake_pct` en `[0, 100]`
  - `speed_mps` en `[0, max_speed_mps]`

## Flujo de trabajo recomendado
### Dentro del workspace Docker ROS2
Desde `/home/gmorini/Documentos/codigo/ros2/workspace`:
1. `docker compose up -d --build`
2. `./tools/compile-ros.sh controller_server`
3. `./tools/exec.sh`

### Ejecutar la CLI UART v2
Dentro del contenedor (o en host con entorno equivalente):
1. `cd /ros2_ws/src/controller_server`
2. `python3 -m pip install -r controller_server/controller/requirements.txt`
3. `python3 -m controller_server.rpy_esp32_comms --port /dev/serial0 --baud 115200 --tx-hz 50`

## Testing
- Unit tests del modulo UART:
  - `cd controller_server/controller`
  - `PYTHONPATH=../.. python3 -m pytest -q`
- En este entorno host actual faltan dependencias de test (`pytest` no instalado por defecto). Ejecutar en contenedor o venv antes de validar.

## Reglas para cambios de agentes
- No asumir que este paquete ya publica nodos ROS2: verificar `entry_points` antes de proponer `ros2 run controller_server ...`.
- Si agregas integracion ROS2 (suscriptor `cmd_vel`/publisher estado), declarar explicitamente:
  - nombres de topicos,
  - tipo de mensaje,
  - frecuencia de publicacion,
  - politica de safety al perder comandos.
- Evitar cambios de concurrencia sin tests (hilos TX/RX + locks en `transport.py`).
- No borrar artefactos de evidencia (`session*.jsonl`, `artifacts/*.json`) salvo pedido explicito.

## Diagnostico rapido en campo
- Ver permisos de puerto:
  - `ls -l /dev/serial0`
  - `groups`
- En REPL:
  - `status` para `desired`, ultima telemetria y contadores (`tx_ok`, `rx_ok`, `rx_crc`, `rx_drop`).
  - `watch on` para telemetria continua.
- Si no llega RX:
  - revisar cableado TX/RX/GND,
  - revisar baudios,
  - confirmar firmware ESP32 en protocolo v2,
  - confirmar que ningun otro proceso usa el puerto.

## Integracion con el resto del stack
Este repo vive junto a `navegacion_gps`, `sensores`, `rslidar_sdk`, `map_tools` en `workspace/src`. Cualquier puente hacia navegacion debe respetar el pipeline de seguridad del sistema (comandos filtrados/seguros antes de actuadores).
