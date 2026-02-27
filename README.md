# controller_server

Paquete ROS 2 (Humble, `ament_python`) para el control de actuadores del robot (motor, freno y direccion).  
Este repositorio corre en el workspace Docker del proyecto sobre Raspberry Pi 5.

## Estado actual

Hoy conviven dos capas:

1. Esqueleto ROS 2 del paquete `controller_server` (raiz del repo).
2. Implementacion funcional de comunicaciones UART v2 en:
   - `controller_server/controller/src/rpy_esp32_comms`

Importante: actualmente no hay `console_scripts` ROS2 publicados en `setup.py`, por lo que el flujo operativo principal es via la CLI Python del modulo UART.

## Arquitectura funcional (UART v2)

- Transporte: `/dev/serial0` a `115200` baud.
- TX Pi -> ESP32: continuo (default `50 Hz`).
- RX ESP32 -> Pi: telemetria con parser incremental y validacion CRC.
- Seguridad:
  - `safe_reset()` al iniciar.
  - 3 frames seguros al detener.
  - clamps de comando (`speed`, `steer`, `brake`).

Documentacion tecnica del protocolo:
- `controller_server/controller/COMUNICACIONES_UART_V2.md`

## Estructura relevante

- `setup.py`, `package.xml`: metadata del paquete ROS2 (pendiente de completar).
- `controller_server/controller/src/rpy_esp32_comms/controller.py`: estado de comando y limites.
- `controller_server/controller/src/rpy_esp32_comms/protocol.py`: encode/decode de frames + CRC + parser.
- `controller_server/controller/src/rpy_esp32_comms/transport.py`: `CommsClient` con hilos TX/RX.
- `controller_server/controller/src/rpy_esp32_comms/cli.py`: interfaz REPL para pruebas/manual.
- `controller_server/controller/tests/`: tests unitarios del protocolo y estado.

## Uso rapido (dentro de Docker ROS2)

Desde la raiz del workspace (`/home/gmorini/Documentos/codigo/ros2/workspace`):

```bash
docker compose up -d --build
./tools/compile-ros.sh controller_server
./tools/exec.sh
```

Ya dentro del contenedor:

```bash
cd /ros2_ws/src/controller_server/controller_server/controller
python3 -m pip install -r requirements.txt
PYTHONPATH=src python3 -m rpy_esp32_comms --port /dev/serial0 --baud 115200 --tx-hz 50
```

## Comandos de la CLI

- `help`
- `status`
- `drive on|off`
- `estop on|off`
- `speed <mps>`
- `steer <int -100..100>`
- `brake <0..100>`
- `watch on|off`
- `log on|off`
- `quit`

## Testing

```bash
cd controller_server/controller
PYTHONPATH=src python3 -m pytest -q
```

Si falla por dependencias, instala primero:

```bash
python3 -m pip install -r requirements.txt
```

## Integracion pendiente como nodo ROS2

Para cerrar este paquete como `controller_server` de produccion ROS2, faltan (o deben validarse) al menos:

1. Exponer `console_scripts` ROS2 en `setup.py`.
2. Definir nodo(s) ROS2 de entrada/salida (topics y tipos de mensaje).
3. Completar metadata de `package.xml` (`description`, `license`, maintainers reales).
4. Conectar explícitamente el pipeline de comandos seguros del stack de navegacion.

## Referencias

- `AGENTS.md` (guia operativa para agentes/cambios en este repo).
- README del modulo UART:
  - `controller_server/controller/README.md`
