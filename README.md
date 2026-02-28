# controller_server

Paquete ROS 2 (Humble, `ament_python`) para el control de actuadores del robot (motor, freno y direccion).  
Este repositorio corre en el workspace Docker del proyecto sobre Raspberry Pi 5.

## Estado actual

El paquete ya incluye un nodo ROS2 funcional:

1. `controller_server_node` (nodo ROS2 principal).
2. Libreria UART v2 integrada en:
   - `controller_server/rpy_esp32_comms`
3. Carpeta `controller_server/controller/` conservada para pruebas, artefactos y documentación histórica.

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

- `controller_server/controller_server_node.py`: nodo ROS2, WebSocket y arbitraje.
- `controller_server/control_logic.py`: mapeo `/cmd_vel_safe` -> comando y selección `manual/auto`.
- `controller_server/rpy_esp32_comms/*.py`: transporte UART, protocolo, telemetria.
- `launch/controller_server.launch.py`: launch por defecto.
- `test/test_control_logic.py`: tests de mapeo y arbitraje.
- `controller_server/controller/tests/`: tests del protocolo UART original.

## Interfaces del nodo

### Suscripciones

- `/cmd_vel_safe` (`geometry_msgs/msg/Twist`)

### Publicaciones

- `/controller/status` (`std_msgs/msg/String`, JSON)
- `/controller/telemetry` (`std_msgs/msg/String`, JSON)

### WebSocket

- Endpoint por defecto: `ws://0.0.0.0:8765`
- Payload JSON soportado:
  - `mode`: `"manual"` o `"auto"`
  - `estop`: `true/false` (global)
  - `drive`: `true/false`
  - `speed_mps`: `float`
  - `steer_pct`: `int [-100,100]`
  - `brake_pct`: `int [0,100]`
  - `cmd_estop`: `true/false` (solo comando manual)

## Parámetros principales

- `serial_port` (`/dev/serial0`)
- `serial_baud` (`115200`)
- `serial_tx_hz` (`50.0`)
- `mode` (`auto` o `manual`)
- `manual_timeout_s` / `auto_timeout_s`
- `max_speed_mps`
- `max_abs_angular_z` (default `0.4`, alineado con `wz_max`/`wz_min` de navegación)
- `invert_steer_from_cmd_vel` (invierte signo de `angular.z` al mapear a dirección)
- `reverse_brake_pct`
- `ws_enabled`, `ws_host`, `ws_port`

## Ejecutar en Docker ROS2

Desde la raiz del workspace (`/home/gmorini/Documentos/codigo/ros2/workspace`):

```bash
docker compose up -d --build
./tools/compile-ros.sh controller_server
./tools/exec.sh "source /ros2_ws/install/setup.bash && ros2 launch controller_server controller_server.launch.py"
```

Ejecutar directo:

```bash
./tools/exec.sh "source /ros2_ws/install/setup.bash && ros2 run controller_server controller_server_node"
```

## Probar con teleop_twist_keyboard

Publicar hacia `/cmd_vel_safe`:

```bash
./tools/exec.sh "source /ros2_ws/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_safe"
```

Ver comando efectivo aplicado:

```bash
./tools/exec.sh "source /ros2_ws/install/setup.bash && ros2 topic echo /controller/status"
```

## Logs de comandos entrantes

El nodo loggea explícitamente:
- cada mensaje recibido por `/cmd_vel_safe` y su mapeo a comando interno,
- cada payload recibido por WebSocket y la respuesta/estado aplicado.

## Testing

Tests de lógica del nodo:

```bash
cd /home/gmorini/Documentos/codigo/ros2/workspace/src/controller_server
python3 -m pytest -q test/test_control_logic.py
```

Tests del módulo UART:

```bash
cd controller_server/controller
PYTHONPATH=../.. python3 -m pytest -q
```

Si falla por dependencias, instala primero:

```bash
python3 -m pip install -r requirements.txt
```

## Referencias

- `AGENTS.md` (guia operativa para agentes/cambios en este repo).
- README del modulo UART:
  - `controller_server/controller/README.md`
