# LoRa Mesh Network

A LoRa-based mesh networking implementation for embedded systems using the Zephyr RTOS. This project enables low-power, long-range wireless sensor networks with automatic mesh routing and multi-hop message delivery.

## Notes
This project is very much WIP and not complete. I wrote it for fun and continue to improve and experiment with it.

## Features

- **Multi-hop Mesh Routing**: Automatic routing of messages through intermediate nodes to reach distant destinations
- **Adaptive Reliability-based Routing**: Route selection based on signal strength, reliability metrics, and hop count
- **Neighbor Discovery**: Automatic detection and maintenance of neighboring nodes
- **Sensor Integration**: Support for I2C sensors with automatic detection and data broadcasting
- **Dual Operating Modes**:
  - Active sensor node: Collects and broadcasts sensor data
  - Relay-only mode: Forwards messages when no sensors are detected
- **Efficient Broadcasting**: Single transmission to all neighbors with individual ACKs
- **Dynamic Node Addressing**: Hardware-based unique node IDs from device serial number

## Supported Hardware

### Primary Target
- **PureEngineering Feather STM32WL** (feather_stm32wl)
  - STM32WL55xx SoC with integrated LoRa radio
  - Sub-GHz LoRa communication (915 MHz default for US)

### Additional Targets
- **Raspberry Pi Pico 2** (rpi_pico2/rp2350a/m33)
  - Requires external LoRa module (SX127x compatible)

## Requirements

- [Zephyr RTOS](https://github.com/zephyrproject-rtos/zephyr) v4.2.1
- West build tool
- CMake 3.20.0 or newer
- ARM GCC toolchain

## Getting Started

### 1. Install Zephyr

Follow the official [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) to set up your development environment.

### 2. Clone the Project

```bash
git clone <repository-url>
cd lora_project
```

### 3. Initialize West Workspace

```bash
west init -l .
west update
west zephyr-export
```

### 4. Build the Firmware

For the Adafruit Feather STM32WL:
```bash
west build -p auto -b feather_stm32wl .
```

For Raspberry Pi Pico 2:
```bash
west build -b rpi_pico2/rp2350a/m33 .
```

### 5. Flash the Firmware

```bash
west flash
```

## Configuration

### LoRa Radio Parameters

Configure LoRa parameters in [prj.conf](prj.conf) or via Kconfig:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `CONFIG_APP_LORA_FREQUENCY` | 915000000 | Frequency in Hz (915 MHz US, 868 MHz EU, 433 MHz) |
| `CONFIG_APP_LORA_BANDWIDTH` | 0 | Bandwidth (0=125kHz, 1=250kHz, 2=500kHz) |
| `CONFIG_APP_LORA_SPREADING_FACTOR` | 7 | Spreading factor (5-12, higher=longer range) |
| `CONFIG_APP_LORA_CODING_RATE` | 1 | Coding rate (1=4/5, 2=4/6, 3=4/7, 4=4/8) |
| `CONFIG_APP_LORA_TX_POWER` | 14 | Transmit power in dBm |
| `CONFIG_APP_LORA_PREAMBLE_LEN` | 8 | Preamble length |

### Application Settings

- **Sensor Broadcast Interval**: 10 seconds (configurable in [main.c](src/main.c#L33))
- **Main Stack Size**: 4096 bytes
- **System Workqueue Stack**: 2048 bytes

## Architecture

### Core Components

- **[main.c](src/main.c)**: Application entry point and initialization
- **[mesh.c](src/mesh.c)**: Mesh network management and routing logic
- **[lora.c](src/lora.c)**: LoRa radio driver interface
- **[packet.c](src/packet.c)**: Packet processing and serialization
- **[sensor.c](src/sensor.c)**: Sensor abstraction and data collection
- **[led.c](src/led.c)**: LED status indication
- **[work_queue.c](src/work_queue.c)**: Consolidated work queue management

### Supporting Modules

- **[mesh_handlers.c](src/mesh_handlers.c)**: Mesh packet type handlers
- **[mesh_utils.c](src/mesh_utils.c)**: Mesh utility functions
- **[neighbor_hash.c](src/neighbor_hash.c)**: Neighbor table hash implementation
- **[ring_buffer.c](src/ring_buffer.c)**: Ring buffer for packet queuing

## How It Works

1. **Initialization**: Each node generates a unique address from its hardware device ID
2. **Neighbor Discovery**: Nodes periodically broadcast discovery packets
3. **Routing Table Building**: Nodes learn routes to other nodes through discovery replies
4. **Data Transmission**:
   - Direct transmission to neighbors
   - Multi-hop routing for distant nodes
   - Reliability-based path selection
5. **Sensor Broadcasting**: Nodes with sensors periodically broadcast data to all neighbors
6. **Message Relaying**: All nodes forward packets destined for other nodes

## Development

### Project Structure

```
lora_project/
├── src/                    # Source files
├── include/                # Header files
├── boards/                 # Board-specific configurations
│   └── st/feather_stm32wl/
├── CMakeLists.txt         # Build configuration
├── prj.conf               # Project configuration
├── Kconfig                # Kconfig options
└── west.yml               # West manifest
```

### Logging

Enable logging output via serial console (115200 baud) to monitor:
- Node initialization and addressing
- Neighbor discovery events
- Packet transmission and reception
- Routing decisions
- Sensor readings

## License

Copyright (c) 2024 Ryan Grachek

## Troubleshooting

### No Sensors Detected
The node will operate in relay-only mode. This is normal for nodes without I2C sensors connected.

### Node Address is Zero
A fallback address of 0x00000001 will be used. Check hardware info driver support for your board.

### Build Failures
Ensure all Zephyr dependencies are installed and west workspace is properly initialized:
```bash
west update
west zephyr-export
pip install -r deps/zephyr/scripts/requirements.txt
```
