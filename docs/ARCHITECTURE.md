[![中文](https://img.shields.io/badge/中文-文档-blue)](ARCHITECTURE.zh.md)

# Architecture

## Layers

| Layer | Main paths | Responsibility |
| --- | --- | --- |
| Ai-Thinker applications | `Ai-Demo/` | PB-03F product demos, board configuration, AT commands, and application events |
| SDK examples | `example/` | BLE roles, Mesh, OTA, peripheral demonstrations, and independent build projects |
| Profiles and services | `components/profiles/` | GAP roles, device information, OTA, proprietary services, and application profiles |
| Mesh integration | `components/ethermind/` | EtherMind models, bearer/platform adapters, crypto integration, utilities, and prebuilt libraries |
| BLE and scheduling | `components/ble/`, `components/osal/` | BLE interfaces, OSAL tasks/events, power management, and storage |
| Drivers | `components/driver/` | Clock, GPIO, UART, ADC, flash, power, timer, and other MCU peripherals |
| Source libraries | `components/libraries/` | TinyCrypt, command-line support, storage helpers, and reusable modules |
| Prebuilt platform code | `lib/` and component `lib/` directories | Vendor controller, host, Mesh, and security archives |

All paths in this document are relative to `Ai_PB-03F_OPEN-SOURCE` unless stated otherwise.

## Control and data flow

An application initializes the MCU and RF/PHY, registers OSAL tasks, then transfers control to the OSAL event loop. BLE, Mesh, timers, UART, and peripheral callbacks post events or messages. The application task consumes those events and calls profiles, services, and drivers.

For the representative Ai-Thinker demo, UART data and AT commands are handled by the BLE UART application layer; GAP/GATT services pass BLE events into the same OSAL task. ADC behavior is selected by compile-time run-mode macros.

## Project boundaries

- Each of the 43 `.uvprojx` files is an independent firmware composition. Its sources, includes, macros, libraries, compiler version, and scatter file determine the actual image.
- The GNU Makefile under `example/ble_peripheral/simpleBlePeripheral/gcc` is a separate supported composition and does not prove that Keil-only projects build with GCC.
- Changes under `components/` can affect many examples. Run the repository reference validator after moving or renaming shared files.
- Prebuilt archives cannot be reconstructed completely from this repository. Preserve their ABI and verify toolchain compatibility before replacing them.
- Checked-in AXF, HEX, MAP, object, and build-log files are historical evidence, not proof of a current clean build.

## Dependency direction

```text
application / example
  -> profiles and services
     -> BLE host / Mesh integration
        -> OSAL and drivers
           -> MCU hardware and prebuilt vendor libraries
```

Callbacks in the reverse direction normally post messages/events or invoke registered handlers; application work then runs in the relevant OSAL task context.
