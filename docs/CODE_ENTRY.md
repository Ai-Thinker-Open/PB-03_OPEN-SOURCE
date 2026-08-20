[![中文](https://img.shields.io/badge/中文-文档-blue)](CODE_ENTRY.zh.md)

# Code entry

This document traces the real firmware path of `Ai-Demo/BASE/bleUart_AT_ADC`. Other examples use the same startup/OSAL pattern but register different tasks.

## Build entry

- Keil project: `Ai_PB-03F_OPEN-SOURCE/Ai-Demo/BASE/bleUart_AT_ADC/bleUart_AT.uvprojx`
- CPU declared by the project: ARM Cortex-M0 (`ARMCM0`)
- Recorded compiler: ARM Compiler 5.06 update 6 build 750
- Scatter file: `Ai-Demo/BASE/bleUart_AT_ADC/scatter_load.sct`
- Main source: `Ai-Demo/BASE/bleUart_AT_ADC/Source/main.c`
- Configured output: `bleuart.axf`; post-build commands generate BIN and HEX images

## Runtime path

1. `main()` in `Source/main.c` configures clocks, flash cache, interrupts, RF/PHY, drivers, and logging.
2. `main()` calls `app_main()` in `Source/bleuart_Main.c`.
3. `app_main()` initializes OSAL, selects battery power management, and enters `osal_start_system()`.
4. OSAL initialization calls `osalInitTasks()` in `Source/OSAL_bleuart.c`.
5. `osalInitTasks()` registers protocol/profile tasks and calls `bleuart_Init()` for the application task.
6. The OSAL scheduler dispatches application events to `bleuart_ProcessEvent()` in `Source/bleuart.c`.

```text
Reset/startup
  -> main()
     -> clock + RF/PHY + drivers
     -> app_main()
        -> osal_init_system()
           -> osalInitTasks()
              -> bleuart_Init()
        -> osal_start_system()
           -> bleuart_ProcessEvent()
```

## Where to begin a change

| Goal | Start here |
| --- | --- |
| Board, clock, RF, and driver initialization | `Ai-Demo/BASE/bleUart_AT_ADC/Source/main.c` |
| Task registration and ordering | `Ai-Demo/BASE/bleUart_AT_ADC/Source/OSAL_bleuart.c` |
| BLE UART initialization and event handling | `Ai-Demo/BASE/bleUart_AT_ADC/Source/bleuart.c` |
| AT command behavior | `Ai-Demo/BASE/bleUart_AT_ADC/Source/bleuart_at_cmd.c` |
| UART/BLE data bridge | `Ai-Demo/BASE/bleUart_AT_ADC/Source/bleuart_at_dma.c` and `bleuart_protocol.c` |
| ADC run modes | `Ai-Demo/BASE/bleUart_AT_ADC/Source/adc_*demo.c` |

Do not assume this representative path is the entry for every example. Open the selected `.uvprojx` and follow its included `main.c`, OSAL task table, and application event handler.

## Build-verified GNU example

The clean ARM GNU builds documented in [Validation](VALIDATION.md) use `example/ble_peripheral/simpleBlePeripheral/gcc/Makefile`. That Makefile directly compiles the following entry chain and links it into `output/sbp.elf`:

1. `main()` in `example/ble_peripheral/simpleBlePeripheral/main.c` initializes the platform and calls `app_main()`.
2. `app_main()` in `source/SimpleBLEPeripheral_Main.c` initializes OSAL and starts its scheduler.
3. `osalInitTasks()` in `source/OSAL_SimpleBLEPeripheral.c` registers the task table and calls `SimpleBLEPeripheral_Init()`.
4. OSAL dispatches application events to `SimpleBLEPeripheral_ProcessEvent()` in `source/simpleBLEPeripheral.c`.

```text
main()
  -> app_main()
     -> osal_init_system()
        -> osalInitTasks()
           -> SimpleBLEPeripheral_Init()
     -> osal_start_system()
        -> SimpleBLEPeripheral_ProcessEvent()
```

This GNU path has compiler and linked-artifact evidence. It is a generic SDK example, not the Ai-Thinker BLE UART/AT/ADC application described above; the two paths must not be treated as the same firmware composition.
