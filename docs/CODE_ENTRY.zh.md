[![English](https://img.shields.io/badge/English-Docs-green)](CODE_ENTRY.md)

# 代码入口

本文跟踪 `Ai-Demo/BASE/bleUart_AT_ADC` 的真实固件运行路径。其他示例通常采用相同的启动和 OSAL 模式，但注册的任务不同。

## 构建入口

- Keil 工程：`Ai_PB-03F_OPEN-SOURCE/Ai-Demo/BASE/bleUart_AT_ADC/bleUart_AT.uvprojx`
- 工程声明的 CPU：ARM Cortex-M0（`ARMCM0`）
- 工程记录的编译器：ARM Compiler 5.06 update 6 build 750
- 分散加载文件：`Ai-Demo/BASE/bleUart_AT_ADC/scatter_load.sct`
- 主入口源码：`Ai-Demo/BASE/bleUart_AT_ADC/Source/main.c`
- 配置输出：`bleuart.axf`，构建后命令继续生成 BIN 和 HEX 镜像

## 运行路径

1. `Source/main.c` 中的 `main()` 配置时钟、Flash Cache、中断、RF/PHY、驱动和日志。
2. `main()` 调用 `Source/bleuart_Main.c` 中的 `app_main()`。
3. `app_main()` 初始化 OSAL、选择电池电源管理并进入 `osal_start_system()`。
4. OSAL 初始化过程调用 `Source/OSAL_bleuart.c` 中的 `osalInitTasks()`。
5. `osalInitTasks()` 注册协议和 Profile 任务，并为应用任务调用 `bleuart_Init()`。
6. OSAL 调度器把应用事件分派给 `Source/bleuart.c` 中的 `bleuart_ProcessEvent()`。

```text
复位/启动
  -> main()
     -> 时钟 + RF/PHY + 驱动
     -> app_main()
        -> osal_init_system()
           -> osalInitTasks()
              -> bleuart_Init()
        -> osal_start_system()
           -> bleuart_ProcessEvent()
```

## 修改从哪里开始

| 目标 | 起始位置 |
| --- | --- |
| 开发板、时钟、RF 和驱动初始化 | `Ai-Demo/BASE/bleUart_AT_ADC/Source/main.c` |
| 任务注册和顺序 | `Ai-Demo/BASE/bleUart_AT_ADC/Source/OSAL_bleuart.c` |
| BLE UART 初始化和事件处理 | `Ai-Demo/BASE/bleUart_AT_ADC/Source/bleuart.c` |
| AT 指令行为 | `Ai-Demo/BASE/bleUart_AT_ADC/Source/bleuart_at_cmd.c` |
| UART/BLE 数据桥 | `Ai-Demo/BASE/bleUart_AT_ADC/Source/bleuart_at_dma.c` 和 `bleuart_protocol.c` |
| ADC 运行模式 | `Ai-Demo/BASE/bleUart_AT_ADC/Source/adc_*demo.c` |

不要把这一代表性路径当成所有示例的入口。应打开所选 `.uvprojx`，再跟踪它实际包含的 `main.c`、OSAL 任务表和应用事件处理函数。

## 已通过构建验证的 GNU 示例

[验证文档](VALIDATION.zh.md)记录的 ARM GNU 干净构建使用 `example/ble_peripheral/simpleBlePeripheral/gcc/Makefile`。该 Makefile 直接编译以下入口链，并将其链接到 `output/sbp.elf`：

1. `example/ble_peripheral/simpleBlePeripheral/main.c` 中的 `main()` 初始化平台并调用 `app_main()`。
2. `source/SimpleBLEPeripheral_Main.c` 中的 `app_main()` 初始化 OSAL 并启动调度器。
3. `source/OSAL_SimpleBLEPeripheral.c` 中的 `osalInitTasks()` 注册任务表并调用 `SimpleBLEPeripheral_Init()`。
4. OSAL 把应用事件分派给 `source/simpleBLEPeripheral.c` 中的 `SimpleBLEPeripheral_ProcessEvent()`。

```text
main()
  -> app_main()
     -> osal_init_system()
        -> osalInitTasks()
           -> SimpleBLEPeripheral_Init()
     -> osal_start_system()
        -> SimpleBLEPeripheral_ProcessEvent()
```

这条 GNU 路径具有编译与链接产物证据。它是通用 SDK 示例，不是上文的安信可 BLE UART/AT/ADC 应用；不能把两条路径视为同一套固件组合。
