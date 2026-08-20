[![English](https://img.shields.io/badge/English-Docs-green)](ARCHITECTURE.md)

# 架构说明

## 分层

| 层级 | 主要路径 | 职责 |
| --- | --- | --- |
| 安信可应用 | `Ai-Demo/` | PB-03F 产品 Demo、板级配置、AT 指令和应用事件 |
| SDK 示例 | `example/` | BLE 角色、Mesh、OTA、外设演示及独立构建工程 |
| Profile 和服务 | `components/profiles/` | GAP 角色、设备信息、OTA、私有服务和应用 Profile |
| Mesh 集成 | `components/ethermind/` | EtherMind 模型、Bearer/平台适配、密码集成、工具和预编译库 |
| BLE 与调度 | `components/ble/`、`components/osal/` | BLE 接口、OSAL 任务/事件、电源管理和存储 |
| 驱动 | `components/driver/` | 时钟、GPIO、UART、ADC、Flash、电源、定时器和其他 MCU 外设 |
| 源码库 | `components/libraries/` | TinyCrypt、命令行支持、存储辅助和可复用模块 |
| 预编译平台代码 | `lib/` 及组件下的 `lib/` | 厂商 Controller、Host、Mesh 和安全库 |

除特别说明外，本文路径都相对于 `Ai_PB-03F_OPEN-SOURCE`。

## 控制流与数据流

应用先初始化 MCU 和 RF/PHY、注册 OSAL 任务，然后把控制权交给 OSAL 事件循环。BLE、Mesh、定时器、UART 和外设回调通过事件或消息进行通知，应用任务消费事件后调用 Profile、服务和驱动。

在代表性的安信可 Demo 中，BLE UART 应用层负责 UART 数据与 AT 指令；GAP/GATT 服务把 BLE 事件送入同一 OSAL 任务；ADC 行为则由编译期运行模式宏选择。

## 工程边界

- 43 个 `.uvprojx` 文件分别代表独立的固件组合，实际镜像由其源码、包含路径、宏、库、编译器版本和分散加载文件共同决定。
- `example/ble_peripheral/simpleBlePeripheral/gcc` 下的 GNU Makefile 是另一套可构建组合；它成功并不代表仅支持 Keil 的工程都能用 GCC 构建。
- 修改 `components/` 可能影响大量示例。移动或重命名共享文件后，应运行仓库引用验证器。
- 仓库无法从源码完整重建所有预编译库。替换时必须保持 ABI，并验证工具链兼容性。
- 仓库内已有的 AXF、HEX、MAP、目标文件和构建日志只是历史证据，不能代替当前环境中的干净构建。

## 依赖方向

```text
应用 / 示例
  -> Profile 和服务
     -> BLE Host / Mesh 集成
        -> OSAL 和驱动
           -> MCU 硬件与预编译厂商库
```

反向回调通常只负责投递消息/事件或调用已注册处理器，应用工作随后在对应的 OSAL 任务上下文中执行。
