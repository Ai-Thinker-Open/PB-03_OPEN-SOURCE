[![English](https://img.shields.io/badge/English-Docs-green)](VALIDATION.md)

# 验证与复现

## 验证层级

| 层级 | 命令/工具 | 能证明什么 | 不能证明什么 |
| --- | --- | --- | --- |
| 仓库结构 | `python tools/validate_repository.py` | 43 个 Keil XML 均可解析；仓库内源码/分散加载引用可解析；代表性入口符号和双语文档存在；过期包含搜索路径会被报告 | 编译器兼容性或运行行为 |
| 主机密码库测试 | `bash tools/validate_tinycrypt.sh` | TinyCrypt 可用主机 GCC 构建，内含的 11 个测试程序全部通过 | PB 固件、射频、外设或厂商库 |
| GNU 固件构建 | `bash tools/build_gcc_example.sh` | 仓库提供的 `simpleBlePeripheral` Makefile 可用已安装的 ARM GNU 工具链编译并链接 Cortex-M0 ELF | 仅支持 Keil 的工程或开发板行为 |
| Keil 固件构建 | Keil µVision 和工程记录的编译器 | 所选 `.uvprojx` 能按配置的库完成编译链接 | 在真实硬件上运行正确 |
| 硬件测试 | 匹配的 PB-03/PB-03F 开发板和烧录器 | 当前组合的启动、射频、外设、OTA 和应用行为 | 其他开发板和配置 |

## 复现检查

在仓库根目录运行：

```text
python tools/validate_repository.py
```

在带主机 C 编译器的 WSL/Linux 中运行：

```bash
bash tools/validate_tinycrypt.sh
```

在带 `arm-none-eabi-gcc`、Binutils、Make 和基础工具的 WSL/Linux 中运行：

```bash
bash tools/build_gcc_example.sh
```

TinyCrypt 脚本会清理生成的测试文件。固件脚本把 SDK 复制到临时目录后构建，输出 ELF 信息和哈希，最后删除临时副本。两个脚本都不应修改仓库中已跟踪的历史产物。

验证器会报告已不存在的旧版包含搜索目录，但不会自动改写它们；缺失源码或分散加载文件仍会导致验证失败。原来指向 `components/libraries/cliface` 的过期引用已经修复为仓库实际存在的 `components/libraries/cli`。

## 已记录的干净构建证据

验证环境：WSL2 Ubuntu、GNU Make 4.3、ARM GNU Toolchain 10.3.1。

仓库提供的 `example/ble_peripheral/simpleBlePeripheral/gcc/Makefile` 已成功完成，并生成：

- 编译/链接错误 0 个，警告 59 个；
- ELF32、ARM 架构，入口地址 `0x11020009`；
- text 78,327 字节、data 0 字节、BSS 9,908 字节，`arm-none-eabi-size` 报告合计 88,235 字节；
- BIN SHA-256：`664d1f21ba33cabce910e528b8dffdafbc59c8841b044af01670533e65f78e9b`；
- IHEX SHA-256：`b12c15081ccb47385e1145ddc9a0256edee26b917956754d81a69589de8d286e`。

两次隔离式干净构建得到相同的 BIN 和 IHEX 哈希。由于调试信息会嵌入随机临时构建路径，ELF 哈希并不相同；因此脚本仍输出 ELF 哈希用于追踪，但不声称它能跨临时目录复现。警告作为技术债明确保留，而不是静默屏蔽。其类型包括宏括号、指针类型、格式、未使用变量/函数以及其他旧代码诊断。

## 历史 Keil 证据

仓库内的 `Ai-Demo/BASE/bleUart_AT_ADC/Objects/bleuart.build_log.htm` 记录了 ARM Compiler 5.06 update 6 build 750、程序大小 `Code=72500`、`RO-data=3320`、`RW-data=2516`、`ZI-data=12952`，结果为 0 个错误、2 个警告。对应 AXF 和 HEX 是历史产物，本次环境未重新生成它们。

## 尚未覆盖的证据边界

- 当前验证环境没有 Keil MDK/ARM Compiler 5，因此不声称生成了新的 Keil 产物。
- 没有执行 PB-03/PB-03F 开发板、烧录器、射频、外设、OTA 或功耗测试。
- 本次仅构建仓库提供的 GCC 示例和可在主机上构建的 TinyCrypt 测试；其余固件组合仍需逐个按真实工程构建和测试。
- GNU 构建警告应另行分析；在没有硬件回归测试时直接修改旧版共享代码，风险高于如实记录当前结果。

执行硬件验证时，应记录仓库提交、工具链版本、工程/目标、警告和错误数量、产物哈希、模组与开发板版本、烧录器、测试步骤和观察结果。
