[![English](https://img.shields.io/badge/English-README-green)](README.md)

# 安信可 PB-03/PB-03F SDK

本仓库包含 PHY62XX SDK 3.1.3 源码以及 PB-03/PB-03F 示例，包括 43 个 Keil µVision 工程，覆盖 BLE 角色、Mesh、OTA、外设和安信可应用 Demo，另含一个 ARM GNU 工具链示例。

## 环境要求

- 构建 `.uvprojx` 工程需要 Windows、Keil MDK-ARM 和兼容的旧版 ARM Compiler 5。不同示例记录的编译器版本为 ARM Compiler 5.06 update 4、6 或 7。
- 硬件验证需要对应的 PB-03/PB-03F 模组、烧录器以及与开发板匹配的烧录流程。
- 可选：使用 WSL/Linux、Python 3、GCC、Make 和 `arm-none-eabi-gcc` 运行仓库提供的验证脚本。

仓库包含预编译厂商库，但不包含 Keil MDK、工具链许可证、烧录器驱动，也不包含重建全部预编译库所需的完整源码。

## 快速开始

```powershell
git clone https://github.com/Ai-Thinker-Open/PB-03_OPEN-SOURCE.git
cd PB-03_OPEN-SOURCE
```

1. 进入 `Ai_PB-03F_OPEN-SOURCE` 并选择示例。
2. 如需使用安信可 BLE UART/AT/ADC Demo，在 Keil µVision 中打开 `Ai-Demo/BASE/bleUart_AT_ADC/bleUart_AT.uvprojx`。
3. 保持工程现有的目标、分散加载文件、宏和厂商库配置，然后执行构建。
4. 按照对应模组和开发板的流程烧录生成的镜像。

修改共享组件前，请先阅读[代码入口](docs/CODE_ENTRY.zh.md)中的真实运行路径以及[架构说明](docs/ARCHITECTURE.zh.md)。

## 仓库结构

| 路径 | 用途 |
| --- | --- |
| `Ai_PB-03F_OPEN-SOURCE/Ai-Demo/` | 安信可 PB-03F 应用 Demo |
| `Ai_PB-03F_OPEN-SOURCE/example/` | BLE、Mesh、OTA 和外设示例 |
| `Ai_PB-03F_OPEN-SOURCE/components/` | 驱动、BLE 接口、OSAL、Profile、Mesh 集成和源码库 |
| `Ai_PB-03F_OPEN-SOURCE/lib/` | 预编译 Controller/Host 库及其接口 |
| `Ai_PB-03F_OPEN-SOURCE/misc/` | 链接符号和辅助资源 |
| `Ai_PB-03F_OPEN-SOURCE/release_note.md` | PHY62XX SDK 历史发布说明 |
| `tools/` | 不污染源码的仓库、主机库和固件构建检查 |

## 验证

检查全部 43 个 Keil 工程引用、文档入口符号和双语文档：

```text
python tools/validate_repository.py
```

在 WSL/Linux 中构建并运行全部 11 个 TinyCrypt 测试：

```bash
bash tools/validate_tinycrypt.sh
```

安装 ARM GNU 工具链后，在隔离的临时副本中构建 `simpleBlePeripheral` 固件：

```bash
bash tools/build_gcc_example.sh
```

固件脚本不会改写仓库中已提交的历史构建产物。最近一次记录的构建成功链接 ARM ELF，错误为 0、编译器警告为 59。这些检查不能代替开发板实测，详情见[验证与复现](docs/VALIDATION.zh.md)。

GitHub Actions 会在拉取请求及推送到 `master` 时运行仓库结构与 TinyCrypt 检查。完整 ARM 固件构建仍作为明确的本地检查，因为公共 Runner 不保证提供所需的交叉工具链。

## 技术文档

- [代码入口](docs/CODE_ENTRY.zh.md)
- [架构说明](docs/ARCHITECTURE.zh.md)
- [验证与复现](docs/VALIDATION.zh.md)

## 许可证提示

本仓库没有统一的仓库级许可证。TinyCrypt 在 `components/libraries/tinycrypt-0.2.8` 下包含自身许可证，而许多 SDK 和厂商文件带有 Phyplus 保密、专有或使用限制声明。复制、修改或再分发前，请逐个检查组件声明并确认自身权利。仓库可以公开访问并不等于获得开源许可。
