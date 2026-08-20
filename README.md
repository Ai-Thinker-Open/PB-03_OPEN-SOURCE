[![中文](https://img.shields.io/badge/中文-README-blue)](README.zh.md)

# Ai-Thinker PB-03/PB-03F SDK

## Overview

This repository contains the PHY62XX SDK 3.1.3 sources and PB-03/PB-03F examples. It includes 43 Keil µVision projects for BLE roles, Mesh, OTA, peripherals, and Ai-Thinker application demos, plus one ARM GNU Toolchain example.

## Prerequisites

- Windows with Keil MDK-ARM and a compatible legacy ARM Compiler 5 installation for the `.uvprojx` projects. The projects record ARM Compiler 5.06 update 4, 6, or 7 depending on the example.
- A matching PB-03/PB-03F module, programmer, and board-specific flashing procedure for hardware verification.
- Optional: WSL/Linux, Python 3, GCC, Make, and `arm-none-eabi-gcc` for the included validation scripts.

The repository includes prebuilt vendor libraries. It does not include Keil MDK, toolchain licenses, programmer drivers, or all source required to rebuild those libraries.

## Getting started

```powershell
git clone https://github.com/Ai-Thinker-Open/PB-03_OPEN-SOURCE.git
cd PB-03_OPEN-SOURCE
```

1. Enter `Ai_PB-03F_OPEN-SOURCE` and choose an example.
2. For the Ai-Thinker BLE UART/AT/ADC demo, open `Ai-Demo/BASE/bleUart_AT_ADC/bleUart_AT.uvprojx` in Keil µVision.
3. Keep its existing target, scatter file, macros, and vendor-library selection, then build it.
4. Program the generated image using the procedure appropriate to your module and board.

See [Code entry](docs/CODE_ENTRY.md) for the exact runtime path and [Architecture](docs/ARCHITECTURE.md) before changing shared components.

## Repository layout

| Path | Purpose |
| --- | --- |
| `Ai_PB-03F_OPEN-SOURCE/Ai-Demo/` | Ai-Thinker PB-03F application demos |
| `Ai_PB-03F_OPEN-SOURCE/example/` | BLE, Mesh, OTA, and peripheral examples |
| `Ai_PB-03F_OPEN-SOURCE/components/` | Drivers, BLE interfaces, OSAL, profiles, Mesh integration, and source libraries |
| `Ai_PB-03F_OPEN-SOURCE/lib/` | Prebuilt controller/host libraries and exported interfaces |
| `Ai_PB-03F_OPEN-SOURCE/misc/` | Link symbols and supporting resources |
| `Ai_PB-03F_OPEN-SOURCE/release_note.md` | Historical PHY62XX SDK release notes |
| `tools/` | Non-destructive repository, host-library, and firmware-build checks |

## Validation

Check all 43 Keil project references, the documented entry symbols, and bilingual documentation:

```text
python tools/validate_repository.py
```

On WSL/Linux, build and run all 11 TinyCrypt tests:

```bash
bash tools/validate_tinycrypt.sh
```

With the ARM GNU Toolchain installed, build the `simpleBlePeripheral` firmware in an isolated temporary copy:

```bash
bash tools/build_gcc_example.sh
```

The firmware script does not alter checked-in build artifacts. The latest recorded run linked an ARM ELF successfully with no errors and 59 compiler warnings. These checks do not replace board testing; see [Validation and reproducibility](docs/VALIDATION.md).

GitHub Actions runs the repository-structure and TinyCrypt checks for pull requests and pushes to `master`. The full ARM firmware build remains an explicit local check because the runner image does not guarantee the required cross-toolchain.

## Troubleshooting and contributing

- If Keil reports a missing compiler, select/install the ARM Compiler 5 version recorded by the chosen `.uvprojx`; do not silently migrate a legacy project to ARM Compiler 6.
- If a project cannot find a source or scatter file, run `python tools/validate_repository.py` and distinguish hard failures from the reported legacy include-only warnings.
- If the GNU example fails, confirm that `arm-none-eabi-gcc`, Binutils, and Make are available in the same WSL/Linux shell.
- For a change, identify the exact project and module, run the repository checks, perform a clean firmware build when the toolchain is available, and report the commit, command, warnings/errors, output hashes, and hardware result in the pull request.
- Do not include credentials, toolchain licenses, or generated build directories in a contribution.

## Technical documentation

- [Code entry](docs/CODE_ENTRY.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Validation and reproducibility](docs/VALIDATION.md)

## Licensing notice

There is no single repository-level license. TinyCrypt has its own license under `components/libraries/tinycrypt-0.2.8`, while many SDK and vendor files contain Phyplus confidential/proprietary notices and restrictions. Review the notice in each component and confirm your rights before copying, modifying, or redistributing it. Public repository access alone does not grant an open-source license.
