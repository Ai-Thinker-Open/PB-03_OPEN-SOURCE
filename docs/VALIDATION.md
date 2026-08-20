[![中文](https://img.shields.io/badge/中文-文档-blue)](VALIDATION.zh.md)

# Validation and reproducibility

## Validation levels

| Level | Command/tool | What it proves | What it does not prove |
| --- | --- | --- | --- |
| Repository structure | `python tools/validate_repository.py` | All 43 Keil XML files parse; repository-local source/scatter references resolve; representative entry symbols and bilingual docs exist; stale include search paths are reported | Compiler compatibility or runtime behavior |
| Host crypto tests | `bash tools/validate_tinycrypt.sh` | TinyCrypt builds with host GCC and all 11 included test programs pass | PB firmware, radio, peripherals, or vendor archives |
| GNU firmware build | `bash tools/build_gcc_example.sh` | The supplied `simpleBlePeripheral` Makefile compiles and links a Cortex-M0 ELF with the installed ARM GNU Toolchain | Keil-only projects or physical-board behavior |
| Keil firmware build | Keil µVision and the compiler recorded by a project | The selected `.uvprojx` compiles and links against its configured libraries | Correct operation on hardware |
| Hardware test | Matching PB-03/PB-03F board and programmer | Boot, radio, peripheral, OTA, and application behavior for the tested setup | Other boards and configurations |

## Reproduce the checks

From the repository root:

```text
python tools/validate_repository.py
```

On WSL/Linux with a host C compiler:

```bash
bash tools/validate_tinycrypt.sh
```

On WSL/Linux with `arm-none-eabi-gcc`, Binutils, Make, and core utilities:

```bash
bash tools/build_gcc_example.sh
```

The TinyCrypt script cleans generated test files. The firmware script copies the SDK to a temporary directory, builds there, reports ELF metadata and hashes, then removes the copy. Neither script should modify tracked historical artifacts.

The validator reports legacy include-search directories that no longer exist but does not rewrite them automatically. Missing source or scatter references remain hard failures. The obsolete `components/libraries/cliface` references were repaired to the repository's actual `components/libraries/cli` directory.

## Recorded clean-build evidence

Validation environment: WSL2 Ubuntu, GNU Make 4.3, ARM GNU Toolchain 10.3.1.

The supplied `example/ble_peripheral/simpleBlePeripheral/gcc/Makefile` completed and produced:

- 0 compiler/linker errors and 59 warnings;
- ELF32 for ARM, entry address `0x11020009`;
- text 78,327 bytes, data 0 bytes, BSS 9,908 bytes (88,235 bytes total reported by `arm-none-eabi-size`);
- BIN SHA-256 `664d1f21ba33cabce910e528b8dffdafbc59c8841b044af01670533e65f78e9b`;
- IHEX SHA-256 `b12c15081ccb47385e1145ddc9a0256edee26b917956754d81a69589de8d286e`.

Two isolated clean builds produced identical BIN and IHEX hashes. The ELF hashes differed because debug information embeds the randomized temporary build path; therefore the ELF hash is reported by the script for traceability but is not claimed to be reproducible across temporary directories. Warnings are retained as visible technical debt rather than silently suppressed. They include macro-parentheses, pointer-type, format, unused-variable/function, and related legacy-code diagnostics.

## Historical Keil evidence

The checked-in `Ai-Demo/BASE/bleUart_AT_ADC/Objects/bleuart.build_log.htm` records ARM Compiler 5.06 update 6 build 750, program size `Code=72500`, `RO-data=3320`, `RW-data=2516`, `ZI-data=12952`, and a result of 0 errors/2 warnings. Its corresponding checked-in AXF and HEX are historical outputs; they were not regenerated in the current environment.

## Remaining evidence boundary

- Keil MDK/ARM Compiler 5 was not available in the current validation environment, so no new Keil output is claimed.
- No PB-03/PB-03F board, programmer, radio test, peripheral test, OTA test, or power measurement was performed.
- Only the supplied GCC example and host-buildable TinyCrypt suite were built. The other firmware compositions remain unverified until their exact projects are built and tested.
- The GNU build warnings should be triaged separately; changing legacy shared code without hardware regression testing would carry more risk than documenting the current result.

When performing a hardware validation, record repository commit, toolchain version, project/target, warning/error counts, output hashes, module and board revisions, programmer, test procedure, and observed result.
