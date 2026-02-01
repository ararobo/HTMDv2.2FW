# HTMDv2.2FW
![Platform](https://img.shields.io/badge/Platform-STM32%20)
Gento Aiba(GN10)が開発しているモータードライバーのファームウェアです。

HTMDとはHaruto Tanakaが作成したモータードライバーです。
このリポジトリにはHTMDv2.2cという基板のファームウェアが入っています。

HTMDv2.2cはSTM32F303K8T6と、STM32G431K8T6の2つのMCUに対応しているので、それぞれのMCU用に計２つのファームウェアがあります。

## Development Environment Setup

### Common
Install **CMake Tools** extension for VSCode.

### Ubuntu
```bash
sudo apt update
sudo apt install build-essential cmake ninja-build
```

### Windows(for STM32)

Install STM32CubeCLT.

## Build
for STM32F303K8T6:
```bash
cmake --preset f303-debug
cmake --build --preset f303-debug
```
for STM32G431K8T6:
```bash
cmake --preset g431-debug
cmake --build --preset g431-debug
```

## Class Diagram

![Class Diagram](uml/class.png)

## Development Rules

### 1. Naming Conventions
Variable and function names must be self-explanatory. We follow the **Google C++ Style Guide** basics:
- **Class/Struct names**: `PascalCase` (e.g., `SpeedMsg`, `BatteryStatus`)
- **Function/Variable names**: `snake_case` (e.g., `get_id()`, `target_velocity`)
- **Constants/Enum values**: `kPascalCase` or `ALL_CAPS` (e.g., `kMaxSpeed`, `BATTERY_LOW`)
- **Private Member variables**: Must have a trailing underscore (e.g., speed_, voltage_) to distinguish them from local variables.

### 2. Code Formatting
- Do not rely on IDE defaults.
- All code must be formatted using **Clang-Format**.
- A `.clang-format` file is provided in the root directory. Please configure your editor to use it on save.
