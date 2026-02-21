# HTMDv2.2FW

[English](README.md) | [日本語](README_ja.md)

Firmware for the customized motor driver developed by Gento Aiba (GN10).

This repository contains firmware for the [HTMDv2.2c](https://github.com/ararobo/HTMDv2.2c) board.

The HTMDv2.2c board supports two MCUs: STM32F303K8T6 and STM32G431K8T6. Therefore, this repository includes firmware targets for both MCUs.

## Development Environment Setup

### Common
Install **CMake Tools** extension for VSCode.

### Ubuntu
```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build gcc-arm-none-eabi gdb-multiarch
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

![class simplified](uml/motor_driver_architecture_simplified.png)


## Development Rules

### 1. Naming Conventions
Variable and function names must be self-explanatory. We follow the **Google C++ Style Guide** basics:
- **Class/Struct names**: `PascalCase` (e.g., `SpeedMsg`, `BatteryStatus`)
- **Function/Variable names**: `snake_case` (e.g., `get_id()`, `target_velocity`)
- **Constants/Enum values**: `kPascalCase` or `ALL_CAPS` (e.g., `kMaxSpeed`, `BATTERY_LOW`)
- **Private Member variables**: Must have a trailing underscore (e.g., speed_, voltage_) to distinguish them from local variables.

### 2. Code Formatting
- A `.clang-format` file is provided in the root directory. Please configure your editor to use it on save.

### 3. Memory Management (No Dynamic Allocation)
- Avoid using dynamic memory allocation (`new`, `malloc`, `std::vector`, `std::string`) inside the models.
- Use fixed-size arrays and primitive types.

## License
This project is licensed under the Apache-2.0 - see the [LICENSE](LICENSE) file for details.