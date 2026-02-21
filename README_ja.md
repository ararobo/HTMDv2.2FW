# HTMDv2.2FW

[English](README.md) | [日本語](README_ja.md)

Gento Aiba (GN10) が開発しているモータードライバーのファームウェアです。
このリポジトリには [HTMDv2.2c](https://github.com/ararobo/HTMDv2.2c) という基板のファームウェアが含まれています。

HTMDv2.2c は STM32F303K8T6 と STM32G431K8T6 の2つの MCU に対応しているため、それぞれの MCU 用に計2つのファームウェアターゲットが存在します。

## 開発環境のセットアップ

### 共通
VSCode の **CMake Tools** 拡張機能をインストールしてください。

### Ubuntu
```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build gcc-arm-none-eabi gdb-multiarch
```

### Windows (STM32用)
STM32CubeCLT をインストールしてください。

## ビルド方法

STM32F303K8T6 用:
```bash
cmake --preset f303-debug
cmake --build --preset f303-debug
```

STM32G431K8T6 用:
```bash
cmake --preset g431-debug
cmake --build --preset g431-debug
```

## クラス図

![Class Diagram](uml/class.png)

## 開発ルール

### 1. 命名規則
変数名や関数名は、それだけで意味が分かるように命名してください。**Google C++ Style Guide** に準拠します：
- **クラス/構造体名**: `PascalCase` (例: `SpeedMsg`, `BatteryStatus`)
- **関数/変数名**: `snake_case` (例: `get_id()`, `target_velocity`)
- **定数/列挙値**: `kPascalCase` または `ALL_CAPS` (例: `kMaxSpeed`, `BATTERY_LOW`)
- **プライベートメンバ変数**: ローカル変数と区別するため、末尾にアンダースコアを付けます (例: `speed_`, `voltage_`)。

### 2. コードフォーマット
- IDE のデフォルト設定に頼らないでください。
- すべてのコードは **Clang-Format** を使用してフォーマットする必要があります。
- ルートディレクトリにある `.clang-format` ファイルを使用するようにエディタを設定してください（保存時自動フォーマット推奨）。

### 3. メモリ管理（動的割り当て禁止）
- モデル内部での動的メモリ割り当て（`new`, `malloc`, `std::vector`, `std::string` など）の使用は避けてください。
- 固定長配列やプリミティブ型を使用してください。

## ライセンス
このプロジェクトは Apache-2.0 ライセンスの下で提供されています。詳細は [LICENSE](LICENSE) ファイルを参照してください。

## メンテナ
- Gento Aiba
- Koichiro Watanabe
- Akeru Kamagata
