# gn10-motor

[English](README.md) | [日本語](README_ja.md)

このリポジトリには [HTMDv2.2c](https://github.com/ararobo/HTMDv2.2c)・[HTMDv2.2s](https://github.com/ararobo/HTMDv2.2s) という基板のファームウェアが含まれています。

HTMDv2.2c は STM32F303K8T6 と STM32G431K8T6 の2つの MCU に対応しており、HTMDv2.2s は STM32G431CEU6 に対応しています。

## ドキュメント

| ドキュメント | 説明 |
| :-: | :-: |
| [CONTRIBUTING.md](./CONTRIBUTING.md) | 開発フロー・コミット規約・コーディング規約 |
| [docs/coding-rules.md](./docs/coding-rules.md) | コーディング規約の詳細 |

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

STM32F303K8T6 用 (HTMDv2.2c-f303):
```bash
cmake --preset HTMDv2.2c-f303-debug
cmake --build --preset HTMDv2.2c-f303-debug
```

STM32G431K8T6 用 (HTMDv2.2c-g431):
```bash
cmake --preset HTMDv2.2c-g431-debug
cmake --build --preset HTMDv2.2c-g431-debug
```

HTMDv2.2s 用:
```bash
cmake --preset HTMDv2.2s-debug
cmake --build --preset HTMDv2.2s-debug
```

## クラス図

![class simplified](docs/uml/motor_driver_architecture_simplified.png)

## コントリビューション

[CONTRIBUTING.md](./CONTRIBUTING.md) を参照してください。

## ライセンス
このプロジェクトは Apache-2.0 ライセンスの下で提供されています。詳細は [LICENSE](LICENSE) ファイルを参照してください。
