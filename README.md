# BeltConveyorController

## 概要

複数のベルトコンベアを登録し、ASM風の動作手順を検証・実行するC++17アプリケーションです。
既定の実行はドライランです。実機動作には`--live`と明示的な確認が必要です。

## 主な機能

- コンベアの定義ファイル登録と対話CLIでの管理
- ASM手順ファイルによる単軸移動、補間付き複数軸移動、XY/XZ円弧、待機、FOR反復、IF条件分岐
- 汎用ASMエンジン[EasyMachineLanguage](../EasyMachineLanguage/README.md)と
  モーションAPI[MarlinController](../Marlin/MarlinController/README.md)の利用


## 必要環境

- CMake 3.16以降
- C++17対応コンパイラ
- `../Marlin/MarlinController`および`../EasyMachineLanguage`
  （別の場所にある場合はCMake変数で指定可能）
- LinuxでMarlin実機を操作する場合はUSBシリアル接続

## ビルドとテスト

```sh
make build
make test
```

依存プロジェクトの場所を変更する場合は、CMakeの
`MARLIN_CONTROLLER_SOURCE_DIR`と`EASY_MACHINE_LANGUAGE_SOURCE_DIR`を指定してください。

## クイックスタート

次の例は実機へ接続せず、定義・手順をドライランで実行します。

```sh
./build/belt_conveyor_controller --run examples/sequence.asm --config belts.conf
```

実機運転の前に、利用ガイドに記載されたポート・軸・移動範囲・エンドストップ・非常停止を
確認してください。

## 詳細ドキュメント

- [利用ガイド](docs/USAGE.md) — 定義/手順ファイル、ASM構文、曲線、CLI、実機運転
- [開発ガイド](docs/DEVELOPMENT.md) — プロジェクト分担、ビルド、テスト、軌道計画
