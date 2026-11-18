# 開発ガイド

## 構成

- `include/` は公開API、`src/` は実装とCLI、`tests/` は単体テストです。
- `beltcontroller_core` はベルト固有命令の解析、上限検証、軌道サンプリングを担当します。
- `../EasyMachineLanguage/` は独立したC++17/CMakeプロジェクトで、汎用ASM構文
  （FOR/IF、変数展開、検証と実行）を担当します。
- BeltConveyorControllerの `BeltProgramHost` は `easyml::ProgramHost` を継承し、
  ベルト固有命令、位置条件、シミュレーション/実機の動作を提供します。
- CMakeは `../EasyMachineLanguage` と `../Marlin/MarlinController` を依存として取り込みます。

EasyMachineLanguage単独のビルドとテスト:

```sh
cmake -S ../EasyMachineLanguage -B ../EasyMachineLanguage/build
cmake --build ../EasyMachineLanguage/build
ctest --test-dir ../EasyMachineLanguage/build --output-on-failure
```

BeltConveyorController全体の `make test` はEasyMachineLanguageのテストも実行します。

MarlinControllerプロジェクトが提供する `motion::MotionController` APIはXYZ軸、位置mm、
送り速度mm/minです（Marlin G0/G1と同じ）。`--live`では単軸移動、1つのG1で補間する複数軸
移動、G2/G3のXY円およびXZ円を実行します。XZ円はMarlinの`CNC_WORKSPACE_PLANES`を必要とします。
Bezier PATHは現在もドライラン専用です。Marlin G5のFは
mm/sなので、G5を実装する場合はプロトコル境界で単位変換します。

## 曲線計画

- XYの3次Bezierは4つの制御点を必須とし、五次smoothstepでパラメータを進めながら
  100区間にサンプリングします。
- 各制御点とサンプル点を定義済み軸上限で検証します。送り速度はmm/minで保持します。
- 単軸移動のQUINTICは五次smoothstep `6t^5 - 15t^4 + 10t^3` の正規化進捗です。
- 同一行の複数座標は1回の `motion::MotionController::moveTo(Position, feedrate)` で補間し、
  次の行へ進む前に同期完了を待ちます。
- `CIRCLE`は中心・直径からXYまたはXZ全円を定義し、円周上の開始点へ移動して単一のG2/G3で実行します。
- `--live`ではBezier PATHを拒否します。現在の同期APIでは連続キュー投入できません。
- 実機モードは対象装置の接続時に確認済みの定義軸だけをホーミングします。
- PATHは実機モードで拒否します。現在の同期APIでは位置点を連続キュー投入できません。
- 既定の `LIMITS` は最大距離220mm、最大時間30000ms、timeout35000msです。
- Marlin G5によるベジェ送信は未実装です。将来実装する場合はファームウェアの
  `BEZIER_CURVE_SUPPORT`が必要で、G5のmm/s F単位への換算を送信箇所に限定します。

`motion::MotionController::moveTo()` は同期して移動完了を待つAPIです。サンプル点をそのまま
一つずつ送信する方式は連続性を保証しないため、PATHは実機モードで拒否します。

## ビルドと安全

```sh
make test
```

ビルドやテストからシリアル接続または物理動作を実行してはいけません。実機モードは
`--live`と対話確認を必須とします。実際の速度/座標の安全性、原点スイッチ、非常停止を
ソフトウェアだけで保証できないため、実機前に担当者が定義ファイルと機器を照合します。
