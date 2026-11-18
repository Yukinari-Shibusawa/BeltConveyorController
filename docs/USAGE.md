# 利用ガイド

## ビルド

MarlinControllerが隣接ディレクトリにある場合:

```sh
make build
make test
```

別の場所にある場合:

```sh
cmake -S . -B build -DMARLIN_CONTROLLER_SOURCE_DIR=/path/to/Marlin/MarlinController
cmake --build build
```

`make install` はMarlinControllerと本アプリのライブラリ/ヘッダーを `./install/` に
インストールします。インストール先は `make INSTALL_PREFIX=/path/to/prefix install` で
変更できます。sudoを使ったシステム全体へのインストールは行いません。

## 定義/手順ファイル

装置定義は `belts.conf` に置きます。

```text
LIMITS 220 30000 35000
BELT A MARLIN /dev/ttyUSB0
AXIS A X 220
AXIS A Y 220
AXIS A Z 220
```

`LIMITS` は1命令あたりの最大距離mm、最大実行時間ms、通信timeout msです。
`AXIS`は最大位置だけを定義し、速度上限は設定しません。
MarlinController側のCLI用TOML設定は参照しません。

シーケンスファイルには動作だけを記述します。

```text
A X100
WAIT 1000
```

`BELT` は名前、通信方式、ポートを定義します。現時点で `MARLIN` のみ有効で、
baud rateは115200固定です。`AXIS` は名前、XYZの軸、最大位置(mm)を定義します。
位置は絶対座標です。手順の`F`値はMarlin G0/G1と同じmm/minで、省略時は`F10000`
（約166.67mm/s）です。
たとえば`A X100 F600`はAのX軸を100mmへ600mm/min（10mm/s）で動かします。
MotionController APIにもmm/minのまま渡します。速度上限はソフトウェアで設定しません。
`F`値は装置とファームウェアの定格・安全設定の範囲内にしてください。

1行に複数の軸座標を書くと、同じMarlinコントローラーへ1つの補間移動として送ります。
個別行は従来どおり順番に実行され、各移動の完了を待ってから次の行へ進みます。

```text
A X100 Y100 F600
A Z20 F300
```

最初の行はXYを同時に補間して移動し、次の行はXY移動の完了後にZを動かします。
指定しない軸は現在位置を維持します。

中心座標と直径でXYまたはXZ平面の全円を指定できます。

```text
CIRCLE A X100 Y100 D50 CW F600
CIRCLE A X110 Z125 D40 CW F600
```

1行目はXY平面で中心(X100,Y100)、2行目はXZ平面で中心(X110,Z125)の円です。
開始点は円周上のX正方向端とし、必要ならそこまで直線移動してから円弧を実行します。
方向は`CW`（既定）または`CCW`です。実機では単一のMarlin G2/G3円弧と完了待ちを使い、
円周全体をトラバース範囲、最大移動距離、最大時間で検証します。

### XZ平面の円

XZ平面の円は`G18`とG2/G3のI/Kオフセットを使います。ファームウェアが
`CNC_WORKSPACE_PLANES`を有効にしている必要があります。無効な場合はMarlinが拒否します。
`examples/xz-circle.asm`は中心X110/Z125 mm、直径40 mm、送り速度600 mm/minのネイティブ円弧例です。

最初にドライランで確認してください。`G18`単独をMarlinに送ってXZ平面選択を確認し、
続けて`G17`を送りXY平面へ戻せます。Liveモード実行時は設定された全軸をG28でホームします。
ホーム動作やX/Zの動作方向・範囲を安全確認できていない場合はLive実行しないでください。
座標と直径は[`belts.conf`](../belts.conf)のX/Z上限および実際の可動範囲内に調整してください。

`REMOVE <name>` はインスタンス登録を削除します。定義ファイルと手順ファイルは
分離します。`;` と `#` 以降はコメントです。

## FOR / IF

ASM手順ファイルでは整数カウンターによる反復と条件分岐を使えます。構文解析と制御フローは
隣接する独立プロジェクト[EasyMachineLanguage](../../EasyMachineLanguage/README.md)が担当し、
ベルト固有命令の意味付けはBeltConveyorController側のホストが行います。

```text
FOR step = 1 TO 3
A X{step} F600
END

IF A.X >= 3
A X100
ELSE
A X0
END
```

`FOR <変数> = <整数> TO <整数> [STEP <正の整数>]`で反復します。STEPを省略すると、
開始値から終点まで1ずつ増減します。刻み幅を指定すると進行方向は開始値と終点から決まり、
`FOR i = 100 TO 300 STEP 50`は100、150、200、250、300の順になります。終点に届かない
場合は、終点を超える値を処理しません。カウンターは`{step}`の形で命令の数値に埋め込めます。IFは
`IF <変数|ベルト名.軸> <比較演算子> <数値|変数>`の形式です。比較演算子は`<`、`<=`、
`==`、`!=`、`>=`、`>`です。`ELSE`は省略でき、`END`でブロックを閉じます。
位置条件（例:`A.X <= 100`）はドライランではシミュレーション上の既知位置を、
実機モードではMarlinが報告する推定位置を使います（エンコーダーによる実位置では
ありません）。位置が不明・取得不能ならエラーになります。実機モードではIF評価時に
必要に応じて接続・原点復帰してから位置を読みます。
各FORは最大10000反復、展開後の検証・実行は最大100000文です。
制御ブロックのネストは128階層までです。
定義命令は制御ブロック内に置けません。実機実行時は全分岐を事前検証し、条件に合う
分岐だけを実行します。FOR/IFは`--run`で実行するASMファイル用で、1行ずつ入力する
REPLではブロック構文を使えません。

## Bezier曲線

別の曲線ファイルに次の形式で記述します。

```text
CURVE diagonal BEZIER A
P0 X0 Y0
P1 X0 Y100
P2 X100 Y100
P3 X100 Y0
END
```

この曲線はAにX/Yの両軸が登録されている必要があります。手順に
`PATH diagonal` と書くと（`F`省略時は10000mm/min）、定義された始点P0から終点P3までを
五次smoothstepでパラメータ補間し、101点でサンプリングして位置上限と正の送り速度
（mm/min）を検証します。
現在位置が既知の場合はP0と一致する必要があります。
Marlin G5のFはmm/s仕様です。実機G5送信を追加する場合は、送信時にmm/minからmm/sへ
換算します。現在のPATHはドライラン専用で、実機送信は行いません。

## 実行

プログラムファイルをドライラン:

```sh
./build/belt_conveyor_controller --run examples/sequence.asm --config belts.conf \
  --curves examples/curves.curve
```

対話CLI:

```sh
./build/belt_conveyor_controller --config belts.conf --curves examples/curves.curve
```

REPLでは `BELT` / `AXIS` / `REMOVE` による定義変更が指定された設定ファイルに自動保存
されます。動作命令は入力直後に検証・表示します。`LIST` で登録一覧、`HELP` で構文、
`QUIT` で終了します。

## 実機動作

実機モードはデフォルトでは無効です。明示的に起動し、確認プロンプトで`y`または`Y`
を入力したときだけMarlinに接続して動きます。既定の`N`（空入力を含む）やその他の
入力はキャンセルになります:

```sh
./build/belt_conveyor_controller --live --run examples/live-sequence.asm \
  --config belts.conf
```

最初の移動時に接続・ファームウェア応答確認を行い、そのポートで定義されている軸のみ
`G28`で原点復帰してから座標を0に設定します。ファイル全体を事前検証し、構文・範囲
エラーがあれば接続も原点復帰も行いません。`PATH`は現在の同期MotionController APIでは
連続軌道を保証できないため、実機モードでは実行を拒否します。曲線はドライランで
のみ使用できます。

軸位置・速度、1回の移動距離、移動時間は、設定された上限と同じ値まで許可されます
（`<=`）。上限を超える値は拒否されます。

## ドライランについて

デフォルト実行とテストはドライランで、シリアルポートを開きません。実機動作は
`--live`と対話確認が必要です。BezierのPATHサンプル表示はドライラン用です。実機の連続的な
Bezier経路動作にはMotionController側に軌道バッファ/連続送信APIが必要です。実機操作前に、
物理原点、各軸の範囲と速度、非常停止、ポート対応を確認してください。
