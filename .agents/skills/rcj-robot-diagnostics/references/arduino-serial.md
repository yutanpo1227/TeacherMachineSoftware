# Arduino Mega 2560の接続とログ採取

## ポートを特定する

```sh
arduino-cli version
arduino-cli board list
arduino-cli core list
arduino-cli board details -b arduino:avr:mega:cpu=atmega2560
```

Macでは `/dev/cu.usbmodem…` や `/dev/cu.usbserial-…` が候補。抜き差しの前後で `board list` を比較し、接続したArduinoに対応する行を確認する。USB変換チップによってはボード名が `Unknown` でもポートは使える。カメラのUSBも接続されている場合は、片方ずつ確認する。ポート名を過去のログから決め打ちしない。Arduino AVRコアが必要で未導入なら `arduino-cli core update-index`、`arduino-cli core install arduino:avr`。

## USBシリアルを読む

`robot_port` に上で確認したArduinoのポートを設定する。

```sh
robot_port=/dev/cu.usbserial-XXXX
arduino-cli monitor -p "$robot_port" --describe
arduino-cli monitor -p "$robot_port" -c baudrate=115200 --timestamp
```

終了は Ctrl+C。ポートが使用中なら `lsof "$robot_port"` で開いているプロセスを確認し、PITCH、Arduino IDE、PlatformIOなどの該当モニターを閉じる。接続時にMegaがリセットして走行を始める場合があるため、ログ採取前に駆動電源の状態と機体の置き方を確認する。書き込み前にもモニターを閉じる。

症状の前後を保存するときは、リポジトリ外の一時ファイルへ出す。

```sh
robot_log=$(mktemp /tmp/rcj-serial.XXXXXX)
arduino-cli monitor -p "$robot_port" -c baudrate=115200 --quiet --timestamp | tee "$robot_log"
# Ctrl+C で終了後、必要な行を確認する
rg 'telemetry|heading-test|defense' "$robot_log"
```

## ログの読み方

- 現行の既定設定では `ENABLE_DEBUGGER` が有効で、約250msごとに `telemetry key=value ...` が出る。`role` は設定した役割、`state` は現在の動作。起動メッセージだけで止まる場合は、初期化メッセージとモード設定も確認する。
- `camera=1` はArduinoがカメラから新しい有効フレームを受信している状態。`ownRear=-1` が同時に出るなら、自ゴールを検出できていない。 `camera=0` ならカメラの給電・TX→Mega RX1（19番）・共通GND・UART設定を先に調べる。
- `ball` は制御が採用した角度で、`ir` と `cameraBall` は各検出値。角度の `-1` は未検出。`lineUsed` は白線を制御で使うモードかどうかの目安で、実際に停止や退避を指令したことまでは表さない。守備設定ではログ用の白線計測に別インスタンスを使うため、攻撃中の制御値と一致するとは限らない。
- `bottomGap` は画像上の画素値。`yaw` はジャイロ方位、`error` は目標とのずれ。停止や減速の原因は、これらと `state` を同じ時刻の行で照合する。
- MacとMegaのUSBシリアル、カメラからMegaの `Serial1`（115200bps）、MaixPy IDE用のカメラUSBは別経路。Macにログが出るだけではカメラとの通信は確認できない。

## ビルドと書き込み

リポジトリはPlatformIO構成。依存ライブラリとUARTバッファ設定を反映するため、通常のビルド・書き込みは次を使う。

```sh
pio run -e megaatmega2560
pio run -e megaatmega2560 -t upload --upload-port "$robot_port"
```

Arduino CLIでの書き込みが必要な場合は、同じ環境でビルドしたHEXと対象ポートを確認してから実行する。`arduino-cli upload` 自体はビルドしない。

```sh
arduino-cli upload -b arduino:avr:mega:cpu=atmega2560 -p "$robot_port" \
  --input-file .pio/build/megaatmega2560/firmware.hex --verify
```

Arduino CLI公式資料の [monitor](https://arduino.github.io/arduino-cli/latest/commands/arduino-cli_monitor/)、[upload](https://arduino.github.io/arduino-cli/latest/commands/arduino-cli_upload/)、[ポート検出のFAQ](https://arduino.github.io/arduino-cli/latest/FAQ/)。
