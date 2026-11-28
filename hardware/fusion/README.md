# Fusion 設計データ

2026年11月28日に Fusion の「TeacherMachine」フォルダから書き出したデータです。
Fusion のデータパネルでアップロードして開けます。

- `assembly/Machine2.f3z`：機体全体の 3D アセンブリ。外部参照を含む Fusion アーカイブです。
- `electronics/IRSensor.fbrd`：IR センサー基板の PCB 配置。
- `electronics/メイン基盤.fbrd`：メイン基板の PCB 配置。
- `electronics/BatteryConnectBoard.fbrd`：バッテリー接続基板の PCB 配置。
- `electronics/サークルライン.fbrd`：ラインセンサー基板の PCB 配置。
- `electronics/MotorCircuit (~recovered).fbrd`：モーター基板の PCB 配置。Fusion で開いていた復元データから書き出したため、保存済みの設計との差分は未確認です。

`.fbrd` は PCB 配置の書き出しで、回路図を含むとは限りません。各基板の回路図はこのディレクトリにはまだ含まれていません。機体の 3D データに含まれる基板形状とは別に、元の電子設計データの確認が必要です。

書き出し時の機体アセンブリには、Fusion 上で外部参照の更新通知と履歴の警告が表示されていました。元の設計を変更しないよう、参照の更新は行っていません。基板を単独で開いた際には回路図と PCB の同期に関する警告も表示されました。編集や製造に使う前に、Fusion 上の元データと照合してください。
