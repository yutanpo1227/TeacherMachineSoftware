# Teacher Machine

Arduino Mega 2560を使用したロボットサッカー（ロボカップジュニア）用の自律制御ロボットプロジェクトです。

## 📋 概要

このプロジェクトは、以下の機能を持つサッカーロボットを制御するためのファームウェアです：

- **ボール追跡**: IRセンサーによるボール位置検出
- **ライン回避**: ラインセンサーによる白線検出
- **姿勢制御**: ジャイロセンサーによる方向制御
- **全方向移動**: 4輪オムニホイールによる機動性
- **回り込み戦略**: 効率的なボール攻撃アルゴリズム

## 🔧 ハードウェア構成

### メインボード
- **Arduino Mega 2560**

### センサー類
- **ジャイロセンサー**: Adafruit BNO055
- **ラインセンサー**: 8個のフォトリフレクタ
- **IRセンサー**: 8個の赤外線センサー（ボール検出用）

### アクチュエーター
- **モーター**: 4個のDCモーター（オムニホイール用）

## 📦 ライブラリ構成

| ライブラリ | 機能 |
|-----------|------|
| `LineSensor` | 8個のセンサーから白線を検出し、線の角度とベクトル強度を計算 |
| `IRSensor` | 8個のセンサーからボールの位置（角度・距離）を検出 |
| `GyroSensor` | BNO055センサーからヨー角度を取得 |
| `Motor` | 個別モーターの方向・速度制御 |
| `MotorController` | 4輪の協調制御による全方向移動 |
| `Debugger` | シリアル通信によるデバッグ情報出力 |

## 🚀 ビルドとアップロード

### 必要環境
- [PlatformIO](https://platformio.org/)
- Python 3.7以上

### ビルドコマンド
```bash
# ライブラリ依存関係のインストール
pio lib install

# プロジェクトのビルド
pio run

# ビルドとアップロード
pio run -t upload

# シリアルモニター
pio device monitor
```

### CI/CD
GitHub Actionsによる自動ビルドチェックが設定されています。

## ⚙️ 設定パラメータ

### 基本設定
```cpp
#define SPEED 100                    // モーター速度 (0-255)
#define ENABLE_LINE_SENSOR true      // ラインセンサー有効化
#define ENABLE_GYRO_SENSOR true      // ジャイロセンサー有効化
#define ENABLE_DEBUGGER true         // デバッグモード有効化
```

### 回り込み戦略
```cpp
#define WRAP_AROUND_ANGLE_THRESHOLD 10    // 回り込み角度閾値 (度)
#define WRAP_AROUND_BALL_DIST_THRESHOLD 850  // 回り込み距離閾値
#define WRAP_AROUND_ANGLE_RATIO 1.8       // 回り込み角度倍率
```

### ラインセンサー閾値
```cpp
const int LINE_SENSOR_THRESHOLDS[] = {880, 850, 920, 870, 850, 850, 820, 900};
```

## 🎯 制御アルゴリズム

### 回り込み戦略
ボールに対する効率的なアプローチを実現：

1. **ボール角度の正規化**: -180°〜180°の範囲に変換
2. **距離による戦略変更**: 
   - 遠距離時: 大きく回り込んで攻撃角度を確保
   - 近距離時: 直接的なアプローチ
3. **角度閾値制御**: 小さな角度変化では回り込みを抑制

### センサー統合
- **ライン回避**: 白線検出時の緊急回避動作
- **姿勢補正**: ジャイロセンサーによる向き制御
- **ボール追跡**: IRセンサーによる目標設定

## 🔍 デバッグ

### シリアル出力
デバッグモードが有効な場合、以下の情報がシリアル出力されます：
- ジャイロ角度
- ライン角度・ベクトル強度
- ボール角度・距離
- センサーステータス

### モニタリング
```bash
pio device monitor --baud 9600
```

## 📁 プロジェクト構造

```
software/
├── .github/workflows/    # GitHub Actions設定
├── include/              # ヘッダーファイル
├── lib/                  # カスタムライブラリ
│   ├── Debugger/
│   ├── GyroSensor/
│   ├── IRSensor/
│   ├── LineSensor/
│   ├── Motor/
│   └── MotorController/
├── src/                  # メインソースコード
│   └── main.cpp
├── test/                 # テストコード
└── platformio.ini        # PlatformIO設定
```

## 🛠 カスタマイズ

### センサー調整
各センサーの閾値は`main.cpp`で調整可能です。

### 戦略パラメータ
回り込み戦略のパラメータを調整することで、ロボットの動作特性を変更できます。

### 新機能追加
`lib/`ディレクトリにカスタムライブラリを追加して機能拡張が可能です。

---

**対象競技**: ロボカップジュニア サッカー  
**プラットフォーム**: Arduino / PlatformIO 