# lazytatzv_ws (最強 ROS2 構成)

ROX2026 プロフェッショナル仕様のモジュール化構成。

## システム構成

本ワークスペースは、ROS2 の設計思想に基づき機能を完全に分離した「最強」の構成をとっています。

1.  **`robot_interfaces`**: 独自通信定義。
    *   `WheelSpeeds.msg`: 4輪の独立速度指令。
2.  **`base_teleop`**: 足回り入力変換。
    *   DualSense の `joy` メッセージを解析し、安全ガード(L2/R2)を適用した上で `cmd_vel` を発行。将来的な射出機構などの追加時は `shooter_teleop` などを並列で作る構成。
3.  **`mecanum_kinematics`**: 運動学計算。
    *   `cmd_vel` を受け取り、メカナムホイールの幾何学計算を行って `WheelSpeeds` に変換。
4.  **`at_motor_driver`**: ハードウェア抽象化（シリアル/ATコマンド版）。
    *   `WheelSpeeds` を受け取り、実際のモーター指令を発行。

## 特徴

*   **コンポーネント化 (`rclcpp_components`)**: 全ノードを同一プロセス内のコンテナで実行可能。Zero-copy transport により、**極限の低レイテンシ**を実現。
*   **関心の分離**: ハードウェア(Driver)、計算(Kinematics)、UI(Teleop)が分離しているため、メンテナンス性と拡張性が非常に高い。

## ビルド

```bash
cd lazytatzv_ws
colcon build --symlink-install
source install/setup.bash
```

## テスト

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## CI

A GitHub Actions workflow is included to run `colcon build` and `colcon test` on PRs.

## 実行

コンポーネントコンテナを利用した一括起動：
```bash
ros2 launch robot_bringup system_pro.launch.py
```

### 個別モーターテスト (デバッグ・調整用)

「とりあえず手元にあるモーターを動かしたい」「PIDの調整をしたい」という場合に便利な、個別テスト用 launch です。

```bash
# 基本起動 (ID:12(0x0C), Port:/dev/ttyUSB1)
ros2 launch robot_bringup motor_test.launch.py

# IDとポートを指定して起動 (IDは10進数で指定)
ros2 launch robot_bringup motor_test.launch.py id:=20 port:=/dev/ttyUSB0

# PID制御をオフにして生値を送信する場合
ros2 launch robot_bringup motor_test.launch.py use_pid:=false
```

#### テストコマンド例：
起動後、別のターミナルから以下のように指令を送れます。

**PIDありの場合:**
```bash
ros2 topic pub /test_motor/desired_velocity std_msgs/msg/Float64 "{data: 1.0}"
```

**PIDなしの場合:**
```bash
ros2 topic pub /test_motor/target_velocity std_msgs/msg/Float64 "{data: 0.5}"
```

**パラメータの動的変更 (PID調整など):**
```bash
ros2 param set /test_motor_controller kp 0.8
```

#### インタラクティブ・テストツール

より高度なテスト（ステップ応答、サイン波など）を簡単に行うための専用ツールを用意しています。

```bash
# コンテナ内の lazytatzv_ws ディレクトリで実行
./motor_test_tool.py
```

このツールでは以下の操作が可能です：
- **数値入力**: 指定した速度を継続して送信。
- **step**: ステップ応答テスト（指定秒数だけ回転して停止）。
- **sine**: サイン波送信（PIDの追従性確認などに便利）。
- **s**: 非常停止（速度 0.0 を送信）。
