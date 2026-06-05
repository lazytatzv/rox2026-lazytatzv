#!/usr/bin/env python3
"""
メカナムホイール 4WD ラジコン制御プログラム
================================================

【メカナムホイール配置】
  [ID1 前左 FL] -------- [ID2 前右 FR]
      |                  |
  [ID3 後左 RL] -------- [ID4 後右 RR]

【DualSense 操作方法】
  左スティック 上下  → 前進 / 後退
  左スティック 左右  → 左右 平行移動（ストレーフ）
  右スティック 左右  → その場 右旋回 / 左旋回
  OPTIONS ボタン    → プログラム終了

【起動手順】
  1. DualSense を Bluetooth で接続する（初回は pair_dualsense.sh を参照）
    2. モーター通信用 USBシリアルを接続する
  3. uv run python mecanum_rc.py  でこのプログラムを実行する

【最初に調整するパラメータ】
    - DEADZONE             : スティックの微小なぶれを無視する範囲
    - AT_SPEED_PERCENT     : 最高速度の上限（0〜100[%]）
    - TRANSLATION_GAIN     : 前後/左右移動の効き
    - ROTATION_GAIN        : 旋回の効き
    - INPUT_CURVE_EXPONENT : スティック中心付近の繊細さ（大きいほどマイルド）
    - INPUT_SLEW_RATE      : 急加速を抑える上限（0で無効）

【通常は変更しないパラメータ】
    - MOTOR_ID            : モーターID変更時のみ
    - SERIAL_PORT/BAUD    : 接続先や通信速度を変えるときのみ
    - CONTROL_HZ          : 制御周期（タイミング設計に関わる）
================================================
"""

import serial
import pygame
import os
import glob
import time
import sys
import subprocess
from pathlib import Path

try:
    from evdev import InputDevice, ecodes, list_devices
except ImportError:
    InputDevice = None
    ecodes = None

    def list_devices() -> list[str]:
        return []


# ============================================================
# 設定パラメータ
# ============================================================

#「デッドゾーン」「速度感」「操作感」を調整する
# ここで挙げた以外の値は、理由がある場合のみ変更する

# 低レイヤ設定（通常は変更不要）
# USBシリアルの接続先・通信条件
SERIAL_PORT = os.getenv("SERIAL_PORT", "/dev/ttyUSB1").strip() or "/dev/ttyUSB1"
SERIAL_BAUD = int(os.getenv("SERIAL_BAUD", "921600"))
# フレーム送信の最小間隔 [s]（全書き込みで一元管理）
SERIAL_WRITE_INTERVAL = float(os.getenv("SERIAL_WRITE_INTERVAL", "0.0008"))
ENABLE_RETRY_COUNT = int(os.getenv("ENABLE_RETRY_COUNT", "3"))
ENABLE_RETRY_INTERVAL = float(os.getenv("ENABLE_RETRY_INTERVAL", "0.05"))
# 変化時のみ送信
SEND_ONLY_ON_CHANGE = os.getenv("SEND_ONLY_ON_CHANGE", "1").strip().lower() in {
    "1", "true", "yes", "on", "y"
}
# 変化が無くても再送する周期 [s]（0以下で再送無効）
COMMAND_REFRESH_SEC = float(os.getenv("COMMAND_REFRESH_SEC", "0"))

# 低レイヤ設定（通常は変更不要）
# モーターID（ATフレーム内の ID バイト）
MOTOR_ID = {
    "FL": 0x0C,
    "FR": 0x14,
    "RL": 0x1C,
    "RR": 0x24,
}

# 調整項目: 最高速度の上限
# モーター速度レンジ（AT値）
AT_NEUTRAL_VALUE = 0x7FFF
# 直感的な速度上限 [%]。デフォルト50% 　"50"の数字を書き換えてください
AT_SPEED_PERCENT = float(os.getenv("AT_SPEED_PERCENT", "50"))
AT_SPEED_PERCENT = max(0.0, min(100.0, AT_SPEED_PERCENT))
# 互換: 既存の AT_SPEED_SPAN が指定されている場合はそちらを優先
_AT_SPEED_SPAN_ENV = os.getenv("AT_SPEED_SPAN", "").strip()
if _AT_SPEED_SPAN_ENV:
    AT_SPEED_SPAN = int(_AT_SPEED_SPAN_ENV)
    AT_SPEED_SPAN = max(0, min(AT_NEUTRAL_VALUE, AT_SPEED_SPAN))
    AT_SPEED_PERCENT = (AT_SPEED_SPAN / AT_NEUTRAL_VALUE) * 100.0
else:
    AT_SPEED_SPAN = int(round(AT_NEUTRAL_VALUE * (AT_SPEED_PERCENT / 100.0)))

# 機体パラメータ（機体寸法に合わせて変更。旋回の効きに関係）
WHEEL_BASE_HALF_L = 0.12   # 前後ホイール間距離の半分 [m]
WHEEL_BASE_HALF_W = 0.10   # 左右ホイール間距離の半分 [m]

# スティックのデッドゾーン
DEADZONE = 0.08
# 入力センター補正（起動時に中立値を自動学習）
INPUT_CENTER_CALIB_SEC = float(os.getenv("INPUT_CENTER_CALIB_SEC", "0.5"))
# 念のため最終的な速度指令にも小さなデッドバンドをかける
COMMAND_DEADBAND = float(os.getenv("COMMAND_DEADBAND", "0.03"))

# 操作感（効き・曲線・急加速制限　デフォルト：1.0）
TRANSLATION_GAIN = float(os.getenv("TRANSLATION_GAIN", "1.0"))
ROTATION_GAIN = float(os.getenv("ROTATION_GAIN", "1.0"))
INPUT_CURVE_EXPONENT = float(os.getenv("INPUT_CURVE_EXPONENT", "1.0"))
# 1秒あたりに変化できる最大量（0で無効）
INPUT_SLEW_RATE = float(os.getenv("INPUT_SLEW_RATE", "0.0"))
# ニュートラル近傍は0指令へ吸着（方向反転のチャタリング抑制）
MOTOR_ZERO_HOLD_BAND = float(os.getenv("MOTOR_ZERO_HOLD_BAND", "0.06"))
MOTOR_ZERO_HOLD_BAND = max(0.0, min(1.0, MOTOR_ZERO_HOLD_BAND))
# モーター指令の最小更新間隔 [s]
MOTOR_MIN_SEND_INTERVAL = float(os.getenv("MOTOR_MIN_SEND_INTERVAL", "0.05"))
# 大きな変化時は最小更新間隔を無視して即時送信する差分閾値
MOTOR_FORCE_SEND_DELTA = float(os.getenv("MOTOR_FORCE_SEND_DELTA", "0.20"))
# AT値のヒステリシス幅（count）。小変化を吸収してカタつき低減。
AT_VALUE_HYSTERESIS_COUNTS = int(os.getenv("AT_VALUE_HYSTERESIS_COUNTS", "220"))
# 方向反転時のゼロ近傍ガード幅（count）。微小反転を中立へ吸着。
AT_REVERSE_GUARD_COUNTS = int(os.getenv("AT_REVERSE_GUARD_COUNTS", "420"))
# 起動直後の誤動作防止: 一定時間は停止コマンドを維持
STARTUP_STOP_SEC = float(os.getenv("STARTUP_STOP_SEC", "0.8"))
# L2/R2 押下判定: 起動時基準値からの増加量
TRIGGER_DELTA_THRESHOLD = float(os.getenv("TRIGGER_DELTA_THRESHOLD", "0.20"))
# L2/R2 押下判定のヒステリシス幅（境界付近のON/OFFチラつき抑制）
TRIGGER_HYSTERESIS = float(os.getenv("TRIGGER_HYSTERESIS", "0.05"))


def _env_flag(name: str, default: str = "0") -> bool:
    value = os.getenv(name, default).strip().lower()
    return value in {"1", "true", "yes", "on", "y"}


DEBUG_INPUT = _env_flag("DEBUG_INPUT", "0")

# DualSense の MAC アドレスを指定
DUALSENSE_MAC_ADDRESS = os.getenv("DUALSENSE_MAC_ADDRESS", "").strip()

# 制御ループ周期
CONTROL_HZ = int(os.getenv("CONTROL_HZ", "20"))  # Hz


def candidate_serial_ports() -> list[str]:
    ports: list[str] = []
    if SERIAL_PORT:
        ports.append(SERIAL_PORT)

    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        for path in sorted(glob.glob(pattern)):
            if path not in ports:
                ports.append(path)

    return ports


def _build_enable_cmd(motor_addr: int) -> bytes:
    return bytes([
        0x41, 0x54, 0x20, 0x07, 0xE8, motor_addr,
        0x08, 0x00, 0xC4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x0A,
    ])


def _build_velocity_cmd(motor_addr: int, normalized_speed: float) -> bytes:
    speed = max(-1.0, min(1.0, normalized_speed))
    if abs(speed) < 1e-4:
        direction = 0x00
        value = AT_NEUTRAL_VALUE
    else:
        direction = 0x01
        delta = int(speed * AT_SPEED_SPAN)
        value = max(0x0000, min(0xFFFF, AT_NEUTRAL_VALUE + delta))

    return bytes([
        0x41, 0x54, 0x90, 0x07, 0xE8, motor_addr,
        0x08, 0x05, 0x70, 0x00, 0x00, 0x07, direction,
        (value >> 8) & 0xFF, value & 0xFF, 0x0D, 0x0A,
    ])


def _normalized_to_at_value(normalized_speed: float) -> int:
    speed = max(-1.0, min(1.0, normalized_speed))
    delta = int(round(speed * AT_SPEED_SPAN))
    return max(0x0000, min(0xFFFF, AT_NEUTRAL_VALUE + delta))


def _build_velocity_cmd_value(motor_addr: int, at_value: int) -> bytes:
    value = max(0x0000, min(0xFFFF, int(at_value)))
    direction = 0x00 if value == AT_NEUTRAL_VALUE else 0x01
    return bytes([
        0x41, 0x54, 0x90, 0x07, 0xE8, motor_addr,
        0x08, 0x05, 0x70, 0x00, 0x00, 0x07, direction,
        (value >> 8) & 0xFF, value & 0xFF, 0x0D, 0x0A,
    ])


# ============================================================
# モータークラス
# ============================================================

class SerialATBus:
    def __init__(self):
        self.ser = None
        self.raw_fd: int | None = None
        self._last_write_at: float = 0.0
        last_error: Exception | None = None
        tried_ports = candidate_serial_ports()

        for port in tried_ports:
            try:
                self.ser = serial.Serial(port=port, baudrate=SERIAL_BAUD, timeout=0.01)
                try:
                    self.ser.setDTR(False)
                    self.ser.setRTS(False)
                except (OSError, serial.SerialException) as e:
                    print(f"[WARN] DTR/RTS の設定をスキップします ({port}): {e}")
                print(f"           ✓ シリアル接続完了: {port} @ {SERIAL_BAUD}")
                break
            except Exception as e:
                last_error = e
                # pyserial が tcflush で失敗する環境向けフォールバック
                if "Inappropriate ioctl for device" in str(e):
                    if self._open_raw_fallback(port):
                        print(f"           ✓ シリアル接続完了(raw): {port} @ {SERIAL_BAUD}")
                        break

        if self.ser is None and self.raw_fd is None:
            raise RuntimeError(
                "\n"
                "  シリアルポートに接続できませんでした。\n"
                f"  候補: {tried_ports}\n"
                f"  最後のエラー: {last_error}\n"
            )

    def _open_raw_fallback(self, port: str) -> bool:
        try:
            subprocess.run(
                ["stty", "-F", port, str(SERIAL_BAUD), "raw", "-echo"],
                check=True,
                capture_output=True,
                text=True,
            )
            self.raw_fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
            return True
        except Exception:
            self.raw_fd = None
            return False

    def write(self, data: bytes) -> None:
        now = time.monotonic()
        wait_sec = SERIAL_WRITE_INTERVAL - (now - self._last_write_at)
        if wait_sec > 0:
            time.sleep(wait_sec)

        if self.ser is not None:
            self.ser.write(data)
            self._last_write_at = time.monotonic()
            return
        if self.raw_fd is not None:
            try:
                os.write(self.raw_fd, data)
                self._last_write_at = time.monotonic()
            except OSError as e:
                raise serial.SerialException(f"raw write failed: {e}") from e
            return
        raise RuntimeError("シリアルポートが初期化されていません")

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()
        if self.raw_fd is not None:
            os.close(self.raw_fd)
            self.raw_fd = None


class EDULITE05:
    """ATフレーム経由で Robstride EDULITE05 を制御するクラス"""

    def __init__(self, bus: SerialATBus, motor_id: int):
        self.bus = bus
        self.motor_id = motor_id
        self._last_sent_speed: float | None = None
        self._last_sent_value: int = AT_NEUTRAL_VALUE
        self._last_sent_at: float = 0.0

    def _send(self, data: bytes) -> None:
        try:
            self.bus.write(data)
        except (serial.SerialException, OSError) as e:
            print(f"[Serial Error] Motor {self.motor_id:#04x}: {e}")

    def enable(self) -> None:
        self._send(_build_enable_cmd(self.motor_id))

    def disable(self) -> None:
        self.set_velocity(0.0)

    def set_velocity(self, normalized_speed: float, force: bool = False) -> None:
        speed = max(-1.0, min(1.0, normalized_speed))
        if abs(speed) < MOTOR_ZERO_HOLD_BAND:
            speed = 0.0

        target_value = _normalized_to_at_value(speed)
        current_offset = self._last_sent_value - AT_NEUTRAL_VALUE
        target_offset = target_value - AT_NEUTRAL_VALUE

        # 方向反転直後の微小指令は中立に吸着してギア鳴き/カタつきを抑える
        if (
            not force
            and current_offset * target_offset < 0
            and abs(target_offset) < AT_REVERSE_GUARD_COUNTS
        ):
            target_value = AT_NEUTRAL_VALUE

        # 微小なAT値変化は前回値を保持（整数境界付近の往復更新を防ぐ）
        if (
            not force
            and AT_VALUE_HYSTERESIS_COUNTS > 0
            and abs(target_value - self._last_sent_value) < AT_VALUE_HYSTERESIS_COUNTS
        ):
            target_value = self._last_sent_value

        speed_for_compare = (target_value - AT_NEUTRAL_VALUE) / max(1.0, float(AT_SPEED_SPAN))
        now = time.monotonic()

        if (
            not force
            and self._last_sent_speed is not None
            and MOTOR_MIN_SEND_INTERVAL > 0.0
        ):
            dt = now - self._last_sent_at
            delta = abs(speed_for_compare - self._last_sent_speed)
            if dt < MOTOR_MIN_SEND_INTERVAL and delta < MOTOR_FORCE_SEND_DELTA:
                return

        if not force and SEND_ONLY_ON_CHANGE and self._last_sent_speed is not None:
            unchanged = target_value == self._last_sent_value
            refresh_enabled = COMMAND_REFRESH_SEC > 0.0
            fresh_enough = refresh_enabled and ((now - self._last_sent_at) < COMMAND_REFRESH_SEC)
            if unchanged and fresh_enough:
                return
            if unchanged and not refresh_enabled:
                return

        self._send(_build_velocity_cmd_value(self.motor_id, target_value))
        self._last_sent_speed = speed_for_compare
        self._last_sent_value = target_value
        self._last_sent_at = now

    def stop(self) -> None:
        self.set_velocity(0.0, force=True)


# ============================================================
# メカナムホイール運動学
# ============================================================

class MecanumDrive:
    """
    4輪メカナムホイールの運動学計算 + モーター制御

    ホイール番号と回転方向 (ローラー傾き 45°):
      v_FL =  vx - vy - ω*(L+W)    (正転 = 前方)
      v_FR =  vx + vy + ω*(L+W)    (正転 = 前方, 取付方向で反転)
      v_RL =  vx + vy - ω*(L+W)
      v_RR =  vx - vy + ω*(L+W)    (取付方向で反転)
    """

    def __init__(self, bus: SerialATBus):
        self.motors = {
            name: EDULITE05(bus, mid) for name, mid in MOTOR_ID.items()
        }
        lw = WHEEL_BASE_HALF_L + WHEEL_BASE_HALF_W
        self._lw = lw
        self._last_debug_at = 0.0

    def enable_all(self) -> None:
        for _ in range(ENABLE_RETRY_COUNT):
            for m in self.motors.values():
                m.enable()
            time.sleep(ENABLE_RETRY_INTERVAL)
        print("全モーター イネーブル完了")

    def disable_all(self) -> None:
        for m in self.motors.values():
            m.disable()
        print("全モーター ディスエーブル完了")

    def drive(self, vx: float, vy: float, omega: float) -> None:
        """
        速度指令を計算してモーターに送信

        Parameters
        ----------
        vx    : 前後 [-1, +1]  (+1 = 前進)
        vy    : 左右 [-1, +1]  (+1 = 右ストレーフ)
        omega : 旋回 [-1, +1]  (+1 = 右旋回)
        """
        lw = self._lw

        fl = vx - vy - lw * omega
        fr = vx + vy + lw * omega
        rl = vx + vy - lw * omega
        rr = vx - vy + lw * omega

        # 最大値で正規化 (|値| > 1 にならないよう)
        max_val = max(abs(fl), abs(fr), abs(rl), abs(rr))
        if max_val > 1.0:
            inv = 1.0 / max_val
            fl *= inv
            fr *= inv
            rl *= inv
            rr *= inv

        # シリアルAT速度値へ変換
        fl_out = fl
        fr_out = -fr
        rl_out = rl
        rr_out = -rr

        self.motors["FL"].set_velocity(fl_out)
        self.motors["FR"].set_velocity(fr_out)
        self.motors["RL"].set_velocity(rl_out)
        self.motors["RR"].set_velocity(rr_out)

        if DEBUG_INPUT:
            now = time.monotonic()
            if now - self._last_debug_at >= 1.0:
                self._last_debug_at = now
                print(
                    "[DEBUG] "
                    f"vx={vx:+.2f} vy={vy:+.2f} omega={omega:+.2f} "
                    f"| FL={fl_out:+.2f} FR={fr_out:+.2f} RL={rl_out:+.2f} RR={rr_out:+.2f}"
                )

    def stop(self) -> None:
        for m in self.motors.values():
            m.stop()


# ============================================================
# DualSense コントローラー入力
# ============================================================

class DualSenseController:
    """
    DualSense (Bluetooth) の入力を pygame で読み取るクラス

    pygame の軸番号 (Linux/Ubuntu):
      Axis 0: 左スティック X
      Axis 1: 左スティック Y  (上方向が -1)
      Axis 2: 右スティック X
      Axis 3: 右スティック Y

    ボタン番号:
      9: OPTIONSボタン (終了)
    """

    OPTIONS_BTN = 9
    # コントローラーが見つかるまで待機する最大秒数
    WAIT_TIMEOUT = 30
    NAME_CANDIDATES = ("DualSense", "Wireless Controller")
    EVDEV_QUIT_BTNS = ("BTN_START", "BTN_MODE")
    PYGAME_AXIS_LEFT_X = int(os.getenv("PYGAME_AXIS_LEFT_X", "0"))
    PYGAME_AXIS_LEFT_Y = int(os.getenv("PYGAME_AXIS_LEFT_Y", "1"))
    # ps5_to_robot_Pi.py で動作実績のある Ubuntu DualSense マッピング
    PYGAME_AXIS_RIGHT_X = int(os.getenv("PYGAME_AXIS_RIGHT_X", "2"))
    # 予備軸は既定で完全無効（必要時のみ PYGAME_USE_RIGHT_X_ALT=1 で明示的に使用）
    PYGAME_USE_RIGHT_X_ALT = _env_flag("PYGAME_USE_RIGHT_X_ALT", "0")
    PYGAME_AXIS_RIGHT_X_ALT = int(os.getenv("PYGAME_AXIS_RIGHT_X_ALT", "-1"))
    PYGAME_AXIS_L2 = int(os.getenv("PYGAME_AXIS_L2", "5"))
    PYGAME_AXIS_R2 = int(os.getenv("PYGAME_AXIS_R2", "4"))
    PYGAME_BTN_L2 = int(os.getenv("PYGAME_BTN_L2", "6"))
    PYGAME_BTN_R2 = int(os.getenv("PYGAME_BTN_R2", "7"))
    ADDR_FILE = Path(__file__).with_name(".dualsense_addr")
    EVDEV_AXIS_CANDIDATES = {
        "left_x": ("ABS_X",),
        "left_y": ("ABS_Y",),
        "right_x": ("ABS_RX",),
    }
    EVDEV_TRIGGER_CANDIDATES = {
        "l2": ("ABS_RZ",),
        "r2": ("ABS_Z",),
    }

    def __init__(self):
        # SSH 等で DISPLAY が無い場合でも pygame が初期化できるようにする
        if "DISPLAY" not in os.environ and "SDL_VIDEODRIVER" not in os.environ:
            os.environ["SDL_VIDEODRIVER"] = "dummy"

        pygame.init()
        pygame.joystick.init()
        self.backend = ""
        self.joy = None
        self.pygame_axis_count = 0
        self.pygame_button_count = 0
        self.evdev_device = None
        self.evdev_abs_ranges: dict[int, tuple[int, int]] = {}
        self.evdev_axis_codes: dict[str, int] = {}
        self.evdev_trigger_codes: dict[str, int] = {}
        self.evdev_trigger_ranges: dict[int, tuple[int, int]] = {}
        self.evdev_state = {
            "left_x": 0.0,
            "left_y": 0.0,
            "right_x": 0.0,
        }
        self.trigger_state = {
            "l2": 0.0,
            "r2": 0.0,
        }
        self.trigger_baseline = {
            "l2": 0.0,
            "r2": 0.0,
        }
        self.axis_offsets = {
            "left_x": 0.0,
            "left_y": 0.0,
            "right_x": 0.0,
        }
        self.right_x_alt_offset = 0.0
        self.evdev_quit_codes: set[int] = set()
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_omega = 0.0
        self._last_read_at = time.monotonic()
        self._trigger_latched = {
            "l2": False,
            "r2": False,
        }

        # DualSense が接続されるまで最大 WAIT_TIMEOUT 秒待機する
        print(f"[手順 1/3] DualSenseコントローラーを待機中...")
        print(f"           まだ繋がっていない場合は PS ボタンを押してください。")
        print(f"           （{self.WAIT_TIMEOUT}秒以内に接続されない場合は終了します）")

        deadline = time.monotonic() + self.WAIT_TIMEOUT
        reconnect_attempted = False
        while True:
            if self._init_pygame_controller():
                break

            if self._init_evdev_controller():
                break

            if not reconnect_attempted and self._connect_saved_dualsense():
                reconnect_attempted = True
                time.sleep(1.0)
                pygame.joystick.quit()
                pygame.joystick.init()
                continue

            if time.monotonic() > deadline:
                visible_pygame = self._list_pygame_joysticks()
                visible_evdev = self._list_evdev_devices()
                visible_pygame_text = ", ".join(visible_pygame) if visible_pygame else "なし"
                visible_evdev_text = ", ".join(visible_evdev) if visible_evdev else "なし"
                raise RuntimeError(
                    "\n"
                    "  コントローラーが見つかりませんでした。\n"
                    "  ---- 確認事項 ----\n"
                    "  1. DualSense の PS ボタンを押して電源が入っているか確認\n"
                    "  2. 初回ペアリングが済んでいるか確認 (pair_dualsense.sh を実行)\n"
                    "  3. DUALSENSE_MAC_ADDRESS を設定している場合は値を確認\n"
                    "  4. 他の機器と接続済みの場合は一度電源を切り直す\n"
                    f"  5. pygame から見えている入力デバイス: {visible_pygame_text}\n"
                    f"  6. evdev から見えている入力デバイス: {visible_evdev_text}\n"
                )
            pygame.event.pump()
            time.sleep(0.5)
            pygame.joystick.quit()
            pygame.joystick.init()

        self._calibrate_center_offsets()

    def _connect_saved_dualsense(self) -> bool:
        address = self._get_dualsense_address()
        if not address:
            return False

        try:
            result = subprocess.run(
                ["bluetoothctl", "connect", address],
                capture_output=True,
                text=True,
                timeout=10,
                check=False,
            )
        except (OSError, subprocess.SubprocessError):
            return False

        combined_output = f"{result.stdout}\n{result.stderr}"
        return any(
            token in combined_output
            for token in (
                "Connection successful",
                "already connected",
                "org.bluez.Error.AlreadyConnected",
            )
        )

    def _get_dualsense_address(self) -> str:
        if DUALSENSE_MAC_ADDRESS:
            return DUALSENSE_MAC_ADDRESS

        if not self.ADDR_FILE.exists():
            return ""

        return self.ADDR_FILE.read_text(encoding="utf-8").strip()

    def _list_pygame_joysticks(self) -> list[str]:
        names = []
        for index in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(index)
            joystick.init()
            names.append(joystick.get_name())
        return names

    def _find_pygame_controller_index(self) -> int | None:
        count = pygame.joystick.get_count()
        if count == 0:
            return None

        fallback_index = 0
        fallback_axes = -1
        for index in range(count):
            joystick = pygame.joystick.Joystick(index)
            joystick.init()
            name = joystick.get_name()
            if any(candidate in name for candidate in self.NAME_CANDIDATES):
                return index

            # DualSense 名で見つからない環境では、軸数が最も多いデバイスを優先する
            axis_count = joystick.get_numaxes()
            if axis_count > fallback_axes:
                fallback_axes = axis_count
                fallback_index = index

        return fallback_index

    def _init_pygame_controller(self) -> bool:
        joy_index = self._find_pygame_controller_index()
        if joy_index is None:
            return False

        self.joy = pygame.joystick.Joystick(joy_index)
        self.joy.init()
        self.pygame_axis_count = self.joy.get_numaxes()
        self.pygame_button_count = self.joy.get_numbuttons()
        self.backend = "pygame"
        print(
            "           ✓ コントローラー接続 (pygame): "
            f"{self.joy.get_name()} / axes={self.pygame_axis_count}"
            f" / map(LX={self.PYGAME_AXIS_LEFT_X}, LY={self.PYGAME_AXIS_LEFT_Y}, "
            f"RX={self.PYGAME_AXIS_RIGHT_X}, RX_ALT={self.PYGAME_AXIS_RIGHT_X_ALT}, "
            f"use_alt={'ON' if self.PYGAME_USE_RIGHT_X_ALT else 'OFF'}, "
            f"L2={self.PYGAME_AXIS_L2}|btn={self.PYGAME_BTN_L2}, "
            f"R2={self.PYGAME_AXIS_R2}|btn={self.PYGAME_BTN_R2})"
        )
        return True

    def _get_pygame_axis(self, axis_index: int) -> float:
        if self.joy is None:
            return 0.0
        if axis_index < 0 or axis_index >= self.pygame_axis_count:
            return 0.0
        return self.joy.get_axis(axis_index)

    def _is_valid_pygame_axis(self, axis_index: int) -> bool:
        return self.joy is not None and 0 <= axis_index < self.pygame_axis_count

    def _is_valid_pygame_button(self, button_index: int) -> bool:
        return self.joy is not None and 0 <= button_index < self.pygame_button_count

    def _get_pygame_button(self, button_index: int) -> bool:
        if not self._is_valid_pygame_button(button_index):
            return False
        return bool(self.joy.get_button(button_index))

    @staticmethod
    def _apply_center_offset(value: float, offset: float) -> float:
        corrected = value - offset
        return max(-1.0, min(1.0, corrected))

    @staticmethod
    def _apply_command_deadband(value: float) -> float:
        if abs(value) < COMMAND_DEADBAND:
            return 0.0
        return value

    def _trigger_delta_latched(self, name: str, raw_value: float) -> bool:
        baseline = self.trigger_baseline[name]
        delta = abs(raw_value - baseline)
        press_th = TRIGGER_DELTA_THRESHOLD + TRIGGER_HYSTERESIS
        release_th = max(0.0, TRIGGER_DELTA_THRESHOLD - TRIGGER_HYSTERESIS)

        if self._trigger_latched[name]:
            if delta <= release_th:
                self._trigger_latched[name] = False
        else:
            if delta >= press_th:
                self._trigger_latched[name] = True

        return self._trigger_latched[name]

    def _is_l2_pressed(self) -> bool:
        if self.backend == "evdev":
            return self._trigger_delta_latched("l2", self.trigger_state["l2"])

        # ボタンが取得できる環境では誤軸マッピングの影響を避けるためボタン判定を優先
        if self._is_valid_pygame_button(self.PYGAME_BTN_L2):
            return self._get_pygame_button(self.PYGAME_BTN_L2)

        if self._is_valid_pygame_axis(self.PYGAME_AXIS_L2):
            raw = self._get_pygame_axis(self.PYGAME_AXIS_L2)
            return self._trigger_delta_latched("l2", raw)

        return False

    def _is_r2_pressed(self) -> bool:
        if self.backend == "evdev":
            return self._trigger_delta_latched("r2", self.trigger_state["r2"])

        # ボタンが取得できる環境では誤軸マッピングの影響を避けるためボタン判定を優先
        if self._is_valid_pygame_button(self.PYGAME_BTN_R2):
            return self._get_pygame_button(self.PYGAME_BTN_R2)

        if self._is_valid_pygame_axis(self.PYGAME_AXIS_R2):
            raw = self._get_pygame_axis(self.PYGAME_AXIS_R2)
            return self._trigger_delta_latched("r2", raw)

        return False

    def _normalize_trigger_value(self, code: int, value: int) -> float:
        minimum, maximum = self.evdev_trigger_ranges.get(code, (0, 0))
        if maximum <= minimum:
            return 0.0
        normalized = (value - minimum) / float(maximum - minimum)
        return max(0.0, min(1.0, normalized))

    def _calibrate_center_offsets(self) -> None:
        if INPUT_CENTER_CALIB_SEC <= 0.0:
            return

        sample_count = max(1, int(INPUT_CENTER_CALIB_SEC * 100.0))
        sum_left_x = 0.0
        sum_left_y = 0.0
        sum_right_x = 0.0
        sum_right_x_alt = 0.0
        sum_l2 = 0.0
        sum_r2 = 0.0

        print(f"           - 入力センター補正を実施中 ({INPUT_CENTER_CALIB_SEC:.1f}秒)")
        for _ in range(sample_count):
            if self.backend == "evdev":
                self._read_evdev_once_for_calibration()
                lx = self.evdev_state["left_x"]
                ly = self.evdev_state["left_y"]
                rx = self.evdev_state["right_x"]
                l2 = self.trigger_state["l2"]
                r2 = self.trigger_state["r2"]
            else:
                pygame.event.pump()
                lx = self._get_pygame_axis(self.PYGAME_AXIS_LEFT_X)
                ly = self._get_pygame_axis(self.PYGAME_AXIS_LEFT_Y)
                rx = self._get_pygame_axis(self.PYGAME_AXIS_RIGHT_X)
                rx_alt = self._get_pygame_axis(self.PYGAME_AXIS_RIGHT_X_ALT)
                l2 = self._get_pygame_axis(self.PYGAME_AXIS_L2)
                r2 = self._get_pygame_axis(self.PYGAME_AXIS_R2)

            sum_left_x += lx
            sum_left_y += ly
            sum_right_x += rx
            sum_right_x_alt += rx_alt if self.backend == "pygame" else rx
            sum_l2 += l2
            sum_r2 += r2
            time.sleep(0.01)

        self.axis_offsets["left_x"] = sum_left_x / sample_count
        self.axis_offsets["left_y"] = sum_left_y / sample_count
        self.axis_offsets["right_x"] = sum_right_x / sample_count
        self.right_x_alt_offset = sum_right_x_alt / sample_count
        self.trigger_baseline["l2"] = sum_l2 / sample_count
        self.trigger_baseline["r2"] = sum_r2 / sample_count
        print(
            "           ✓ 入力センター補正: "
            f"LX={self.axis_offsets['left_x']:+.3f} "
            f"LY={self.axis_offsets['left_y']:+.3f} "
            f"RX={self.axis_offsets['right_x']:+.3f} "
            f"RX_ALT={self.right_x_alt_offset:+.3f} "
            f"L2={self.trigger_baseline['l2']:+.3f} "
            f"R2={self.trigger_baseline['r2']:+.3f}"
        )

    def _read_evdev_once_for_calibration(self) -> None:
        if self.evdev_device is None or ecodes is None:
            return
        try:
            for event in self.evdev_device.read():
                if event.type != ecodes.EV_ABS:
                    continue
                for axis_name, code in self.evdev_axis_codes.items():
                    if event.code == code:
                        self.evdev_state[axis_name] = self._normalize_evdev_axis(axis_name, event.value)
                for trigger_name, code in self.evdev_trigger_codes.items():
                    if event.code == code:
                        self.trigger_state[trigger_name] = self._normalize_trigger_value(code, event.value)
        except BlockingIOError:
            pass

    def _list_evdev_devices(self) -> list[str]:
        if InputDevice is None:
            return []

        names = []
        for path in list_devices():
            try:
                device = InputDevice(path)
                names.append(device.name)
                device.close()
            except OSError:
                continue
        return names

    def _find_evdev_device_path(self) -> str | None:
        if InputDevice is None or ecodes is None:
            return None

        for path in list_devices():
            try:
                device = InputDevice(path)
            except OSError:
                continue

            capabilities = device.capabilities()
            has_abs = ecodes.EV_ABS in capabilities
            has_key = ecodes.EV_KEY in capabilities
            name_matches = any(candidate in device.name for candidate in self.NAME_CANDIDATES)
            if has_abs and has_key and name_matches:
                device.close()
                return path

            device.close()

        return None

    def _init_evdev_controller(self) -> bool:
        if InputDevice is None or ecodes is None:
            return False

        device_path = self._find_evdev_device_path()
        if device_path is None:
            return False

        self.evdev_device = InputDevice(device_path)
        self.evdev_axis_codes = {}
        self.evdev_abs_ranges = {}
        self.evdev_trigger_codes = {}
        self.evdev_trigger_ranges = {}
        capabilities = self.evdev_device.capabilities(absinfo=True)
        abs_caps = capabilities.get(ecodes.EV_ABS, [])
        code_by_name = {
            ecodes.ABS[code]: (code, absinfo)
            for code, absinfo in abs_caps
            if code in ecodes.ABS
        }

        for axis_name, code_names in self.EVDEV_AXIS_CANDIDATES.items():
            for code_name in code_names:
                if code_name not in code_by_name:
                    continue
                code, absinfo = code_by_name[code_name]
                self.evdev_axis_codes[axis_name] = code
                self.evdev_abs_ranges[code] = (absinfo.min, absinfo.max)
                break

        for trigger_name, code_names in self.EVDEV_TRIGGER_CANDIDATES.items():
            for code_name in code_names:
                if code_name not in code_by_name:
                    continue
                code, absinfo = code_by_name[code_name]
                self.evdev_trigger_codes[trigger_name] = code
                self.evdev_trigger_ranges[code] = (absinfo.min, absinfo.max)
                break

        for btn_name in self.EVDEV_QUIT_BTNS:
            code = getattr(ecodes, btn_name, None)
            if code is not None:
                self.evdev_quit_codes.add(code)

        self.backend = "evdev"
        print(f"           ✓ コントローラー接続 (evdev): {self.evdev_device.name}")
        return True

    def _normalize_evdev_axis(self, axis_name: str, value: int) -> float:
        code = self.evdev_axis_codes.get(axis_name)
        if code is None:
            return 0.0

        minimum, maximum = self.evdev_abs_ranges[code]
        if maximum <= minimum:
            return 0.0

        center = (minimum + maximum) / 2.0
        half_span = (maximum - minimum) / 2.0
        normalized = (value - center) / half_span
        return max(-1.0, min(1.0, normalized))

    def _read_evdev(self) -> tuple[float, float, float, bool]:
        quit_req = False
        if self.evdev_device is None or ecodes is None:
            return 0.0, 0.0, 0.0, False

        try:
            for event in self.evdev_device.read():
                if event.type == ecodes.EV_KEY and event.value == 1:
                    if event.code in self.evdev_quit_codes:
                        quit_req = True
                elif event.type == ecodes.EV_ABS:
                    for axis_name, code in self.evdev_axis_codes.items():
                        if event.code == code:
                            self.evdev_state[axis_name] = self._normalize_evdev_axis(axis_name, event.value)
                    for trigger_name, code in self.evdev_trigger_codes.items():
                        if event.code == code:
                            self.trigger_state[trigger_name] = self._normalize_trigger_value(code, event.value)
        except BlockingIOError:
            pass

        left_x = self._apply_center_offset(self.evdev_state["left_x"], self.axis_offsets["left_x"])
        left_y = self._apply_center_offset(self.evdev_state["left_y"], self.axis_offsets["left_y"])
        right_x = self._apply_center_offset(self.evdev_state["right_x"], self.axis_offsets["right_x"])

        left_x = self._deadzone(left_x)
        left_y = self._deadzone(left_y)
        right_x = self._deadzone(right_x)
        vx = left_y
        vy = left_x
        omega = right_x

        if not self._is_l2_pressed():
            vx, vy = 0.0, 0.0
        if not self._is_r2_pressed():
            omega = 0.0

        vx, vy, omega = self._apply_response(vx, vy, omega)
        vx, vy, omega = self._apply_slew_limit(vx, vy, omega)
        vx = self._apply_command_deadband(vx)
        vy = self._apply_command_deadband(vy)
        omega = self._apply_command_deadband(omega)
        return vx, vy, omega, quit_req

    @staticmethod
    def _deadzone(value: float) -> float:
        if abs(value) < DEADZONE:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - DEADZONE) / (1.0 - DEADZONE)

    @staticmethod
    def _shape_axis(value: float, exponent: float, gain: float = 1.0) -> float:
        exponent = max(1.0, exponent)
        magnitude = min(1.0, abs(value))
        shaped = magnitude ** exponent
        signed = shaped if value >= 0.0 else -shaped
        return max(-1.0, min(1.0, signed * gain))

    def _apply_response(self, vx: float, vy: float, omega: float) -> tuple[float, float, float]:
        vx = self._shape_axis(vx, INPUT_CURVE_EXPONENT, TRANSLATION_GAIN)
        vy = self._shape_axis(vy, INPUT_CURVE_EXPONENT, TRANSLATION_GAIN)
        omega = self._shape_axis(omega, INPUT_CURVE_EXPONENT, ROTATION_GAIN)
        return vx, vy, omega

    def _apply_slew_limit(self, vx: float, vy: float, omega: float) -> tuple[float, float, float]:
        if INPUT_SLEW_RATE <= 0.0:
            self._last_vx, self._last_vy, self._last_omega = vx, vy, omega
            self._last_read_at = time.monotonic()
            return vx, vy, omega

        now = time.monotonic()
        dt = max(0.001, now - self._last_read_at)
        self._last_read_at = now
        max_delta = INPUT_SLEW_RATE * dt

        def limit_step(current: float, target: float) -> float:
            delta = target - current
            if delta > max_delta:
                return current + max_delta
            if delta < -max_delta:
                return current - max_delta
            return target

        vx_limited = limit_step(self._last_vx, vx)
        vy_limited = limit_step(self._last_vy, vy)
        omega_limited = limit_step(self._last_omega, omega)

        self._last_vx, self._last_vy, self._last_omega = vx_limited, vy_limited, omega_limited
        return vx_limited, vy_limited, omega_limited

    def read(self) -> tuple[float, float, float, bool]:
        """
        Returns
        -------
        (vx, vy, omega, quit_requested)
        """
        if self.backend == "evdev":
            return self._read_evdev()

        quit_req = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                quit_req = True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == self.OPTIONS_BTN:
                    quit_req = True

        left_x = self._apply_center_offset(
            self._get_pygame_axis(self.PYGAME_AXIS_LEFT_X),
            self.axis_offsets["left_x"],
        )
        left_y = self._apply_center_offset(
            self._get_pygame_axis(self.PYGAME_AXIS_LEFT_Y),
            self.axis_offsets["left_y"],
        )
        right_x = self._apply_center_offset(
            self._get_pygame_axis(self.PYGAME_AXIS_RIGHT_X),
            self.axis_offsets["right_x"],
        )
        # 固定軸運用を既定化。必要時のみ明示指定で予備軸を使用する。
        if self.PYGAME_USE_RIGHT_X_ALT and self._is_valid_pygame_axis(self.PYGAME_AXIS_RIGHT_X_ALT):
            right_x = self._apply_center_offset(
                self._get_pygame_axis(self.PYGAME_AXIS_RIGHT_X_ALT),
                self.right_x_alt_offset,
            )

        left_x = self._deadzone(left_x)
        left_y = self._deadzone(left_y)
        right_x = self._deadzone(right_x)

        vx    =  left_y   # 実機向け: 前後を反転
        vy    =  left_x   # 実機向け: 左右は正方向
        omega =  right_x  # 右方向が右旋回

        if not self._is_l2_pressed():
            vx, vy = 0.0, 0.0
        if not self._is_r2_pressed():
            omega = 0.0

        vx, vy, omega = self._apply_response(vx, vy, omega)
        vx, vy, omega = self._apply_slew_limit(vx, vy, omega)
        vx = self._apply_command_deadband(vx)
        vy = self._apply_command_deadband(vy)
        omega = self._apply_command_deadband(omega)

        return vx, vy, omega, quit_req

    def close(self) -> None:
        if self.evdev_device is not None:
            self.evdev_device.close()
        pygame.quit()


# ============================================================
# メインループ
# ============================================================

def main() -> None:
    controller = None
    robot = None
    transport = None

    print("=" * 50)
    print("  メカナムホイール 4WD ラジコン 起動中...")
    print("=" * 50)
    if DEBUG_INPUT:
        print("  [DEBUG] 入力デバッグ表示: ON")
    else:
        print("  [DEBUG] 入力デバッグ表示: OFF (有効化: DEBUG_INPUT=1)")
    print(
        "  [設定] "
        f"speed_limit={AT_SPEED_PERCENT:.0f}% "
        f"translation_gain={TRANSLATION_GAIN:.2f} "
        f"rotation_gain={ROTATION_GAIN:.2f} "
        f"curve_exp={INPUT_CURVE_EXPONENT:.2f} "
        f"slew_rate={INPUT_SLEW_RATE:.2f} "
        f"min_send={MOTOR_MIN_SEND_INTERVAL:.3f}s "
        f"zero_hold={MOTOR_ZERO_HOLD_BAND:.2f}"
    )

    try:
        # ステップ 1: DualSense 接続待機
        controller = DualSenseController()

        # ステップ 2: シリアル接続
        print(f"[手順 2/3] シリアル接続中...")
        try:
            transport = SerialATBus()
        except Exception as e:
            raise RuntimeError(
                "\n"
                "  シリアル接続に失敗しました。\n"
                "  ---- 確認事項 ----\n"
                "  1. USBシリアルアダプタが接続されているか\n"
                "  2. 権限不足なら dialout グループに所属しているか\n"
                "  3. 必要なら SERIAL_PORT を明示指定して起動する\n"
                f"\n  詳細: {e}\n"
            )
        print(f"           ✓ シリアル接続準備完了")

        # ステップ 3: モーター起動
        print(f"[手順 3/3] モーターをイネーブルにしています...")
        robot = MecanumDrive(transport)
        robot.enable_all()
        # イネーブル直後に必ず停止コマンドを送って安全側に倒す
        robot.stop()
        print(f"           ✓ 全モーター起動完了")

        print()
        print("=" * 50)
        print("  操作準備完了")
        print()
        print("  左スティック ↑↓  : 前進 / 後退")
        print("  左スティック ←→  : 左右 平行移動")
        print("  右スティック ←→  : 左旋回 / 右旋回")
        print("  OPTIONS ボタン   : 終了")
        print()
        print("=" * 50)
        print("                Get Ready ROX ON!")
        print("=" * 50)

        interval = 1.0 / CONTROL_HZ
        startup_guard_until = time.monotonic() + STARTUP_STOP_SEC
        while True:
            t_start = time.monotonic()

            vx, vy, omega, quit_req = controller.read()

            if quit_req:
                print("\n  OPTIONS ボタンが押されました。終了します...")
                break

            if time.monotonic() < startup_guard_until:
                robot.stop()
                elapsed = time.monotonic() - t_start
                sleep_time = interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                continue

            robot.drive(vx, vy, omega)

            elapsed = time.monotonic() - t_start
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n  Ctrl+C で停止しました。")
    except RuntimeError as e:
        print(f"\n[エラー] {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"\n[予期しないエラー] {e}", file=sys.stderr)
        raise
    finally:
        if robot:
            try:
                robot.stop()
                time.sleep(0.1)
                robot.disable_all()
            except Exception as e:
                print(f"[WARN] 停止処理でエラー: {e}")
        if transport:
            transport.close()
        if controller:
            controller.close()
        print("\n  シャットダウン完了。")


if __name__ == "__main__":
    main()