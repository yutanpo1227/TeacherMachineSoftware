import sensor
import time
from fpioa_manager import fm
from machine import UART

# カメラ画像の右・左を別々の鏡として処理し、検出結果を1行のUARTデータにする。
# Arduino側は lib/Camera/Camera.cpp で受信し、機体の走行と姿勢制御に使う。
# --- 高速化 ---
USE_CAMERA_DUAL_BUFF = True
BLOB_X_STRIDE = 2
BLOB_Y_STRIDE = 2
DRAW_OVERLAY = True
PRINT_EVERY_N_FRAMES = 1

# --- V字ミラー設定 ---
# QVGA (320×240) の画像では右側に前方、左側に後方が映る。
# 実機で前後を確認するときは、片側だけにボールを置いてROI表示を見る。
ROI_FRONT = (150, 0, 160, 240)   # x, y, w, h
ROI_BACK  = (0,   0, 150, 240)

# 映像のy軸に割り当てる視野角。既知の方位に対象を置いて校正する。
MIRROR_VIEW_FOV_DEG = 80.0

# 角度が逆に動く場合は +1 / -1 を入れ替える。
# 後方はV字ミラーで前方と反対向きに写るので、前方と逆符号にする。
FRONT_ANGLE_SIGN = 1
BACK_ANGLE_SIGN  = -1

# ロボット制御用:
# 前方正面を90度、後方正面を270度として送る例
FRONT_BASE_ANGLE = 90.0
BACK_BASE_ANGLE  = 270.0

# ROI内で「正面=0度」にしたい座標。
# 角度計算は常にy座標を使う。
# NoneならROIの中央を使う。
FRONT_CENTER_POS = None
BACK_CENTER_POS = None

# --- 色閾値 ---
ORANGE_THRESHOLDS = [(0, 100, 23, 49, 9, 127)]
# LABの閾値は、実際に使う照明でミラー越しのゴールを見て調整する。
YELLOW_THRESHOLDS = [(43, 79, -20, 63, 12, 127)]
#YELLOW_THRESHOLDS = [(0, 0, 0, 0, 0, 0)]
BLUE_THRESHOLDS = [(0, 100, 10, 43, -127, -19)]

# --- blob条件 ---
BLOB_PIXELS_THRESHOLD = 80
BLOB_AREA_THRESHOLD = 80
BLOB_MERGE_MARGIN = 8

# 前後の両方で検出したとき、現在の向きをどれくらい優先するか。
# 例: 1.25 なら、反対側の面積が現在側の1.25倍を超えたときだけ切り替える。
VIEW_SWITCH_PIXELS_RATIO = 1.25

# --- カメラ制御 ---
USE_AUTO_GAIN = False
FIXED_GAIN_DB = 0
USE_AUTO_WHITEBAL = False

# --- UART ---
# カメラのTXからArduinoのSerial1 RXへ前後別の検出結果を送る。
fm.register(34, fm.fpioa.UART1_RX, force=True)
fm.register(35, fm.fpioa.UART1_TX, force=True)
uart = UART(UART.UART1, 115200, 8, None, 1, timeout=1000, read_buf_len=4096)


def axis_angle_deg(pos, size, fov_deg, center_pos=None):
    """
    指定した中心を0度、両端を+-fov_deg/2として線形に角度へ変換する。
    視野角と中心は実機校正値であり、この関数だけでは実距離は求められない。
    """
    max_pos = size - 1
    if max_pos <= 0:
        return 0.0

    if center_pos is None:
        center_pos = max_pos / 2.0

    half_fov = fov_deg / 2.0

    if pos < center_pos:
        distance_to_edge = center_pos
    else:
        distance_to_edge = max_pos - center_pos

    if distance_to_edge <= 0:
        return 0.0

    angle = (pos - center_pos) * half_fov / distance_to_edge

    if angle < -half_fov:
        return -half_fov
    if angle > half_fov:
        return half_fov
    return angle


def _sensor_reset_with_dual_buff():
    # ファームウェアによってdual_buffや周波数指定を使えないため、使える設定へ戻す。
    if USE_CAMERA_DUAL_BUFF:
        try:
            sensor.reset(freq=20000000, dual_buff=True)
            return
        except TypeError:
            pass
        except Exception as e:
            print("dual_buff reset failed:", e)

    try:
        sensor.reset(freq=20000000)
    except TypeError:
        sensor.reset()


def safe_sensor_setup():
    # 撮影時の明るさや色の自動調整を設定し、色検出が変動しにくいようにする。
    try:
        if USE_AUTO_GAIN:
            sensor.set_auto_gain(True)
        else:
            sensor.set_auto_gain(False, gain_db=FIXED_GAIN_DB)
        print("auto_gain: OK")
    except Exception as e:
        print("auto_gain failed:", e)

    try:
        sensor.set_auto_whitebal(USE_AUTO_WHITEBAL)
        print("auto_whitebal: OK")
    except Exception as e:
        print("auto_whitebal failed:", e)


def find_best_blob_in_roi(img, roi, thresholds):
    # ROI内で閾値に合う色のかたまりを探し、最大のものを代表として使う。
    # pixels_thresholdとarea_thresholdは小さなノイズを除くための下限。
    blobs = img.find_blobs(
        thresholds, roi=roi, pixels_threshold=BLOB_PIXELS_THRESHOLD,
        area_threshold=BLOB_AREA_THRESHOLD, merge=True,
        margin=BLOB_MERGE_MARGIN, x_stride=BLOB_X_STRIDE, y_stride=BLOB_Y_STRIDE)
    return max(blobs, key=lambda b: b.pixels()) if blobs else None


def detect_view(img, name, roi, base_angle, angle_sign, center_pos,
                thresholds=ORANGE_THRESHOLDS):
    """
    1つのミラー領域から指定色を探し、ロボット基準角度に変換する。

    name:
      "F" = front
      "B" = back

    戻り値:
      None または dict
    """
    blob = find_best_blob_in_roi(img, roi, thresholds)
    if blob is None:
        return None

    _, roi_y, _, roi_h = roi
    # この取り付けでは機体の左右が画像のy軸に沿う。
    local_y = blob.cy() - roi_y
    if center_pos is None:
        center_pos = (roi_h - 1) / 2.0
    local_angle = axis_angle_deg(local_y, roi_h, MIRROR_VIEW_FOV_DEG, center_pos)
    robot_angle = (base_angle + angle_sign * local_angle) % 360.0

    return {
        "name": name,
        "blob": blob,
        "roi": roi,
        "center_pos": center_pos,
        "robot_angle": robot_angle,
        "pixels": blob.pixels(),
    }


def choose_single_detection(detections, active_view_name):
    """
    同じ対象について、前後どちらか1つの検出だけを採用する。

    active_view_name:
      前回採用した "F" / "B" / None。

    戻り値:
      (採用した検出dict, 新しいactive_view_name)
    """
    if not detections:
        return None, None

    best = max(detections, key=lambda d: d["pixels"])

    active = None
    if active_view_name is not None:
        for det in detections:
            if det["name"] == active_view_name:
                active = det
                break

    # 両方に候補があるとき、画素数の差が小さければ前回と同じ側を追う。
    if active is not None:
        switching = (best["name"] != active_view_name and
                     best["pixels"] > active["pixels"] * VIEW_SWITCH_PIXELS_RATIO)
        if not switching:
            return active, active_view_name

    return best, best["name"]


def draw_detection(img, det, color):
    blob = det["blob"]
    roi = det["roi"]

    img.draw_rectangle(roi, color=color, thickness=1)
    img.draw_rectangle(blob.rect(), color=color, thickness=2)
    img.draw_cross(blob.cx(), blob.cy(), color=color, thickness=1, size=8)

    # 角度計算で0度として扱う基準線
    x, y, w, h = roi
    center = int(det["center_pos"] + 0.5)
    center_y = y + center
    img.draw_line(x, center_y, x + w, center_y, color=color, thickness=1)


def detect_views(img, thresholds):
    # 同じ色を前方・後方それぞれで検出する。ゴールは両方の結果を送信する。
    return [detect_view(img, name, roi, base, sign, center, thresholds)
            for name, roi, base, sign, center in (
                ("F", ROI_FRONT, FRONT_BASE_ANGLE, FRONT_ANGLE_SIGN, FRONT_CENTER_POS),
                ("B", ROI_BACK, BACK_BASE_ANGLE, BACK_ANGLE_SIGN, BACK_CENTER_POS))]


def detect_object(img, thresholds, active_view_name):
    # ボールの走行制御用には前後どちらか一方を選び、切り替え時の揺れを抑える。
    return choose_single_detection(
        [d for d in detect_views(img, thresholds) if d is not None], active_view_name)


def detection_span(det):
    # ゴールの画像上の横幅を機体から見た角幅へ換算する。見切れた幅は採用しない。
    if det is None:
        return -1.0
    x, y, w, h = det["blob"].rect()
    rx, ry, rw, rh = det["roi"]
    # 左右方向は画像のy軸。奥行き方向（x）の端切れだけなら
    # 左右の端は見えているので、ゴール中心の方向を使える。
    if y <= ry + 2 or y + h >= ry + rh - 2:
        return -1.0
    # このV字ミラーでは画像のy方向が機体の左右方向。
    lo = axis_angle_deg(y - ry, rh, MIRROR_VIEW_FOV_DEG, det["center_pos"])
    hi = axis_angle_deg(y + h - 1 - ry, rh, MIRROR_VIEW_FOV_DEG, det["center_pos"])
    span = round(abs(hi - lo), 1)
    return span if span > 0 else -1.0


def rear_bottom_gap(det):
    # 後方ゴールのx軸側の端を距離調整に使う。画素数なので実距離は別途校正する。
    # y方向の端切れは許容。中央側の端が画面外なら測定不可。
    if det is None or det["name"] != "B":
        return -1
    x, y, w, h = det["blob"].rect()
    rx, ry, rw, rh = det["roi"]
    edge = x + w - 1
    if edge >= rx + rw - 2 or edge < rx:
        return -1
    return rx + rw - edge


def detection_angle(det):
    # 丸めにより360.0になる場合も0.0へ戻す。
    return round(det["robot_angle"], 1) % 360.0 if det is not None else -1.0


def goal_direction(det):
    # 守備で追う後方ゴールには、色の濃淡に左右されにくい矩形の中点を使う。
    if det is None or det["name"] != "B":
        return detection_angle(det)
    _, y, _, h = det["blob"].rect()
    _, ry, _, rh = det["roi"]
    center = y + (h - 1) / 2.0 - ry
    local_angle = axis_angle_deg(center, rh, MIRROR_VIEW_FOV_DEG, det["center_pos"])
    angle = BACK_BASE_ANGLE + BACK_ANGLE_SIGN * local_angle
    return round(angle, 1) % 360.0


def encode_frame(ball, yellow_views, blue_views):
    # 前後別に送るので、前方の同色ノイズが後方自ゴールを隠さない。
    # V3の並び: ボール角度・画素数・前後、黄色の前後の角度と幅、
    # 青色の前後の角度と幅、黄色・青色の後方ゴール下端。改行で1フレームを区切る。
    # 対象が見えない場合の角度・幅・下端は-1、ボール画素数と前後識別は0。
    values = ["%.1f" % detection_angle(ball),
              str(ball["pixels"] if ball is not None else 0),
              str((1 if ball["name"] == "F" else 2) if ball is not None else 0)]
    for det in yellow_views + blue_views:
        values.extend(["%.1f" % goal_direction(det), "%.1f" % detection_span(det)])
    values.extend([str(rear_bottom_gap(yellow_views[1])),
                   str(rear_bottom_gap(blue_views[1]))])
    return "V3," + ",".join(values) + "\n"


def main():
    _sensor_reset_with_dual_buff()

    # RGB画像を320×240で撮り、実機のミラー配置に合わせてROIを切り出す。
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)

    # 必要なら反転
    sensor.set_hmirror(False)
    sensor.set_vflip(False)

    sensor.skip_frames(time=500)
    safe_sensor_setup()
    sensor.skip_frames(time=500)

    clock = time.clock()
    frame_n = 0
    ball_view = None

    while True:
        clock.tick()
        frame_n += 1

        img = sensor.snapshot()

        # 描画色が次の色検出に混ざらないよう、全対象の検出を先に済ませる。
        ball, ball_view = detect_object(img, ORANGE_THRESHOLDS, ball_view)
        yellow_views = detect_views(img, YELLOW_THRESHOLDS)
        blue_views = detect_views(img, BLUE_THRESHOLDS)
        # オーバーレイ描画より先に送信内容を確定し、描画色が次の検出に混じるのを避ける。
        packet = encode_frame(ball, yellow_views, blue_views)
        uart.write(packet)

        if DRAW_OVERLAY:
            for detections, color in (([ball], (255, 128, 0)),
                                      (yellow_views, (255, 255, 0)),
                                      (blue_views, (0, 128, 255))):
                for det in detections:
                    if det is not None:
                        draw_detection(img, det, color)

        if PRINT_EVERY_N_FRAMES <= 1 or frame_n % PRINT_EVERY_N_FRAMES == 0:
            print("fps=%.1f tx=%s" % (clock.fps(), packet.strip()))


if __name__ == "__main__":
    main()
