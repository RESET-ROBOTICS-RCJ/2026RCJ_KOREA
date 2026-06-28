import sensor, time, math, ustruct
from pyb import UART, USB_VCP

# ==================================================
# CONFIG - 常改的放這裡
# ==================================================
usb = USB_VCP()
DEBUG = usb.isconnected()
SEND_UART = True

CENTER_X = 132
CENTER_Y = 120
FRONT_X = 127
FRONT_Y = 121
# ROI = [20, 3, 255, 239]
ROI = [56, 3, 257, 239]

BEAK_W = 10
BEAK_H = 10

CALIB_FILE = "color_thresholds.csv"
SENSOR_FILE = "sensor_settings.csv"

CALIB_TIME_MS = 1000
SENSOR_CALIB_TIME_MS = 1200

BALL_IN_DISTANCE_CM = 12

# ==================================================
# SENSOR AUTO-CALIB SETTINGS
# ==================================================
EXPOSURE_FACTOR = 1.4
RGB_GAIN_FACTOR = (1.0, 1.0, 1.0)
GAIN_DB_FACTOR = 1.0

MIN_EXPOSURE = 3000
MAX_EXPOSURE = 30000

MIN_GAIN_DB = 8.0
MAX_GAIN_DB = 28.0

# ==================================================
# OBJECT CONFIG
# ==================================================
ITEM_BALL = 0
ITEM_YELLOW = 1
ITEM_BLUE = 2

CMD_SENSOR = 0x14
CMD_BALL_YELLOW = 0x15

ITEMS = (
    ("ball",   0x11, (15, 95, 22, 60, 12, 127),   (255, 128, 0)),
    ("yellow", 0x12, (50, 80, -5, 25, 38, 65),     (255, 255, 0)),
    ("blue",   0x13, (0, 100, 0, 15, -52, -30),    (0, 0, 255)),
)

CALIB_MARGINS = (
    (10, 10, 10),
    (4, 6, 6),
    (4, 6, 6),
)

BALL_MIN_A = 35
BALL_MIN_B = 12
BALL_MIN_A_MEAN = 45
BALL_MIN_B_MEAN = 25

YELLOW_MIN_A = -5
YELLOW_MIN_B = 20
YELLOW_ORANGE_AVOID_MARGIN = 10

BLUE_MIN_L = 5
BLUE_MIN_L_MEAN = 10

# 球(橘)跟黃色球門的 a 通道閾值校正後強制留出的間隔，避免互相重疊誤判
COLOR_SEPARATION_MARGIN = 4

# ==================================================
# BALL FALLBACK
# ==================================================
FALLBACK_LAST_SEEN_ANGLE = 20
FALLBACK_LAST_SEEN_DISTANCE_CM = 20
FALLBACK_BALL_ANGLE = 0
FALLBACK_BALL_DISTANCE = 8

# ==================================================
# GLOBAL STATE
# ==================================================
BEAK_X = FRONT_X - (BEAK_W // 2)
BEAK_Y = FRONT_Y - (BEAK_H // 2)
BEAK_RECT = (BEAK_X, BEAK_Y, BEAK_W, BEAK_H)

thresholds = [item[2] for item in ITEMS]

DEFAULT_EXPOSURE = int(21450 * 1.0)
DEFAULT_RGB_GAIN = (64.0, 61.0, 65.0)
DEFAULT_GAIN_DB = 21.0
sensor_exposure = DEFAULT_EXPOSURE
sensor_rgb_gain = DEFAULT_RGB_GAIN
sensor_gain_db = DEFAULT_GAIN_DB

uart = UART(3, 115200, timeout_char=1000)

# ==================================================
# MAIN - 平常常改 sensor.set... 可以在這裡改
# ==================================================
def main():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_windowing(ROI)
    sensor.set_hmirror(True)

    clock = time.clock()

    load_sensor_settings_csv()
    apply_sensor_settings()
    load_thresholds_csv()

    last_ball_angle = None
    last_ball_distance = None

    while True:
        if DEBUG:
            clock.tick()

        check_uart_commands()
        img = sensor.snapshot()

        if DEBUG:
            draw_debug_reference(img)

        bv, ba, bd, ball_blob = find_object(img, ITEM_BALL, 2, 1, 10)
        yv, ya, yd, yellow_blob = find_object(img, ITEM_YELLOW, 100, 100, 20)
        blv, bla, bld, blue_blob = find_object(img, ITEM_BLUE, 100, 100, 20)

        ball_in = False

        if bv:
            ball_in = bd < BALL_IN_DISTANCE_CM and abs(ba) < 10
            print(ba)
            last_ball_angle = ba
            last_ball_distance = bd

            if DEBUG and ball_blob:
                draw_blob(img, ball_blob, ITEM_BALL)

        elif should_use_ball_fallback(last_ball_angle, last_ball_distance):
            bv = True
            ba = FALLBACK_BALL_ANGLE
            bd = FALLBACK_BALL_DISTANCE
            ball_in = True

        if DEBUG:
            if yellow_blob:
                draw_blob(img, yellow_blob, ITEM_YELLOW)
            if blue_blob:
                draw_blob(img, blue_blob, ITEM_BLUE)

        if SEND_UART:
            if DEBUG:
                print(
                    int(ba), int(bd),
                    int(ya), int(yd),
                    int(bla), int(bld),
                    1 if ball_in else 0
                )

            send_objects_packet(
                (bv, ba, bd),
                (yv, ya, yd),
                (blv, bla, bld),
                ball_in
            )

# ==================================================
# DETECTION
# ==================================================
def find_object(img, idx, pixels_threshold, area_threshold, margin):
    blobs = img.find_blobs(
        [thresholds[idx]],
        pixels_threshold=pixels_threshold,
        area_threshold=area_threshold,
        merge=True,
        margin=margin
    )

    if not blobs:
        return False, 0, 255, None

    if idx == ITEM_BLUE:
        blobs = [b for b in blobs if blue_candidate_ok(img, b)]
        if not blobs:
            return False, 0, 255, None

    blob, angle, pixel_dist = blob_output(blobs)
    distance = pixel_to_distance_cm(pixel_dist)

    return True, angle, distance, blob

def pixel_to_distance_cm(pixel_dist):
    return 0.02273 * (pixel_dist ** 2) - 2.273 * pixel_dist + 63.704

def best_blob(blobs):
    best = None
    best_score = -1

    for blob in blobs:
        score = blob.pixels()

        if is_in_beak(blob):
            score += 100000

        if score > best_score:
            best_score = score
            best = blob

    return best

def blob_output(blobs):
    blob = best_blob(blobs)
    angle, pixel_dist = angle_and_pixel_dist(blob.cx(), blob.cy())
    return blob, angle, pixel_dist

def angle_and_pixel_dist(x, y):
    dx = x - CENTER_X
    dy = -(y - CENTER_Y)

    angle = math.degrees(math.atan2(dx, dy))
    pixel_dist = math.sqrt(dx * dx + dy * dy)

    return angle, pixel_dist

def is_in_beak(blob):
    return (
        BEAK_X <= blob.cx() <= BEAK_X + BEAK_W and
        BEAK_Y <= blob.cy() <= BEAK_Y + BEAK_H
    )

def should_use_ball_fallback(last_angle, last_distance):
    return (
        last_angle is not None and
        last_distance is not None and
        abs(last_angle) < FALLBACK_LAST_SEEN_ANGLE and
        last_distance < FALLBACK_LAST_SEEN_DISTANCE_CM
    )

# ==================================================
# COLOR CALIBRATION
# ==================================================
def run_single_calibration(idx):
    name, cmd = ITEMS[idx][0], ITEMS[idx][1]
    print("Calibrating", name)

    extremes = None
    start = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start) < CALIB_TIME_MS:
        img = sensor.snapshot()
        extremes = capture_extremes(img, idx, extremes)

    if not extremes:
        send_calib_response(cmd, False, (0, 0, 0, 0, 0, 0))
        print("Calibration failed", name)
        return

    thresholds[idx] = threshold_from_stats(extremes, idx)
    save_thresholds_csv()
    send_calib_response(cmd, True, thresholds[idx])

    print("Calibration OK", name, thresholds[idx])

def run_ball_yellow_calibration():
    # 球(手持)跟黃色球門通常同時入鏡，一次校正窗口同時抓兩色，
    # 用雙方實際量到的範圍公平分開重疊的 a 通道，兩色一起存、一起回應。
    print("Calibrating ball + yellow")

    ball_extremes = None
    yellow_extremes = None
    start = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start) < CALIB_TIME_MS:
        img = sensor.snapshot()
        ball_extremes = capture_extremes(img, ITEM_BALL, ball_extremes)
        yellow_extremes = capture_extremes(img, ITEM_YELLOW, yellow_extremes)

    ball_th = threshold_from_stats(ball_extremes, ITEM_BALL) if ball_extremes else None
    yellow_th = threshold_from_stats(yellow_extremes, ITEM_YELLOW) if yellow_extremes else None

    ball_th, yellow_th = separate_ball_yellow(ball_th, yellow_th)

    if ball_th is not None:
        thresholds[ITEM_BALL] = ball_th
    if yellow_th is not None:
        thresholds[ITEM_YELLOW] = yellow_th

    if ball_th is not None or yellow_th is not None:
        save_thresholds_csv()

    send_calib_response(ITEMS[ITEM_BALL][1], ball_th is not None, thresholds[ITEM_BALL])
    send_calib_response(ITEMS[ITEM_YELLOW][1], yellow_th is not None, thresholds[ITEM_YELLOW])

    print("Ball:", "OK" if ball_th else "FAILED", thresholds[ITEM_BALL])
    print("Yellow:", "OK" if yellow_th else "FAILED", thresholds[ITEM_YELLOW])

def capture_extremes(img, idx, extremes):
    blobs = img.find_blobs(
        [ITEMS[idx][2]],
        pixels_threshold=15,
        area_threshold=15,
        merge=True,
        margin=10
    )

    if not blobs:
        return extremes

    blob, stats = get_calibration_blob_and_stats(img, blobs, idx)

    if blob is None or stats is None:
        return extremes

    if DEBUG:
        color = ITEMS[idx][3]
        roi = calibration_roi(img, blob, idx)

        # 外框：偵測到的 blob
        img.draw_rectangle(blob.rect(), color=color)

        # 中心點：blob center
        img.draw_cross(blob.cx(), blob.cy(), color=color, size=2)

        # 內框：實際用來取 LAB statistics 的 calibration ROI
        img.draw_rectangle(roi, color=(255, 255, 255))

    return merge_extremes(extremes, stats)

def merge_extremes(extremes, stats):
    l_min, l_max = stats.l_min(), stats.l_max()
    a_min, a_max = stats.a_min(), stats.a_max()
    b_min, b_max = stats.b_min(), stats.b_max()

    if extremes is None:
        return (l_min, l_max, a_min, a_max, b_min, b_max)

    return (
        min(extremes[0], l_min), max(extremes[1], l_max),
        min(extremes[2], a_min), max(extremes[3], a_max),
        min(extremes[4], b_min), max(extremes[5], b_max),
    )

def separate_ball_yellow(ball_th, yellow_th):
    if ball_th is not None and yellow_th is not None:
        return split_ball_yellow_overlap(ball_th, yellow_th)

    if ball_th is not None:
        ball_th = clip_ball_against_yellow(ball_th, thresholds[ITEM_YELLOW])

    if yellow_th is not None:
        yellow_th = clip_yellow_against_ball(yellow_th, thresholds[ITEM_BALL])

    return ball_th, yellow_th

def split_ball_yellow_overlap(ball_th, yellow_th):
    ball_a_min = ball_th[2]
    yellow_a_max = yellow_th[3]

    overlap = (yellow_a_max + COLOR_SEPARATION_MARGIN) - ball_a_min

    if overlap > 0:
        shift_ball = overlap // 2
        shift_yellow = overlap - shift_ball

        ball_a_min = min(ball_a_min + shift_ball, ball_th[3])
        yellow_a_max = max(yellow_a_max - shift_yellow, yellow_th[2])

    ball_th = (ball_th[0], ball_th[1], ball_a_min, ball_th[3], ball_th[4], ball_th[5])
    yellow_th = (yellow_th[0], yellow_th[1], yellow_th[2], yellow_a_max, yellow_th[4], yellow_th[5])

    return ball_th, yellow_th

def clip_ball_against_yellow(ball_th, yellow_th):
    a_min = max(ball_th[2], yellow_th[3] + COLOR_SEPARATION_MARGIN)
    a_min = min(a_min, ball_th[3])
    return (ball_th[0], ball_th[1], a_min, ball_th[3], ball_th[4], ball_th[5])

def clip_yellow_against_ball(yellow_th, ball_th):
    a_max = min(yellow_th[3], ball_th[2] - COLOR_SEPARATION_MARGIN)
    a_max = max(a_max, yellow_th[2])
    return (yellow_th[0], yellow_th[1], yellow_th[2], a_max, yellow_th[4], yellow_th[5])

def get_calibration_blob_and_stats(img, blobs, idx):
    if idx == ITEM_BALL:
        return get_ball_calibration_blob_and_stats(img, blobs)

    if idx == ITEM_YELLOW:
        return get_yellow_calibration_blob_and_stats(img, blobs)

    return get_blue_calibration_blob_and_stats(img, blobs)

def get_blue_calibration_blob_and_stats(img, blobs):
    # 跟即時偵測用同一套 blue_candidate_ok 篩選，避免校正時選到太暗的陰影/雜訊 blob
    fallback_blob = max(blobs, key=lambda b: b.pixels())
    best = None

    for candidate in blobs:
        if not blue_candidate_ok(img, candidate):
            continue

        if best is None or candidate.pixels() > best.pixels():
            best = candidate

    if best is None:
        best = fallback_blob

    stats = img.get_statistics(roi=calibration_roi(img, best, ITEM_BLUE))
    return best, stats

def get_ball_calibration_blob_and_stats(img, blobs):
    best = None
    best_stats = None

    for candidate in blobs:
        stats = ball_candidate_stats(img, candidate)

        if not stats:
            continue

        if best is None or candidate.pixels() > best.pixels():
            best = candidate
            best_stats = stats

    if best is not None:
        return best, best_stats

    # 沒有候選通過嚴格門檻 (BALL_MIN_A_MEAN/BALL_MIN_B_MEAN) 時，
    # 退而求其次採用 seed threshold 內最大的 blob，避免整段時間都校正不到。
    fallback_blob = max(blobs, key=lambda b: b.pixels())
    fallback_stats = img.get_statistics(roi=calibration_roi(img, fallback_blob, ITEM_BALL))
    return fallback_blob, fallback_stats

def get_yellow_calibration_blob_and_stats(img, blobs):
    orange_rects = orange_avoid_rects(img)

    fallback_blob = max(blobs, key=lambda b: b.pixels())
    best = None

    for candidate in blobs:
        if is_near_orange(candidate, orange_rects):
            continue

        if best is None or candidate.pixels() > best.pixels():
            best = candidate

    if best is None:
        best = fallback_blob

    stats = img.get_statistics(roi=calibration_roi(img, best, ITEM_YELLOW))
    return best, stats

def calibration_roi(img, blob, idx):
    if idx == ITEM_BALL:
        w = max(3, (blob.w() * 2) // 3)
        h = max(3, (blob.h() * 2) // 3)
        x = max(0, blob.cx() - w // 2)
        y = max(0, blob.cy() - h // 2)
    else:
        w = max(4, blob.w() // 8)
        h = max(4, blob.h() // 8)
        x = max(0, blob.cx() - w // 2)
        y = max(0, blob.cy() - h )

    if x + w > img.width():
        w = img.width() - x
    if y + h > img.height():
        h = img.height() - y

    return (x, y, w, h)

def threshold_from_stats(extremes, idx):
    l_margin, a_margin, b_margin = CALIB_MARGINS[idx]

    l_min = clamp(extremes[0] - l_margin, 0, 100)
    l_max = clamp(extremes[1] + l_margin, 0, 100)

    a_min = clamp(extremes[2] - a_margin, -128, 127)
    a_max = clamp(extremes[3] + a_margin, -128, 127)

    b_min = clamp(extremes[4] - b_margin, -128, 127)
    b_max = clamp(extremes[5] + b_margin, -128, 127)

    if idx == ITEM_BALL:
        a_min = max(BALL_MIN_A, a_min)
        b_min = max(BALL_MIN_B, b_min)

    elif idx == ITEM_YELLOW:
        a_min = max(YELLOW_MIN_A, a_min)
        b_min = max(YELLOW_MIN_B, b_min)

    elif idx == ITEM_BLUE:
        l_min = max(BLUE_MIN_L, l_min)
        b_max = min(-1, b_max)

    return (l_min, l_max, a_min, a_max, b_min, b_max)

def ball_candidate_stats(img, blob):
    stats = img.get_statistics(roi=calibration_roi(img, blob, ITEM_BALL))

    if stats.a_mean() < BALL_MIN_A_MEAN:
        return None

    if stats.b_mean() < BALL_MIN_B_MEAN:
        return None

    return stats

def orange_avoid_rects(img):
    rects = []

    orange_blobs = img.find_blobs(
        [ITEMS[ITEM_BALL][2]],
        pixels_threshold=2,
        area_threshold=2,
        merge=True,
        margin=6
    )

    for blob in orange_blobs:
        if ball_candidate_stats(img, blob):
            rects.append(expanded_rect(blob, YELLOW_ORANGE_AVOID_MARGIN, img))

    return rects

def blue_candidate_ok(img, blob):
    stats = img.get_statistics(roi=calibration_roi(img, blob, ITEM_BLUE))
    return stats.l_mean() >= BLUE_MIN_L_MEAN

# ==================================================
# SENSOR CALIBRATION
# ==================================================
def run_sensor_calibration():
    global sensor_exposure, sensor_rgb_gain, sensor_gain_db

    print("Calibrating sensor exposure/WB/gain")

    try:
        sensor.set_auto_exposure(True)
        sensor.set_auto_whitebal(True)
        sensor.set_auto_gain(True)

        start = time.ticks_ms()

        while time.ticks_diff(time.ticks_ms(), start) < SENSOR_CALIB_TIME_MS:
            sensor.snapshot()

        auto_exposure = float(sensor.get_exposure_us())
        auto_rgb = sensor.get_rgb_gain_db()
        auto_gain = float(sensor.get_gain_db())

        sensor_exposure = int(clamp(
            auto_exposure * EXPOSURE_FACTOR,
            MIN_EXPOSURE,
            MAX_EXPOSURE
        ))

        sensor_rgb_gain = (
            auto_rgb[0] * RGB_GAIN_FACTOR[0],
            auto_rgb[1] * RGB_GAIN_FACTOR[1],
            auto_rgb[2] * RGB_GAIN_FACTOR[2],
        )

        sensor_gain_db = clamp(
            auto_gain * GAIN_DB_FACTOR,
            MIN_GAIN_DB,
            MAX_GAIN_DB
        )

        apply_sensor_settings()
        save_sensor_settings_csv()
        send_sensor_calib_response(True)

        print("Auto sensor:", auto_exposure, auto_rgb, auto_gain)
        print("Mapped sensor:", sensor_exposure, sensor_rgb_gain, sensor_gain_db)

    except Exception as e:
        apply_sensor_settings()
        send_sensor_calib_response(False)
        print("Sensor calibration failed:", e)

def apply_sensor_settings(settle=True):
    sensor.set_auto_exposure(False, exposure_us=int(sensor_exposure))
    sensor.set_auto_whitebal(False, rgb_gain_db=sensor_rgb_gain)
    sensor.set_auto_gain(False, gain_db=sensor_gain_db)

    if settle:
        sensor.skip_frames(time=1000)

# ==================================================
# CSV LOAD / SAVE
# ==================================================
def save_thresholds_csv():
    try:
        with open(CALIB_FILE, "w") as f:
            f.write("name,l_min,l_max,a_min,a_max,b_min,b_max\n")

            for i, item in enumerate(ITEMS):
                f.write(threshold_to_csv(item[0], thresholds[i]))

        print("Saved thresholds to", CALIB_FILE)

    except Exception as e:
        print("Threshold save failed:", e)

def load_thresholds_csv():
    loaded = 0

    try:
        with open(CALIB_FILE, "r") as f:
            for line in f:
                line = line.strip()

                if not line or line.startswith("name,"):
                    continue

                parts = line.split(",")

                if len(parts) != 7:
                    continue

                idx = item_index_by_name(parts[0].strip())

                if idx < 0:
                    continue

                thresholds[idx] = tuple(int(v.strip()) for v in parts[1:7])
                loaded += 1

        sanitize_thresholds()
        print("Loaded", loaded, "threshold row(s) from", CALIB_FILE)

    except Exception as e:
        print("No threshold CSV found. Using loose startup thresholds.", e)

def threshold_to_csv(name, th):
    return "%s,%d,%d,%d,%d,%d,%d\n" % (
        name,
        th[0], th[1],
        th[2], th[3],
        th[4], th[5]
    )

def save_sensor_settings_csv():
    try:
        with open(SENSOR_FILE, "w") as f:
            f.write("exposure,r_gain,g_gain,b_gain,gain_db\n")
            f.write(
                "%d,%0.4f,%0.4f,%0.4f,%0.4f\n" %
                (
                    sensor_exposure,
                    sensor_rgb_gain[0],
                    sensor_rgb_gain[1],
                    sensor_rgb_gain[2],
                    sensor_gain_db
                )
            )

        print("Saved sensor settings to", SENSOR_FILE)

    except Exception as e:
        print("Sensor settings save failed:", e)

def load_sensor_settings_csv():
    global sensor_exposure, sensor_rgb_gain, sensor_gain_db

    try:
        with open(SENSOR_FILE, "r") as f:
            for line in f:
                line = line.strip()

                if not line or line.startswith("exposure,"):
                    continue

                parts = line.split(",")

                if len(parts) != 5:
                    continue

                sensor_exposure = int(float(parts[0]))

                sensor_rgb_gain = (
                    clamp(float(parts[1]), MIN_RGB_GAIN, MAX_RGB_GAIN),
                    clamp(float(parts[2]), MIN_RGB_GAIN, MAX_RGB_GAIN),
                    clamp(float(parts[3]), MIN_RGB_GAIN, MAX_RGB_GAIN),
                )

                sensor_gain_db = clamp(
                    float(parts[4]),
                    MIN_GAIN_DB,
                    MAX_GAIN_DB
                )

                print("Loaded sensor settings from", SENSOR_FILE)
                return

    except Exception as e:
        print("No sensor settings CSV found. Using ideal defaults.", e)

def sanitize_thresholds():
    ball = thresholds[ITEM_BALL]
    thresholds[ITEM_BALL] = (
        ball[0],
        ball[1],
        max(BALL_MIN_A, ball[2]),
        ball[3],
        max(BALL_MIN_B, ball[4]),
        ball[5],
    )

    yellow = thresholds[ITEM_YELLOW]
    thresholds[ITEM_YELLOW] = (
        yellow[0],
        yellow[1],
        max(YELLOW_MIN_A, yellow[2]),
        yellow[3],
        max(YELLOW_MIN_B, yellow[4]),
        yellow[5],
    )

    blue = thresholds[ITEM_BLUE]
    thresholds[ITEM_BLUE] = (
        max(BLUE_MIN_L, blue[0]),
        blue[1],
        blue[2],
        blue[3],
        blue[4],
        min(-1, blue[5]),
    )

# ==================================================
# UART
# ==================================================
def check_uart_commands():
    while uart.any() >= 2:
        if uart.read(1) != b'\xbb':
            continue

        raw_cmd = uart.read(1)

        if not raw_cmd:
            return

        cmd = raw_cmd[0]

        if cmd == ITEMS[ITEM_BLUE][1]:
            run_single_calibration(ITEM_BLUE)

        elif cmd == CMD_SENSOR:
            run_sensor_calibration()

        elif cmd == CMD_BALL_YELLOW:
            run_ball_yellow_calibration()

def send_objects_packet(ball, yellow, blue, ball_in=False):
    b_flags = 2 if ball_in else 0

    bf, bs, bm, bd = object_fields(ball[0], ball[1], ball[2], b_flags)
    yf, ys, ym, yd = object_fields(yellow[0], yellow[1], yellow[2])
    blf, bls, blm, bld = object_fields(blue[0], blue[1], blue[2])

    pkt = ustruct.pack(
        "<BBBBBBBBBBBBBB",
        0xA5,
        bf, bs, bm, bd,
        yf, ys, ym, yd,
        blf, bls, blm, bld,
        0x55
    )

    uart.write(pkt)

def object_fields(valid, angle, distance, flags=0):
    sign, mag = signed_mag(angle)

    return (
        (1 if valid else 0) | flags,
        sign,
        clamp(mag, 0, 255),
        clamp(int(distance), 0, 255)
    )

def send_calib_response(cmd, ok, th):
    pkt = ustruct.pack(
        "<BBBBBBBBBB",
        0xCC,
        cmd,
        1 if ok else 0,
        clamp(th[0], 0, 255),
        clamp(th[1], 0, 255),
        clamp(th[2] + 128, 0, 255),
        clamp(th[3] + 128, 0, 255),
        clamp(th[4] + 128, 0, 255),
        clamp(th[5] + 128, 0, 255),
        0x55
    )

    uart.write(pkt)

def send_sensor_calib_response(ok):
    exp = int(sensor_exposure)

    pkt = ustruct.pack(
        "<BBBBBBBBBB",
        0xCC,
        CMD_SENSOR,
        1 if ok else 0,
        (exp >> 8) & 0xFF,
        exp & 0xFF,
        clamp(int(sensor_rgb_gain[0]), 0, 255),
        clamp(int(sensor_rgb_gain[1]), 0, 255),
        clamp(int(sensor_rgb_gain[2]), 0, 255),
        clamp(int(sensor_gain_db), 0, 255),
        0x55
    )

    uart.write(pkt)

# ==================================================
# GEOMETRY / RECT
# ==================================================
def expanded_rect(blob, margin, img):
    x = max(0, blob.x() - margin)
    y = max(0, blob.y() - margin)

    right = min(img.width(), blob.x() + blob.w() + margin)
    bottom = min(img.height(), blob.y() + blob.h() + margin)

    return (x, y, right - x, bottom - y)

def rects_overlap(a, b):
    return not (
        a[0] + a[2] <= b[0] or
        b[0] + b[2] <= a[0] or
        a[1] + a[3] <= b[1] or
        b[1] + b[3] <= a[1]
    )

def is_near_orange(blob, orange_rects):
    rect = blob.rect()

    for orange_rect in orange_rects:
        if rects_overlap(rect, orange_rect):
            return True

    return False

# ==================================================
# DEBUG DRAW
# ==================================================
def draw_debug_reference(img):
    img.draw_rectangle(BEAK_RECT, color=(255, 0, 0))
    img.draw_cross(FRONT_X, FRONT_Y, color=(255, 0, 0), size=2)
    img.draw_cross(CENTER_X, CENTER_Y, color=(255, 255, 255), size=2)

def draw_blob(img, blob, idx):
    color = ITEMS[idx][3]
    img.draw_rectangle(blob.rect(), color=color)
    img.draw_cross(blob.cx(), blob.cy(), color=color, size=2)

# ==================================================
# SMALL HELPERS
# ==================================================
def clamp(v, lo, hi):
    return min(max(v, lo), hi)

def signed_mag(angle):
    rel = int(angle)

    if rel > 180:
        rel -= 360

    return (1 if rel >= 0 else 0), abs(rel)

def item_index_by_name(name):
    for i, item in enumerate(ITEMS):
        if item[0] == name:
            return i

    return -1

# ==================================================
# START
# ==================================================
main()
