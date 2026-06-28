import sensor, time, math, ustruct
from pyb import UART

# ==================================================
# CONFIG
# ==================================================
DEBUG = 1
SEND_UART = True

# ==================================================
# CAMERA GEOMETRY
# ==================================================
CENTER_X = 124
CENTER_Y = 116
FRONT_X = 127
FRONT_Y = 121
ROI = [60, 0, 250, 240]

BEAK_W = 10
BEAK_H = 10
BEAK_X = FRONT_X - (BEAK_W // 2)
BEAK_Y = FRONT_Y - (BEAK_H // 2)
BEAK_RECT = (BEAK_X, BEAK_Y, BEAK_W, BEAK_H)

# Mirror reflection to ground cm.
DIST_A = 2.8
DIST_B = 0.04
BALL_IN_DISTANCE_CM = 15

# ==================================================
# COLOR CALIBRATION
# ==================================================
CALIB_FILE = "color_thresholds.csv"
SENSOR_FILE = "sensor_settings.csv"
CALIB_TIME_MS = 1000
SENSOR_CALIB_TIME_MS = 1200

ITEM_BALL = 0
ITEM_YELLOW = 1
ITEM_BLUE = 2
CMD_SENSOR = 0x14

ITEMS = (
    ("ball", 0x11, (15, 95, 22, 127, 12, 127), (255, 128, 0)),
    ("yellow", 0x12, (30, 60, -5, 25, 38, 65), (255, 255, 0)),
    ("blue", 0x13, (0, 60, -20, 25, -80, -5), (0, 0, 255)),
)

CALIB_MARGINS = (
    (6, 8, 8),  # ball: enough tolerance without swallowing yellow/background
    (4, 6, 6),
    (6, 8, 8),
)

BALL_MIN_A = 25
BALL_MIN_B = 12
BALL_MIN_A_MEAN = 30
BALL_MIN_B_MEAN = 25
YELLOW_MIN_A = -5
YELLOW_MIN_B = 34
YELLOW_ORANGE_AVOID_MARGIN = 10
BLUE_MIN_L = 12
BLUE_MIN_L_MEAN = 16

# Tracking thresholds are loaded from CSV. Until calibrated, use loose thresholds
# so the camera can still boot and accept calibration commands.
thresholds = [item[2] for item in ITEMS]

DEFAULT_EXPOSURE = int(21450 * 0.6)
DEFAULT_RGB_GAIN = (65.0, 61.0, 65.0)
DEFAULT_GAIN_DB = 21.0

# Auto sensor calibration scale factors. These are tuned from the preferred
# correction: exposure lower, RGB gains slightly stronger, sensor gain lower.
EXPOSURE_FACTOR = 1.0
RGB_GAIN_FACTOR = (1.0317, 1.0132, 1.0228)
GAIN_DB_FACTOR = 0.8821
MIN_EXPOSURE = 3000
MAX_EXPOSURE = 25000
MIN_RGB_GAIN = 40.0
MAX_RGB_GAIN = 80.0
MIN_GAIN_DB = 8.0
MAX_GAIN_DB = 28.0

sensor_exposure = DEFAULT_EXPOSURE
sensor_rgb_gain = DEFAULT_RGB_GAIN
sensor_gain_db = DEFAULT_GAIN_DB

# Temporary close-ball fallback.
FALLBACK_LAST_SEEN_ANGLE = 20
FALLBACK_LAST_SEEN_DISTANCE_CM = 20
FALLBACK_BALL_ANGLE = 0
FALLBACK_BALL_DISTANCE = 8

# ==================================================
# UART
# ==================================================
uart = UART(3, 115200, timeout_char=1000)

# ==================================================
# HELPERS
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

def item_index_by_cmd(cmd):
    for i, item in enumerate(ITEMS):
        if item[1] == cmd:
            return i
    return -1

def is_in_beak(blob):
    return (BEAK_X <= blob.cx() <= BEAK_X + BEAK_W and
            BEAK_Y <= blob.cy() <= BEAK_Y + BEAK_H)

def angle_and_pixel_dist(x, y):
    dx = x - CENTER_X
    dy = -(y - CENTER_Y)
    angle = math.degrees(math.atan2(dx, dy))
    return angle, math.sqrt(dx * dx + dy * dy)

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

def threshold_to_csv(name, th):
    return "%s,%d,%d,%d,%d,%d,%d\n" % (name, th[0], th[1], th[2], th[3], th[4], th[5])

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

def save_sensor_settings_csv():
    try:
        with open(SENSOR_FILE, "w") as f:
            f.write("exposure,r_gain,g_gain,b_gain,gain_db\n")
            f.write("%d,%0.4f,%0.4f,%0.4f,%0.4f\n" %
                    (sensor_exposure, sensor_rgb_gain[0], sensor_rgb_gain[1],
                     sensor_rgb_gain[2], sensor_gain_db))
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
                sensor_gain_db = clamp(float(parts[4]), MIN_GAIN_DB, MAX_GAIN_DB)
                print("Loaded sensor settings from", SENSOR_FILE)
                return
    except Exception as e:
        print("No sensor settings CSV found. Using ideal defaults.", e)

def apply_sensor_settings(settle=True):
    sensor.set_auto_exposure(False, exposure_us=int(sensor_exposure))
    sensor.set_auto_whitebal(False, rgb_gain_db=sensor_rgb_gain)
    sensor.set_auto_gain(False, gain_db=sensor_gain_db)
    if settle:
        sensor.skip_frames(time=1000)

def calibration_roi(img, blob, idx):
    if idx == ITEM_BALL:
        w = max(4, (blob.w() * 2) // 3)
        h = max(4, (blob.h() * 2) // 3)
        x = max(0, blob.cx() - w // 2)
        y = max(0, blob.cy() - h // 2)
    else:
        w = max(4, blob.w() // 2)
        h = max(4, blob.h() // 2)
        x = max(0, blob.cx() - w // 2)
        y = max(0, blob.cy() - h // 2)

    if x + w > img.width():
        w = img.width() - x
    if y + h > img.height():
        h = img.height() - y

    return (x, y, w, h)

def expanded_rect(blob, margin, img):
    x = max(0, blob.x() - margin)
    y = max(0, blob.y() - margin)
    right = min(img.width(), blob.x() + blob.w() + margin)
    bottom = min(img.height(), blob.y() + blob.h() + margin)
    return (x, y, right - x, bottom - y)

def rects_overlap(a, b):
    return not (a[0] + a[2] <= b[0] or
                b[0] + b[2] <= a[0] or
                a[1] + a[3] <= b[1] or
                b[1] + b[3] <= a[1])

def is_near_orange(blob, orange_rects):
    rect = blob.rect()
    for orange_rect in orange_rects:
        if rects_overlap(rect, orange_rect):
            return True
    return False

def threshold_from_stats(stats, idx):
    l_margin, a_margin, b_margin = CALIB_MARGINS[idx]

    l_min = clamp(stats.l_min() - l_margin, 0, 100)
    l_max = clamp(stats.l_max() + l_margin, 0, 100)
    a_min = clamp(stats.a_min() - a_margin, -128, 127)
    a_max = clamp(stats.a_max() + a_margin, -128, 127)
    b_min = clamp(stats.b_min() - b_margin, -128, 127)
    b_max = clamp(stats.b_max() + b_margin, -128, 127)

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
    orange_rects = []
    orange_blobs = img.find_blobs([ITEMS[ITEM_BALL][2]], pixels_threshold=8,
                                  area_threshold=8, merge=True, margin=6)
    for blob in orange_blobs:
        if ball_candidate_stats(img, blob):
            orange_rects.append(expanded_rect(blob, YELLOW_ORANGE_AVOID_MARGIN, img))
    return orange_rects

def blue_candidate_ok(img, blob):
    stats = img.get_statistics(roi=calibration_roi(img, blob, ITEM_BLUE))
    return stats.l_mean() >= BLUE_MIN_L_MEAN

def send_calib_response(cmd, ok, th):
    pkt = ustruct.pack("<BBBBBBBBBB",
        0xCC,
        cmd,
        1 if ok else 0,
        clamp(th[0], 0, 255),
        clamp(th[1], 0, 255),
        clamp(th[2] + 128, 0, 255),
        clamp(th[3] + 128, 0, 255),
        clamp(th[4] + 128, 0, 255),
        clamp(th[5] + 128, 0, 255),
        0x55)
    uart.write(pkt)

def run_color_calibration(idx):
    name, cmd, loose_threshold, color = ITEMS[idx]
    print("Calibrating", name)

    best_stats = None
    best_pixels = 0
    start = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start) < CALIB_TIME_MS:
        img = sensor.snapshot()
        blobs = img.find_blobs([loose_threshold], pixels_threshold=15, area_threshold=15,
                               merge=True, margin=10)
        if not blobs:
            continue

        if idx == ITEM_BALL:
            blob = None
            stats = None
            for candidate in blobs:
                candidate_stats = ball_candidate_stats(img, candidate)
                if not candidate_stats:
                    continue
                if blob is None or candidate.pixels() > blob.pixels():
                    blob = candidate
                    stats = candidate_stats
            if blob is None:
                continue
        elif idx == ITEM_YELLOW:
            orange_rects = orange_avoid_rects(img)
            fallback_blob = max(blobs, key=lambda d: d.pixels())
            blob = None
            for candidate in blobs:
                if is_near_orange(candidate, orange_rects):
                    continue
                if blob is None or candidate.pixels() > blob.pixels():
                    blob = candidate
            if blob is None:
                blob = fallback_blob
            stats = img.get_statistics(roi=calibration_roi(img, blob, idx))
        else:
            blob = max(blobs, key=lambda d: d.pixels())
            stats = img.get_statistics(roi=calibration_roi(img, blob, idx))

        if blob.pixels() >= best_pixels:
            best_pixels = blob.pixels()
            best_stats = stats

        if DEBUG:
            img.draw_rectangle(blob.rect(), color=color)
            img.draw_cross(blob.cx(), blob.cy(), color=color, size=2)

    if not best_stats:
        send_calib_response(cmd, False, (0, 0, 0, 0, 0, 0))
        print("Calibration failed", name)
        return

    thresholds[idx] = threshold_from_stats(best_stats, idx)
    save_thresholds_csv()
    send_calib_response(cmd, True, thresholds[idx])
    print("Calibration OK", name, thresholds[idx])

def send_sensor_calib_response(ok):
    exp = int(sensor_exposure)
    pkt = ustruct.pack("<BBBBBBBBBB",
        0xCC,
        CMD_SENSOR,
        1 if ok else 0,
        (exp >> 8) & 0xFF,
        exp & 0xFF,
        clamp(int(sensor_rgb_gain[0]), 0, 255),
        clamp(int(sensor_rgb_gain[1]), 0, 255),
        clamp(int(sensor_rgb_gain[2]), 0, 255),
        clamp(int(sensor_gain_db), 0, 255),
        0x55)
    uart.write(pkt)

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

        sensor_exposure = int(clamp(auto_exposure * EXPOSURE_FACTOR, MIN_EXPOSURE, MAX_EXPOSURE))
        sensor_rgb_gain = (
            clamp(auto_rgb[0] * RGB_GAIN_FACTOR[0], MIN_RGB_GAIN, MAX_RGB_GAIN),
            clamp(auto_rgb[1] * RGB_GAIN_FACTOR[1], MIN_RGB_GAIN, MAX_RGB_GAIN),
            clamp(auto_rgb[2] * RGB_GAIN_FACTOR[2], MIN_RGB_GAIN, MAX_RGB_GAIN),
        )
        sensor_gain_db = clamp(auto_gain * GAIN_DB_FACTOR, MIN_GAIN_DB, MAX_GAIN_DB)

        apply_sensor_settings()
        save_sensor_settings_csv()
        send_sensor_calib_response(True)
        print("Auto sensor:", auto_exposure, auto_rgb, auto_gain)
        print("Mapped sensor:", sensor_exposure, sensor_rgb_gain, sensor_gain_db)
    except Exception as e:
        apply_sensor_settings()
        send_sensor_calib_response(False)
        print("Sensor calibration failed:", e)

def check_uart_commands():
    while uart.any() >= 2:
        if uart.read(1) != b'\xbb':
            continue

        raw_cmd = uart.read(1)
        if not raw_cmd:
            return

        idx = item_index_by_cmd(raw_cmd[0])
        if idx >= 0:
            run_color_calibration(idx)
        elif raw_cmd[0] == CMD_SENSOR:
            run_sensor_calibration()

def object_fields(valid, angle, distance, flags=0):
    sign, mag = signed_mag(angle)
    return (
        (1 if valid else 0) | flags,
        sign,
        clamp(mag, 0, 255),
        clamp(int(distance), 0, 255),
    )

def send_objects_packet(ball, yellow, blue, ball_in=False):
    b_flags = 2 if ball_in else 0
    bf, bs, bm, bd = object_fields(ball[0], ball[1], ball[2], b_flags)
    yf, ys, ym, yd = object_fields(yellow[0], yellow[1], yellow[2])
    blf, bls, blm, bld = object_fields(blue[0], blue[1], blue[2])

    pkt = ustruct.pack("<BBBBBBBBBBBBBB",
        0xA5,
        bf, bs, bm, bd,
        yf, ys, ym, yd,
        blf, bls, blm, bld,
        0x55)
    uart.write(pkt)

def find_object(img, idx, pixels_threshold, area_threshold, margin):
    blobs = img.find_blobs([thresholds[idx]], pixels_threshold=pixels_threshold,
                           area_threshold=area_threshold, merge=True, margin=margin)
    if not blobs:
        return False, 0, 255, None

    if idx == ITEM_BLUE:
        blobs = [blob for blob in blobs if blue_candidate_ok(img, blob)]
        if not blobs:
            return False, 0, 255, None

    blob, angle, pixel_dist = blob_output(blobs)
    # distance = DIST_A * math.exp(DIST_B * pixel_dist)
    distance = 0.03684 * (pixel_dist**2) - 4.9836 * pixel_dist + 180.65
    return True, angle, distance, blob

# ==================================================
# SENSOR SETUP
# ==================================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing(ROI)
sensor.set_hmirror(True)
clock = time.clock()
load_sensor_settings_csv()
apply_sensor_settings()
load_thresholds_csv()

# ==================================================
# MAIN LOOP
# ==================================================
last_ball_angle = None
last_ball_distance = None

while True:
    if DEBUG:
        clock.tick()
    check_uart_commands()
    img = sensor.snapshot()

    if DEBUG:
        img.draw_rectangle(BEAK_RECT, color=(255, 0, 0))
        img.draw_cross(FRONT_X, FRONT_Y, color=(255, 0, 0), size=2)
        img.draw_cross(CENTER_X, CENTER_Y, color=(255, 255, 255), size=2)

    bv, ba, bd, ball_blob = find_object(img, ITEM_BALL, 2, 1, 10)
    yv, ya, yd, yellow_blob = find_object(img, ITEM_YELLOW, 60, 60, 20)
    blv, bla, bld, blue_blob = find_object(img, ITEM_BLUE, 60, 60, 20)
    ball_in = False

    if bv:
        ball_in = bd < BALL_IN_DISTANCE_CM
        last_ball_angle = ba
        last_ball_distance = bd
        if DEBUG and ball_blob:
            img.draw_rectangle(ball_blob.rect(), color=ITEMS[ITEM_BALL][3])
            img.draw_cross(ball_blob.cx(), ball_blob.cy(), color=ITEMS[ITEM_BALL][3], size=2)
    elif (last_ball_angle is not None and
          abs(last_ball_angle) < FALLBACK_LAST_SEEN_ANGLE and
          last_ball_distance < FALLBACK_LAST_SEEN_DISTANCE_CM):
        bv = True
        ba = FALLBACK_BALL_ANGLE
        bd = FALLBACK_BALL_DISTANCE
        ball_in = True

    if DEBUG:
        for idx, blob in ((ITEM_YELLOW, yellow_blob), (ITEM_BLUE, blue_blob)):
            if blob:
                img.draw_rectangle(blob.rect(), color=ITEMS[idx][3])
                img.draw_cross(blob.cx(), blob.cy(), color=ITEMS[idx][3], size=2)

    if SEND_UART:
        if DEBUG:
            print(int(ba), int(bd), int(ya), int(yd), int(bla), int(bld), 1 if ball_in else 0)
            # print(int(bd))
            # print(clock.fps())
        send_objects_packet((bv, ba, bd), (yv, ya, yd), (blv, bla, bld), ball_in)

