#include <EEPROM.h>
// DEBUG is now handled dynamically by checking the Serial object

// --------------------------------------------------
//  NFIG
// --------------------------------------------------
const int NUM_SENSORS = 18;
const int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17};

const int BUTTON_PIN = 2;
const int EEPROM_START = 20;

bool readyToStartCollect = false;
bool collecting = false;
bool analogDebugMode = false;

unsigned long pressStart = 0;
unsigned long lastBlink = 0;
bool ledState = false;

int highestVal[NUM_SENSORS];
int lowestVal[NUM_SENSORS];

int highThresh[NUM_SENSORS];
int lowThresh[NUM_SENSORS];

// --- Back Angle State (Offense) ---
float backAngle = -1;         // angle (0-359)
bool backAngleValid = false;  // true = usable
float lastOnlineAvg = -1;
float BACK_ANGLE_LIMIT = 70.0;  // 降低限制，防止大跳變 (跳超過 25 度會被忽略)
float CONTINUE_RANGE = 60.0;
unsigned long lastLineDetectTime = 0;
const unsigned long LINE_CONTINUE_MS = 50;   // 線消失後保留角度的時長 (毫秒)
const unsigned long LINE_DROP_HOLD_MS = 10;  // 單顆感測器瞬間掉值時，短暫保留觸線狀態
unsigned long lastSensorLineTime[NUM_SENSORS];
// --------------------------------------------------

// --------------------------------------------------
// ANGLE UTILITIES
// --------------------------------------------------

float sensorAngle(int index)
{
    float angle = 360.0 - index * (360.0 / NUM_SENSORS);
    if (angle >= 360)
        angle -= 360;
    if (angle < 0)
        angle += 360;
    return angle;
}

float normalize180(float a)
{
    while (a > 180)
        a -= 360;
    while (a <= -180)
        a += 360;
    return a;
}

float angleDiff(float a, float b) { return normalize180(a - b); }
float circularDistance(float a, float b) { return abs(angleDiff(a, b)); }

float circularDifference(float a, float b)
{
    float diff = fabs(a - b);
    return (diff > 180.0) ? 360.0 - diff : diff;
}

void clearBackAngle()
{
    backAngle = -1;
    backAngleValid = false;
}

void findFarthestPair(float angles[], int count, float& a1, float& a2, float& separation)
{
    separation = 0;
    a1 = 0;
    a2 = 0;

    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            float d = circularDistance(angles[i], angles[j]);
            if (d > separation) {
                separation = d;
                a1 = angles[i];
                a2 = angles[j];
            }
        }
    }
}

float circularMidpoint(float a, float b)
{
    float diff = angleDiff(b, a);
    return normalize180(a + diff / 2.0);
}

float getNormalizedError(float separation)
{
    // separation 是兩個觸線感測器之間的夾角，跟機器人實際偏離線中心的距離 d
    // 是 separation = 2*arccos(d/R) 的關係 (R = 感測器環半徑)，不是線性。
    // 這個 arccos 關係是感測器環的幾何形狀決定的物理定律，沒有程式碼可以改；
    // 能動手的只有「測到的 separation 怎麼換算成 lineOffset」這一步。
    // cos 是 arccos 的反函數，cos(separation/2) 直接解出 d/R，
    // 讓算出來的值跟實際物理偏移距離成正比 (改成直線排列的感測器才會天生線性，
    // 但這顆要 360° 全方向偵測線，做成環狀無法避免這個非線性)。
    if (separation < 0)
        separation = 0;
    if (separation > 180)
        separation = 180;
    return cos(separation * PI / 360.0);  // 0.0 (置中) ~ 1.0 (貼邊緣)
}

// --------------------------------------------------
// 高速 UART 封包 (總共 11 位元組)
//
// [0]  0xAA (起始 1)
// [1]  0xBB (起始 2)
// [2]  狀態位元組
//      - 255 : 被抬起 (Lifted)
//      - Bit 0 (1):  守門員對齊線軸 (Centered)
//      - Bit 1 (2):  未偵測白線 (Error)
//      - Bit 2 (4):  踩在白線邊緣 (Edge Standing)
// [3]  誤差幅度 (0–255)
// [4]  白線角度高位元組
// [5]  白線角度低位元組 (0–360 映射)
// [6]  後方角度高位元組
// [7]  後方角度低位元組 (0–359 or 65535)
// [8]  前/後感測器觸線 (bitmask)
//      Bit 0 (1): Front (±25° 內感測器: 0°, ±20°)
//      Bit 1 (2): Back  (|a|≥15⁵°: ±160°, 180°)
// [9]  特定左/右感測器觸線 (bitmask)
//      Bit 0 (1): Left  (a ∈ -70°…-110°: ±80°, ±100°)
//      Bit 1 (2): Right (a ∈  70°… 110°: +80°, +100°)
// [10] 0x55 (結束)
// --------------------------------------------------
// --------------------------------------------------
// --------------------------------------------------
void sendUnifiedPacket(float lineAngle, float errorVal, uint8_t status, float bAngle, bool bValid, uint8_t frontCount,
                       uint8_t backCount)
{
    // 1. Line Tracking Angle (shifted to 0-360)
    int shiftedAngle = (int)normalize180(lineAngle) + 180;
    uint8_t lineH = (uint8_t)(shiftedAngle >> 8);
    uint8_t lineL = (uint8_t)(shiftedAngle & 0xFF);

    // 2. Error Value
    uint8_t errorByte = (uint8_t)constrain(errorVal * 255.0f, 0, 255);

    // 3. Back Angle (0-359 or 65535 if invalid)
    uint16_t bAngleInt = bValid ? (uint16_t)bAngle : 65535;
    uint8_t backH = (uint8_t)(bAngleInt >> 8);
    uint8_t backL = (uint8_t)(bAngleInt & 0xFF);

    Serial8.write(0xAA);
    Serial8.write(0xBB);
    Serial8.write(status);
    Serial8.write(errorByte);
    Serial8.write(lineH);
    Serial8.write(lineL);
    Serial8.write(backH);
    Serial8.write(backL);
    Serial8.write(frontCount);  // [8] standalone front byte
    Serial8.write(backCount);   // [9] standalone back byte
    Serial8.write(0x55);
}

void sendAnalogDebugPacket()
{
    Serial8.write(0xCC);
    Serial8.write(0xDD);
    for (int i = 0; i < NUM_SENSORS; i++) {
        uint16_t value = analogRead(sensorPins[i]);
        Serial8.write((uint8_t)(value >> 8));
        Serial8.write((uint8_t)(value & 0xFF));
    }
    Serial8.write(0x55);
}

// --------------------------------------------------
// SETUP
// --------------------------------------------------
void setup()
{
    Serial.begin(115200);
    Serial8.begin(115200);

    analogReadResolution(12);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);

    loadThresholdsFromEEPROM();

    Serial.println("Unified Line-Slinging Mode: thresholds loaded.");
}

// --------------------------------------------------
// MAIN LOOP
// --------------------------------------------------
void loop()
{
    handleButton();

    if (Serial8.available()) {
        uint8_t cmd = Serial8.read();
        if (cmd == 0xDE) {  // Remote Start
            for (int i = 0; i < NUM_SENSORS; i++) {
                highestVal[i] = 0;
                lowestVal[i] = 4095;
            }
            collecting = true;
            analogDebugMode = false;
        }
        else if (cmd == 0xDF && collecting) {  // Remote Stop
            finishCollectMode();
            collecting = false;
        }
        else if (cmd == 0xE0) {  // Analog debug start
            analogDebugMode = true;
        }
        else if (cmd == 0xE1) {  // Analog debug stop
            analogDebugMode = false;
        }
    }

    if (collecting) {
        collectDataLoop();
    }
    else if (analogDebugMode) {
        sendAnalogDebugPacket();
    }
    else {
        normalModeLoop();
    }
}

// --------------------------------------------------
// BUTTON HANDLING
// --------------------------------------------------
void handleButton()
{
    static bool lastBtn = HIGH;
    bool btn = digitalRead(BUTTON_PIN);

    if (btn == LOW && lastBtn == HIGH)
        pressStart = millis();

    if (!collecting && !readyToStartCollect) {
        if (btn == LOW && millis() - pressStart > 2000) {
            readyToStartCollect = true;
            digitalWrite(LED_BUILTIN, HIGH);
            Serial.println("Hold detected. Release to start collecting...");
        }
    }

    if (readyToStartCollect && btn == HIGH && lastBtn == LOW) {
        delay(100);

        for (int i = 0; i < NUM_SENSORS; i++) {
            highestVal[i] = 0;
            lowestVal[i] = 4095;
        }

        collecting = true;
        readyToStartCollect = false;

        Serial.println("Starting collect mode...");
        ledState = false;
        lastBlink = millis();
    }

    if (collecting && btn == LOW && lastBtn == HIGH) {
        finishCollectMode();
        collecting = false;
    }

    lastBtn = btn;
}

// --------------------------------------------------
// COLLECT MODE LOOP
// --------------------------------------------------
void collectDataLoop()
{
    if (millis() - lastBlink > 300) {
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
        lastBlink = millis();
    }

    for (int i = 0; i < NUM_SENSORS; i++) {
        int v = analogRead(sensorPins[i]);
        if (v > highestVal[i])
            highestVal[i] = v;
        if (v < lowestVal[i])
            lowestVal[i] = v;
    }
}

// --------------------------------------------------
// FINISH COLLECT MODE
// --------------------------------------------------
void finishCollectMode()
{
    Serial.println("Saving thresholds...");

    digitalWrite(LED_BUILTIN, LOW);

    for (int i = 0; i < NUM_SENSORS; i++) {
        int cur = analogRead(sensorPins[i]);
        int h = (highestVal[i] + cur) / 2;
        // int h = cur * 2;
        int l = (lowestVal[i] + cur) / 2;

        EEPROM.write(EEPROM_START + i * 2, map(h, 0, 4095, 0, 255));
        EEPROM.write(EEPROM_START + i * 2 + 1, map(l, 0, 4095, 0, 255));
    }

    loadThresholdsFromEEPROM();

    for (int i = 0; i < 2; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(150);
        digitalWrite(LED_BUILTIN, LOW);
        delay(150);
    }

    Serial.println("Thresholds saved.");
}

// --------------------------------------------------
// LOAD THRESHOLDS
// --------------------------------------------------
void loadThresholdsFromEEPROM()
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        int h = EEPROM.read(EEPROM_START + i * 2);
        int l = EEPROM.read(EEPROM_START + i * 2 + 1);

        highThresh[i] = map(h, 0, 255, 0, 4095);
        lowThresh[i] = map(l, 0, 255, 0, 4095);
    }
}

// --------------------------------------------------
// NORMAL MODE (UNIFIED OFFENSE + DEFENSE LOGIC)
// --------------------------------------------------
void normalModeLoop()
{
    float detectedAngles[NUM_SENSORS];
    bool activeSensors[NUM_SENSORS];

    int countDetected = 0;
    int liftedCount = 0;
    float sumX = 0, sumY = 0;
    unsigned long now = millis();

    for (int i = 0; i < NUM_SENSORS; i++) {
        int v = analogRead(sensorPins[i]);
        activeSensors[i] = false;

        if (v < lowThresh[i]) {
            liftedCount++;
            continue;
        }

        if (v > highThresh[i]) {
            lastSensorLineTime[i] = now;
        }

        if (v > highThresh[i] || now - lastSensorLineTime[i] <= LINE_DROP_HOLD_MS) {
            float a = sensorAngle(i);

            // Store for defense logic
            detectedAngles[countDetected] = normalize180(a);
            activeSensors[i] = true;

            // Sum for offense logic
            float rad = a * DEG_TO_RAD;
            sumX += cosf(rad);
            sumY += sinf(rad);

            countDetected++;
        }
    }

    // Fill single-sensor gaps to handle hardware noise (Relaxed edge standing)
    bool filledSensors[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) {
        filledSensors[i] = activeSensors[i];
        if (!activeSensors[i]) {
            int prev = (i - 1 + NUM_SENSORS) % NUM_SENSORS;
            int next = (i + 1) % NUM_SENSORS;
            if (activeSensors[prev] && activeSensors[next])
                filledSensors[i] = true;
        }
    }

    // Wrap-aware edge detection: count leading & trailing streaks
    int leadingLen = 0;
    for (int i = 0; i < NUM_SENSORS && filledSensors[i]; i++)
        leadingLen++;

    int trailingLen = 0;
    if (leadingLen < NUM_SENSORS) {
        for (int i = NUM_SENSORS - 1; i >= 0 && filledSensors[i]; i--)
            trailingLen++;
    }

    // Linear scan: count total distinct streaks and find the longest
    int streakCount = 0;
    int longestStreak = 0;
    {
        bool inStreak = false;
        int curLen = 0;
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (filledSensors[i]) {
                if (!inStreak) {
                    inStreak = true;
                    streakCount++;
                    curLen = 0;
                }
                curLen++;
            }
            else {
                if (inStreak) {
                    longestStreak = max(longestStreak, curLen);
                    inStreak = false;
                }
            }
        }
        if (inStreak)
            longestStreak = max(longestStreak, curLen);
    }

    // If leading + trailing both exist they form one wrap-around streak
    bool wraps = (leadingLen > 0 && trailingLen > 0 && leadingLen + trailingLen < NUM_SENSORS);
    if (wraps) {
        streakCount--;  // counted as 2; actually 1
        longestStreak = max(longestStreak, leadingLen + trailingLen);
    }

    bool edgeStanding = (streakCount == 1 && longestStreak >= 2);

    // Fix #5: lifted check — clearBackAngle + still encode edgeStanding in packet
    bool liftedState = (liftedCount >= 8);
    if (liftedState) {
        clearBackAngle();
        sendUnifiedPacket(0, 0, 255, backAngle, backAngleValid, 0, 0);
        if (Serial)
            Serial.println("LIFTED: Sending lift signal...");
        return;
    }

    // --------------------------------------------------
    // OFFENSE LOGIC (BACK ANGLE TRACKING)
    // --------------------------------------------------
    if (countDetected > 0) {
        float avgRad = atan2f(sumY, sumX);
        float onlineAvg = avgRad * RAD_TO_DEG;
        if (onlineAvg < 0)
            onlineAvg += 360.0f;

        lastOnlineAvg = onlineAvg;
        float newBackAngle = onlineAvg + 180.0f;
        if (newBackAngle >= 360.0f)
            newBackAngle -= 360.0f;

        if (!backAngleValid) {
            backAngle = newBackAngle;
            backAngleValid = true;
        }
        else {
            float diff = circularDifference(newBackAngle, backAngle);
            if (diff < BACK_ANGLE_LIMIT) {
                backAngle = newBackAngle;
            }
        }
        lastLineDetectTime = millis();
    }
    else {
        // 沒有偵測到線：判斷是否在寬容時間內
        if (millis() - lastLineDetectTime > LINE_CONTINUE_MS) {
            clearBackAngle();
        }
        // 如果在寬容時間內，保持舊的 backAngle 和 backAngleValid = true
    }

    // --------------------------------------------------
    // DEFENSE LOGIC (CORRECTION ANGLE)
    // --------------------------------------------------
    static float lastNormError = 0;
    static float correctionAngle = 0;
    uint8_t status = 0;

    if (countDetected >= 2) {
        float a1 = 0, a2 = 0, separation = 0;
        findFarthestPair(detectedAngles, countDetected, a1, a2, separation);
        float currentCorrectionAngle = circularMidpoint(a1, a2);

        if (currentCorrectionAngle >= 80 && currentCorrectionAngle <= 100)
            currentCorrectionAngle = 90.0f;
        if (currentCorrectionAngle <= -80 && currentCorrectionAngle >= -100)
            currentCorrectionAngle = -90.0f;

        correctionAngle = currentCorrectionAngle;

        float normError = getNormalizedError(separation);
        lastNormError = normError;

        bool leftSide = activeSensors[4] || activeSensors[5];
        bool rightSide = activeSensors[13] || activeSensors[14];
        bool onLineAxis = leftSide && rightSide;

        if (onLineAxis || normError < 0.05f) {
            status |= 1;  // Bit 0: Centered
        }
    }
    else if (countDetected == 1) {
        // Falls back to the single sensor's angle to avoid jumping to 0
        correctionAngle = detectedAngles[0];
    }
    else {
        lastNormError = 0;
        status |= 2;  // Bit 1: Error
    }

    if (edgeStanding) {
        status |= 4;  // Bit 2: Edge Standing
    }

    // 前/後、特定左/右感測器觸線偵測
    uint8_t fbByte = 0, lrByte = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (activeSensors[i]) {
            float a = normalize180(sensorAngle(i));
            float absA = fabs(a);
            if (activeSensors[0] || activeSensors[1] || activeSensors[17])
                fbByte |= 1;  // Front: 0°, ±20°
            if (activeSensors[7] || activeSensors[8] || activeSensors[9])
                fbByte |= 2;  // Back:  ±160°, 180°
            if (activeSensors[5] || activeSensors[6])
                lrByte |= 1;  // Left:  -100°, -120°
            if (activeSensors[12] || activeSensors[13])
                lrByte |= 2;  // Right: +100°, +120°
        }
    }

    // --------------------------------------------------
    // SEND PACKET
    // --------------------------------------------------
    sendUnifiedPacket(correctionAngle, lastNormError, status, backAngle, backAngleValid, fbByte, lrByte);
}
