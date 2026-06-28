#include <Adafruit_BNO055.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <Servo.h>
#include <Wire.h>

// ======================================================
// CONFIG & GLOBALS (Merged)
// ======================================================

// --- Enums & Constants ---
enum Mode
{
    MENU,
    MAINMODE,
    CMPRESET,
    MOTORTEST,
    EEPROMMODE,
    LSCALIB,
    DEBUGMODE,
    CAMTEST,
    CAMCALIB
};
const int menuCount = 8;
const int EEP_COUNT = 18;
const int EEP_UI_COUNT = 18;

// --- Camera Types ---
struct CameraInput
{
    HardwareSerial* port;
    uint8_t rxBuf[16];
    uint8_t rxIdx;

    bool valid;
    int ballAngle;
    int ballDist;
    bool ballIn;
    bool yValid;
    int yAngle;
    int yDist;
    bool yInFront;
    bool bValid;
    int bAngle;
    int bDist;
    bool bInFront;

    unsigned long lastPacketTime;
};
extern CameraInput cam;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

const int modulePin = A15;
const int BTN_UP = A17;
const int BTN_DOWN = A16;
const int BTN_SEL = A15;
const int BTN_BACK = A14;
const int BOTTOM_ANALOG_COUNT = 18;

const int motorPins[4][2] = {{2, 3}, {4, 5}, {6, 9}, {11, 12}};
const int KICK_PIN = 37;
const int ESC_PIN = 36;

const unsigned long KICK_COOLDOWN = 4000;
const unsigned long KICK_PULSE_MS = 50;

const unsigned long MIN_DISPLAY_STENGTH = 100;
const unsigned long MAX_DISPLAY_STENGTH = 320;
const unsigned long CAM_READ_INTERVAL_MS = 10;
const int CAM_READ_BYTE_BUDGET = 28;

// --- Global Hardware Instances ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
Adafruit_BNO055 bno(55, 0x28, &Wire);

Servo esc;
uint8_t serial8_buffer[1024];  // Buffer for Bottom Board data
IntervalTimer kickerTimer;
// --- Global State Variables ---
int mtEscSpeed = 0;
float headingOffset = 0;
bool bnoValid = false;

float camScaleFactor[1] = {1.0f};
float camAngleBias[1] = {0.0f};
const int BALL_ANGLE_CAL_COUNT = 5;
const uint32_t BALL_ANGLE_CAL_MAGIC = 0xBA11A500;
const int BALL_ANGLE_CAL_ADDR = 120;
float ballAngleCalRaw[BALL_ANGLE_CAL_COUNT] = {0, 45, 135, -135, -45};
float ballAngleCalTarget[BALL_ANGLE_CAL_COUNT] = {0, 45, 135, -135, -45};
bool ballAngleCalValid = false;

int camBallAngle = 0;
int camBallDist = 255;
bool camBallValid = false;

int ballAngle = -1;
int ballWorldAngle = -1;
int ballDistance = 255;
bool ballIn = false;
int ballOutDiff = 0;                 // 球與場外方向的角度差
unsigned long ballOutStartTime = 0;  // 球在場外持續時間
float currentCompass = 0.0;

int goalAngle = -1;
int goalWorldAngle = -1;
int lastGoalAngle = 0;
int lastGoalWorldAngle = 0;
int goalDistance = 255;
int lastGoalDistance = 150;
bool goalValid = false;
bool goalInFront = false;

// Multi-Goal State
int gYGOALAngle = -1;
int gYGOALDist = 0;
int gBGOALAngle = -1;
int gBGOALDist = 0;
bool gYGOALInFront = false;
bool gBGOALInFront = false;
uint8_t goalBuf[6];
int goalIdx = 0;

// Dynamic goal assignment (set by params[16]: 0=attack Blue, 1=attack Yellow)
int ownGoalAngle = -1;  // 己方球門 (local angle)
int ownGoalDist = 255;
int ownGoalWorldAngle = -1;
bool ownGoalValid = false;
bool ownGoalInFront = false;
int lastOwnGoalAngle = 0;  // 最後一次有效的己方球門角度 (ownGoalValid 為 false 時使用)
int lastOwnGoalDist = 150;

int playMode;

// EEPROM
int CHASE_SPEED;
float SCALE_LEFT, SCALE_RIGHT;
float SCALE_LEFT_OFFSET, SCALE_RIGHT_OFFSET;
float OFFSET_RIGHT, OFFSET_LEFT;
int DIST_STRAIGHT, ANGLE_STRAIGHT;
int DIST_WEAK, WEAK_OFFSET, ANGLE_MID;
int DRIBBLER_FAST, DRIBBLER_SLOW;
int BORDER_ESCAPE_SPEED;
int BALL_IN_DISTANCE;
unsigned long BALL_IN_HOLD_MS;

int cmp_offset = 0;
float rotate_scale = 0.8;
int backAngle = -1;
int backWorldAngle = -1;
bool liftedRX = false, edgeStanding = false;
bool lineLeft = false, lineRight = false;
bool lineFront = false, lineBack = false;
uint8_t buf[11];
int idx = 0;
uint8_t bottomAnalogBuf[39];
int bottomAnalogIdx = 0;
int bottomAnalogValue[BOTTOM_ANALOG_COUNT];
bool bottomAnalogValid = false;
unsigned long lastBottomAnalogPacketTime = 0;

// -- Defense / Line Sensors --
int lineAngle = 0;
float lineOffset = 0;
bool lineDetected = false;
unsigned long nearStopStartTime = 0;
unsigned long lastBottomPacketTime = 0;
unsigned long lastCamPacketTime = 0;
unsigned long pushStartTime = 0;
unsigned long stuckStartTime = 0;

float params[EEP_COUNT];
int eepIndex = 0;
const int EEPROM_ADDR = 0;
bool eepEditing = false;
const char* eepNames[EEP_COUNT] = {"Chase Speed",  "Scale L",   "Scale R",  "Offset L",  "Offset R",  "Dist Straight",
                                   "Ang Straight", "Dist Weak", "Ang Weak", "Ang Mid",   "Drib Fast", "Drib Slow",
                                   "Border Esc",   "BallIn D",  "BallIn Hold", "Play Mode", "Goal Side", "Rot Scale"};
const int eepUiIndices[EEP_UI_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};

Mode currentMode = MENU;
int menuIndex = 0;

int mtSelected = 0;
int mtSpeed[4] = {0, 0, 0, 0};
bool mtEditing = false;
unsigned long lastKickTime = 0;
bool lineCalReadyForStart = false;

// --- Prototypes ---
void updateSensors();
void parseCameraPort(CameraInput& cam, int byteBudget);
void updateCameraData();
float calibrateBallAngle(float rawAngle);
void saveBallAngleCalibration();
void home();
void chaseBall();
void kickBall();
void stopAllMotors();
void motor(int id, int speedPercent);
void move(int angle, int speed, float cmp_offset = 0.0);
bool moveForMs(int angle, int speed, int cmp_offset, int ms, bool abortOnBallLost = false,
               bool abortOnFrontLine = false, bool abortOnLeftLine = false, bool abortOnRightLine = false);
void rotate(int speed, bool m1 = true, bool m2 = true, bool m3 = true, bool m4 = true);
bool rotateToAngle(float targetAngle, int speed, bool abortOnBallLost = false, bool m1 = true, bool m2 = true,
                   bool m3 = true, bool m4 = true);
void motorTestLoop();
void loadParams();
void resetEEPROMToDefaults();
void eepromModeLoop();
void drawMenu(float cmp);
void handleMenuMode();
void handleMainMode();
void handleOffenseMode();
// handleBallAtEdge() removed — was declared but never defined or called
bool border();
void applyBorderSlowdown();
bool lineBorder();
void cornerCatch();
void handleDefenseMode();
void handleCalibrateMode();
void handleCompassReset();
void handleDebugMode();
void camTestLoop();
void cameraCalibLoop();
void showBallYellowCalibResult();
void runBallAngleCalibration();
bool readBtn(int pin);
void dribbler(int speed);
int angleDiff(int a, int b);
float norm180(float a);
void doTest();

// --- Simple Utility Functions ---
bool readBtn(int pin)
{
    return digitalRead(pin) == LOW;  // 低電位觸發 (active low)
}

float norm180(float a)
{
    a = fmod(a + 540.0f, 360.0f) - 180.0f;
    return a;
}

int angleDiff(int a, int b) { return norm180(a - b); }

// --------------------------------------------------
// 設定
// --------------------------------------------------
void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200);  // Camera Top
    Serial7.begin(115200);  // Camera Front
    Serial8.begin(115200);  // Bottom Board
    Serial8.addMemoryForRead(serial8_buffer, sizeof(serial8_buffer));
    Wire.begin();
    Wire1.begin();

    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(BTN_SEL, INPUT_PULLUP);
    pinMode(BTN_BACK, INPUT_PULLUP);

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
        while (1)
            ;
    display.print("Boot...");
    delay(1000);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    for (int i = 0; i < 4; i++)
    {
        pinMode(motorPins[i][0], OUTPUT);
        pinMode(motorPins[i][1], OUTPUT);
    }
    esc.attach(ESC_PIN, 1000, 2000);
    esc.write(0);
    stopAllMotors();
    pinMode(KICK_PIN, OUTPUT);
    digitalWrite(KICK_PIN, LOW);

    dribbler(0);  // 初始化吸球速度為 0
    delay(300);

    if (!bno.begin(OPERATION_MODE_IMUPLUS))
    {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("BNO FAIL!");
        display.println("SKIP CALIBRATION");
        display.display();
        bnoValid = false;
        delay(1000);
    }
    else
    {
        bnoValid = true;
    }

    if (bnoValid)
    {
        // --- 讓 BNO055 內部 Gyro 校準達到 3 分才抓 0 度 ---
        // 這能大幅減少開機就歪掉或漂移的問題
        uint8_t system = 0, gyro = 0, accel = 0, mag = 0;
        unsigned long startWait = millis();
        while (gyro < 3 && (millis() - startWait < 6000))
        {
            bno.getCalibration(&system, &gyro, &accel, &mag);
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("BNO CALIB...");
            display.setCursor(0, 15);
            display.print("GYRO: ");
            display.print(gyro);
            display.display();
            delay(100);
        }

        delay(300);  // 靜置一下再抓零點

        // 多次採樣取平均，確保 0 度設定精確
        float avgX = 0;
        for (int i = 0; i < 20; i++)
        {
            sensors_event_t ev;
            bno.getEvent(&ev);
            avgX += ev.orientation.x;
            delay(15);
        }
        headingOffset = avgX / 20.0f;
    }
    else
    {
        headingOffset = 0.0f;
    }
    // --- 開機按住 SEL 鍵重設 EEPROM 至合理預設值 ---
    if (readBtn(BTN_SEL))
    {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("FACTORY RESET...");
        display.println("EEPROM DEFAULTS");
        display.display();
        resetEEPROMToDefaults();
        delay(1500);
        display.println("RESET SUCCESS!");
        display.display();
        delay(1000);
    }
    loadParams();
}

// --------------------------------------------------
// 主迴圈
// --------------------------------------------------
void loop()
{
    updateSensors();

    if (currentMode == MENU)
    {
        handleMenuMode();
        return;
    }

    if (readBtn(BTN_BACK))
    {
        if (currentMode == DEBUGMODE)
            Serial8.write(0xE1);  // Stop Bottom Board analog debug mode

        stopAllMotors();
        dribbler(0);  // 停止吸球
        for (int i = 0; i < 4; i++)
            mtSpeed[i] = 0;  // 重置馬達測試速度
        mtEscSpeed = 0;      // 重置 ESC 測試速度
        mtEditing = false;
        kickerTimer.end();            // 停止 any running kick timer
        digitalWrite(KICK_PIN, LOW);  // Ensure kicker is retracted

        // Re-enable OLED I2C bus and display when returning to MENU
        Wire1.begin();
        Wire1.setClock(400000);
        display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

        currentMode = MENU;
        delay(200);
        return;
    }

    switch (currentMode)
    {
        case MENU:
            break;
        case MAINMODE:
            handleMainMode();
            // doTest();
            // delay(1000000);
            break;
        case CMPRESET:
            handleCompassReset();
            break;
        case MOTORTEST:
            motorTestLoop();
            break;
        case EEPROMMODE:
            eepromModeLoop();
            break;
        case LSCALIB:
            handleCalibrateMode();
            break;
        case DEBUGMODE:
            handleDebugMode();
            break;
        case CAMTEST:
            camTestLoop();
            break;
        case CAMCALIB:
            cameraCalibLoop();
            break;
    }
}

void handleMenuMode()
{
    drawMenu(currentCompass);
    if (readBtn(BTN_UP))
    {
        menuIndex = (menuIndex - 1 + menuCount) % menuCount;
        delay(150);
    }
    if (readBtn(BTN_DOWN))
    {
        menuIndex = (menuIndex + 1) % menuCount;
        delay(150);
    }
    if (readBtn(BTN_SEL))
    {
        if (menuIndex == 0)
        {
            display.clearDisplay();
            display.display();
            Wire1.end();  // Disable OLED I2C bus during main gameplay loop
            currentMode = MAINMODE;
        }
        else if (menuIndex == 1)
            currentMode = CMPRESET;
        else if (menuIndex == 2)
            currentMode = EEPROMMODE;
        else if (menuIndex == 3)
        {
            lineCalReadyForStart = false;
            currentMode = LSCALIB;
        }
        else if (menuIndex == 4)
        {
            currentMode = CAMCALIB;
        }
        else if (menuIndex == 5)
            currentMode = MOTORTEST;
        else if (menuIndex == 6)
        {
            currentMode = CAMTEST;
        }
        else if (menuIndex == 7)
        {
            bottomAnalogValid = false;
            bottomAnalogIdx = 0;
            Serial8.write(0xE0);  // Start Bottom Board analog debug mode
            currentMode = DEBUGMODE;
        }
        delay(150);
    }
}

void handleDebugMode()
{
    updateSensors();

    // --- Bottom Board analog read page ---
    display.clearDisplay();
    display.setTextSize(1);

    bool bottomAnalogAlive = bottomAnalogValid && (millis() - lastBottomAnalogPacketTime < 500);
    if (!bottomAnalogAlive)
    {
        display.setCursor(0, 0);
        display.println("BOTTOM ANA DEBUG");
        display.setCursor(0, 18);
        display.println("WAITING PACKET...");
        display.setCursor(0, 36);
        display.println("BACK=EXIT");
        display.display();
        return;
    }

    for (int i = 0; i < BOTTOM_ANALOG_COUNT; i++)
    {
        int col = i / 6;
        int row = i % 6;
        int x = col * 43;
        int y = row * 10;

        display.setCursor(x, y);
        display.print(i);
        display.print(":");
        display.print(bottomAnalogValue[i]);
    }

    display.display();
}

void handleCompassReset()
{
    // 多次採樣取平均，確保 0 度設定精確
    float avgX = 0;
    for (int i = 0; i < 5; i++)
    {
        sensors_event_t ev;
        bno.getEvent(&ev);
        avgX += ev.orientation.x;
        delay(30);
    }
    headingOffset = avgX / 5.0f;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("COMPASS RESET");
    display.setCursor(0, 12);
    display.print("NEW ZERO SET");
    display.display();
    delay(600);
    currentMode = MENU;
}

void handleCalibrateMode()
{
    if (!lineCalReadyForStart)
    {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("LS CAL READY");
        display.setCursor(0, 18);
        display.println("Release SEL");
        display.setCursor(0, 34);
        display.println("Then SEL=START");
        display.setCursor(0, 50);
        display.println("BACK=EXIT");
        display.display();

        if (!readBtn(BTN_SEL))
        {
            lineCalReadyForStart = true;
        }
        return;
    }

    if (!readBtn(BTN_SEL))
    {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("LS CAL READY");
        display.setCursor(0, 20);
        display.println("Press SEL START");
        display.setCursor(0, 40);
        display.println("BACK to cancel");
        display.display();
        return;
    }

    Serial8.write(0xDE);  // Trigger Defense Calibrate
    delay(200);

    while (readBtn(BTN_BACK) == false)
    {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("LS CALIBRATING...");
        display.setCursor(0, 20);
        display.println("MOVE ROBOT ON LINE");
        display.setCursor(0, 40);
        display.println("Press BACK to SAVE");
        display.display();
    }
    Serial8.write(0xDF);  // Stop and save
    lineCalReadyForStart = false;
    currentMode = MENU;
    delay(200);
}

void handleMainMode()
{
    if (playMode == 1)
    {
        handleDefenseMode();
    }
    else
    {
        handleOffenseMode();
    }
}
