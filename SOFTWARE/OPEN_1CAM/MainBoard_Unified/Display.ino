// --------------------------------------------------
// 選單繪製
// --------------------------------------------------
void drawCamMiniMap(int cx, int cy, int r);

void drawMenu(float cmp)
{
    display.clearDisplay();
    display.setTextSize(1);
    const char* shortName[8] = {"RUN", "CMP", "EEP", "CAL", "CCA", "MTR", "CAM", "ANG"};

    const int pageSize = 5;
    int page = menuIndex / pageSize;
    int startIdx = page * pageSize;
    int endIdx = min(startIdx + pageSize, menuCount);

    for (int i = startIdx; i < endIdx; i++)
    {
        display.setCursor(0, 2 + (i - startIdx) * 12);
        if (i == menuIndex)
        {
            display.print(">");
        }
        else
        {
            display.print(" ");
        }
        display.print(shortName[i]);
    }

    display.drawLine(31, 0, 31, 63, SSD1306_WHITE);

    display.setCursor(110, 0);
    display.print(page + 1);
    display.print("/2");

    bool camAlive = (millis() - cam.lastPacketTime < 500);
    bool bottomAlive = (lastBottomPacketTime != 0 && millis() - lastBottomPacketTime < 500);
    bool hasError = !camAlive || !bottomAlive || !bnoValid || liftedRX;

    if (hasError)
    {
        int errorCount = 0;
        if (!camAlive)
            errorCount++;
        if (!bottomAlive)
            errorCount++;
        if (!bnoValid)
            errorCount++;
        if (liftedRX)
            errorCount++;

        display.fillRect(35, 3, 93, 10, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
        display.setCursor(41, 4);
        display.print("!! ERROR !!");
        display.setTextColor(SSD1306_WHITE);

        int y = 18 + max(0, (46 - errorCount * 10) / 2);
        if (!camAlive)
        {
            display.setCursor(35, y);
            display.print("CAM: NO UART");
            y += 10;
        }
        if (!bottomAlive)
        {
            display.setCursor(35, y);
            display.print("BOT: NO UART");
            y += 10;
        }
        if (!bnoValid)
        {
            display.setCursor(35, y);
            display.print("BNO: FAIL");
            y += 10;
        }
        if (liftedRX)
        {
            display.setCursor(35, y);
            display.print("ROBOT LIFTED");
        }

        display.display();
        return;
    }

    display.setCursor(35, 3);
    display.print("CMP:");
    display.print((int)cmp);
    if (camAlive)
    {
        display.setCursor(35, 13);
        display.print("ANG:");
        display.print(ballAngle);

        display.setCursor(35, 23);
        display.print("DST:");
        display.print(ballDistance);
        if (ballIn)
        {
            display.print(" IN");
        }
    }
    else
    {
        display.setCursor(35, 13);
        display.print("CAM_ERR:");
        display.setCursor(35, 23);
        display.print("F");
    }
    display.setCursor(35, 33);
    display.print("BACK:");
    if (liftedRX)
        display.print("LIFT");
    else if (backAngle != -1)
    {
        display.print(backAngle);
        display.print(" ");
        if (edgeStanding)
            display.print("E");
        if (lineFront)
            display.print("F");
        if (lineBack)
            display.print("B");
        if (lineLeft)
            display.print("L");
        if (lineRight)
            display.print("R");
    }
    else
        display.print("X");

    if (playMode == 0)
    {
    display.setCursor(35, 43);
    display.print("G:");
        if (goalValid)
        {
            display.print(goalWorldAngle);
            display.print(",");
            display.print(goalDistance);
        }
        else
        {
            display.print("X");
        }
    }
    else
    {
    display.setCursor(35, 43);
    display.print("ownG:");
        if (ownGoalValid)
        {
            display.print(ownGoalWorldAngle);
            display.print(",");
            display.print(ownGoalDist);
        }
        else
        {
            display.print("X");
        }
    }

    display.setCursor(35, 53);
    display.print("L_ANG:");
    display.print(lineAngle);
    display.print(",");
    display.print(lineOffset);

    drawCamMiniMap(110, 28, 14);
    display.display();
}

void camTestLoop()
{
    display.clearDisplay();
    display.setTextSize(1);

    // Title
    display.setCursor(0, 0);
    display.print("CAM MONITOR [BACK]");
    display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

    // Left column: Camera list & status
    display.setCursor(0, 13);
    display.print("> C0:");
    bool isAlive = (millis() - cam.lastPacketTime < 500);
    if (isAlive)
    {
        display.print("OK");
    }
    else
    {
        display.print("ERR");
    }

    // Split line
    display.drawLine(44, 10, 44, 63, SSD1306_WHITE);

    // Right column: Detailed data
    display.setCursor(49, 13);
    display.print("CAM INFO");

    // Ball
    display.setCursor(49, 25);
    display.print("B: ");
    if (cam.valid)
    {
        display.print(cam.ballAngle);
        display.print("/");
        display.print(cam.ballDist);
    }
    else
    {
        display.print("X");
    }

    // Yellow Goal
    display.setCursor(49, 37);
    display.print("Y: ");
    if (cam.yValid)
    {
        display.print(cam.yAngle);
        display.print("/");
        display.print(cam.yDist);
        if (cam.yInFront)
            display.print("F");
    }
    else
    {
        display.print("X");
    }

    // Blue Goal
    display.setCursor(49, 49);
    display.print("B: ");
    if (cam.bValid)
    {
        display.print(cam.bAngle);
        display.print("/");
        display.print(cam.bDist);
        if (cam.bInFront)
            display.print("F");
    }
    else
    {
        display.print("X");
    }

    display.display();
}

void drawCamMiniMap(int cx, int cy, int r)
{
    display.drawCircle(cx, cy, r, SSD1306_WHITE);
    display.drawLine(cx, cy - r, cx, cy - r + 3, SSD1306_WHITE);

    if (cam.valid && cam.ballDist < 255)
    {
        int dotDist = map(constrain(cam.ballDist, 5, 120), 5, 120, 3, r - 2);
        float ballRad = (cam.ballAngle + 180.0f) * PI / 180.0f + PI / 2;
        display.fillCircle(cx + cos(ballRad) * dotDist, cy + sin(ballRad) * dotDist, 1, SSD1306_WHITE);
    }

    if (cam.yValid)
    {
        float yRad = (cam.yAngle + 180.0f) * PI / 180.0f + PI / 2;
        int x = cx + cos(yRad) * (r - 3);
        int y = cy + sin(yRad) * (r - 3);
        display.setCursor(constrain(x - 3, 0, 122), constrain(y - 3, 0, 56));
        display.print("Y");
    }

    if (cam.bValid)
    {
        float bRad = (cam.bAngle + 180.0f) * PI / 180.0f + PI / 2;
        int x = cx + cos(bRad) * (r - 3);
        int y = cy + sin(bRad) * (r - 3);
        display.setCursor(constrain(x - 3, 0, 122), constrain(y - 3, 0, 56));
        display.print("B");
    }
}

void cameraCalibLoop()
{
    static int calibItem = 0;
    const int itemCount = 4;
    const char* itemNames[itemCount] = {
        "SENSOR",
        "BALL+YELLOW",
        "BLUE",
        "BALL ANGLE",
    };
    const uint8_t itemCmds[itemCount] = {
        0x14,
        0x15,
        0x13,
        0x00,
    };

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("OPENMV CALIB");
    display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

    for (int i = 0; i < itemCount; i++)
    {
        display.setCursor(0, 13 + i * 10);
        display.print(i == calibItem ? "> " : "  ");
        display.print(itemNames[i]);
    }

    drawCamMiniMap(108, 33, 14);

    display.setCursor(0, 56);
    display.print("SEL=CAL BACK=EXIT");
    display.display();

    if (readBtn(BTN_UP))
    {
        calibItem = (calibItem + itemCount - 1) % itemCount;
        delay(150);
        return;
    }

    if (readBtn(BTN_DOWN))
    {
        calibItem = (calibItem + 1) % itemCount;
        delay(150);
        return;
    }

    if (readBtn(BTN_SEL))
    {
        delay(200);

        if (itemCmds[calibItem] == 0x00)
        {
            runBallAngleCalibration();
            return;
        }

        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("CALIB ");
        display.println(itemNames[calibItem]);
        if (itemCmds[calibItem] == 0x14)
        {
            display.println("Reading sensor");
            display.println("Auto->ideal map");
        }
        else
        {
            display.println("Place target only");
        }
        display.println("Waiting OpenMV...");
        display.display();

        while (cam.port->available())
            cam.port->read();

        cam.port->write(0xBB);
        cam.port->write(itemCmds[calibItem]);

        if (itemCmds[calibItem] == 0x15)
        {
            // 球+黃色一次校正：OpenMV 同一個窗口算完兩色後，會依序回兩包
            // 0xCC 回應 (cmd 0x11 球、cmd 0x12 黃色)，這裡兩包都要等到。
            showBallYellowCalibResult();
            return;
        }

        bool gotResponse = false;
        bool ok = false;
        uint8_t th[6] = {0};
        unsigned long startWait = millis();

        while (millis() - startWait < 3000)
        {
            if (cam.port->available() == 0)
            {
                delay(2);
                continue;
            }

            if (cam.port->peek() != 0xCC)
            {
                cam.port->read();
                continue;
            }

            if (cam.port->available() < 10)
            {
                delay(2);
                continue;
            }

            cam.port->read();
            uint8_t cmd = cam.port->read();
            uint8_t status = cam.port->read();
            for (int i = 0; i < 6; i++)
            {
                th[i] = cam.port->read();
            }
            uint8_t tail = cam.port->read();

            if (cmd == itemCmds[calibItem] && tail == 0x55)
            {
                gotResponse = true;
                ok = (status != 0);
                break;
            }
        }

        display.clearDisplay();
        display.setCursor(0, 0);
        if (gotResponse && ok)
        {
            display.println("CALIB SAVED");
            display.print(itemNames[calibItem]);
            display.println(" CSV OK");
            if (itemCmds[calibItem] == 0x14)
            {
                int exposure = ((int)th[0] << 8) | th[1];
                display.print("EXP:");
                display.println(exposure);
                display.print("RGB:");
                display.print(th[2]);
                display.print(",");
                display.print(th[3]);
                display.print(",");
                display.println(th[4]);
                display.print("GAIN:");
                display.print(th[5]);
            }
            else
            {
                display.print(th[0]);
                display.print(",");
                display.print(th[1]);
                display.print(" ");
                display.print((int)th[2] - 128);
                display.print(",");
                display.print((int)th[3] - 128);
                display.setCursor(0, 36);
                display.print((int)th[4] - 128);
                display.print(",");
                display.print((int)th[5] - 128);
            }
        }
        else if (gotResponse)
        {
            display.println("CALIB FAILED");
            if (itemCmds[calibItem] == 0x14)
                display.println("Sensor error");
            else
                display.println("Target not found");
        }
        else
        {
            display.println("CALIB TIMEOUT");
            display.println("Check OpenMV UART");
        }
        display.display();
        delay(1800);
    }
}

bool sampleRawBallAngle(float& rawAngle)
{
    const unsigned long sampleMs = 350;
    unsigned long start = millis();
    float sumX = 0.0f;
    float sumY = 0.0f;
    int count = 0;

    while (millis() - start < sampleMs)
    {
        parseCameraPort(cam, CAM_READ_BYTE_BUDGET);
        if (cam.valid)
        {
            float rad = norm180((float)cam.ballAngle) * PI / 180.0f;
            sumX += cosf(rad);
            sumY += sinf(rad);
            count++;
        }
        delay(5);
    }

    if (count == 0)
        return false;

    rawAngle = norm180(atan2f(sumY, sumX) * 180.0f / PI);
    return true;
}

void runBallAngleCalibration()
{
    const int shownTargets[BALL_ANGLE_CAL_COUNT] = {0, 45, 135, 225, 315};
    const float targetAngles[BALL_ANGLE_CAL_COUNT] = {0, 45, 135, -135, -45};

    while (readBtn(BTN_SEL))
        delay(10);

    for (int i = 0; i < BALL_ANGLE_CAL_COUNT; i++)
    {
        bool sampled = false;
        float rawAngle = 0.0f;

        while (!sampled)
        {
            parseCameraPort(cam, CAM_READ_BYTE_BUDGET);

            display.clearDisplay();
            display.setTextSize(1);
            display.setCursor(0, 0);
            display.println("BALL ANG CAL");
            display.drawLine(0, 9, 127, 9, SSD1306_WHITE);
            display.setCursor(0, 14);
            display.print("Pos ");
            display.print(i + 1);
            display.print("/5  Target:");
            display.println(shownTargets[i]);
            display.setCursor(0, 30);
            display.print("Raw:");
            if (cam.valid)
                display.print(cam.ballAngle);
            else
                display.print("NO BALL");
            display.setCursor(0, 48);
            display.print("SEL=SAVE BACK=EXIT");
            display.display();

            if (readBtn(BTN_BACK))
            {
                delay(200);
                return;
            }

            if (readBtn(BTN_SEL))
            {
                sampled = sampleRawBallAngle(rawAngle);
                if (!sampled)
                {
                    display.clearDisplay();
                    display.setCursor(0, 0);
                    display.println("NO BALL SAMPLE");
                    display.println("Try again");
                    display.display();
                    delay(800);
                }
                while (readBtn(BTN_SEL))
                    delay(10);
            }

            delay(20);
        }

        ballAngleCalRaw[i] = rawAngle;
        ballAngleCalTarget[i] = targetAngles[i];
    }

    saveBallAngleCalibration();

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("BALL ANG SAVED");
    for (int i = 0; i < BALL_ANGLE_CAL_COUNT && i < 4; i++)
    {
        display.setCursor(0, 12 + i * 10);
        display.print((int)ballAngleCalRaw[i]);
        display.print(" -> ");
        display.print((int)ballAngleCalTarget[i]);
    }
    display.display();
    delay(1800);
}

void showBallYellowCalibResult()
{
    bool gotBall = false, gotYellow = false;
    bool ballOk = false, yellowOk = false;
    uint8_t ballTh[6] = {0};
    uint8_t yellowTh[6] = {0};
    unsigned long startWait = millis();

    while (millis() - startWait < 3000 && !(gotBall && gotYellow))
    {
        if (cam.port->available() == 0)
        {
            delay(2);
            continue;
        }

        if (cam.port->peek() != 0xCC)
        {
            cam.port->read();
            continue;
        }

        if (cam.port->available() < 10)
        {
            delay(2);
            continue;
        }

        cam.port->read();
        uint8_t cmd = cam.port->read();
        uint8_t status = cam.port->read();
        uint8_t th[6];
        for (int i = 0; i < 6; i++)
        {
            th[i] = cam.port->read();
        }
        uint8_t tail = cam.port->read();

        if (tail != 0x55)
            continue;

        if (cmd == 0x11)
        {
            gotBall = true;
            ballOk = (status != 0);
            for (int i = 0; i < 6; i++)
                ballTh[i] = th[i];
        }
        else if (cmd == 0x12)
        {
            gotYellow = true;
            yellowOk = (status != 0);
            for (int i = 0; i < 6; i++)
                yellowTh[i] = th[i];
        }
    }

    display.clearDisplay();
    display.setCursor(0, 0);

    if (!gotBall || !gotYellow)
    {
        display.println("CALIB TIMEOUT");
        display.println("Check OpenMV UART");
    }
    else
    {
        display.print("BALL ");
        display.println(ballOk ? "OK" : "FAIL");
        display.print(ballTh[0]);
        display.print(",");
        display.print(ballTh[1]);
        display.print(" ");
        display.print((int)ballTh[2] - 128);
        display.print(",");
        display.println((int)ballTh[3] - 128);

        display.print("YELLOW ");
        display.println(yellowOk ? "OK" : "FAIL");
        display.print(yellowTh[0]);
        display.print(",");
        display.print(yellowTh[1]);
        display.print(" ");
        display.print((int)yellowTh[2] - 128);
        display.print(",");
        display.println((int)yellowTh[3] - 128);
    }

    display.display();
    delay(2200);
}
