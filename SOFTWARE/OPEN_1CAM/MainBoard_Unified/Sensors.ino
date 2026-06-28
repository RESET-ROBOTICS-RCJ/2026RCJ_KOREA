// --------------------------------------------------
// 指南針讀取
// --------------------------------------------------
float getCompass()
{
    if (!bnoValid)
    {
        return 0.0f;
    }
    static float lastHeading = 0.0f;
    static unsigned long lastBnoRead = 0;
    unsigned long now = millis();

    // Only query physical sensor via I2C at 100 Hz (BNO055 fusion refresh rate)
    if (now - lastBnoRead >= 10)
    {
        lastBnoRead = now;
        // getVector is much lighter than getEvent
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        lastHeading = norm180(euler.x() - headingOffset);
    }
    return lastHeading;
}

// --------------------------------------------------
// 攝像頭數據處理 (Single Camera)
// --------------------------------------------------
CameraInput cam = {&Serial2, {0}, 0, false, -1, 0, false, false, -1, 0, false, false, -1, 0, false, 0};

float angle360(float a)
{
    a = norm180(a);
    if (a < 0)
        a += 360.0f;
    return a;
}

float calibrateBallAngle(float rawAngle)
{
    if (!ballAngleCalValid)
    {
        return norm180(rawAngle * camScaleFactor[0] + camAngleBias[0]);
    }

    float raw[BALL_ANGLE_CAL_COUNT];
    float target[BALL_ANGLE_CAL_COUNT];
    for (int i = 0; i < BALL_ANGLE_CAL_COUNT; i++)
    {
        raw[i] = angle360(ballAngleCalRaw[i]);
        target[i] = norm180(ballAngleCalTarget[i]);
    }

    for (int i = 0; i < BALL_ANGLE_CAL_COUNT - 1; i++)
    {
        for (int j = i + 1; j < BALL_ANGLE_CAL_COUNT; j++)
        {
            if (raw[j] < raw[i])
            {
                float tmp = raw[i];
                raw[i] = raw[j];
                raw[j] = tmp;
                tmp = target[i];
                target[i] = target[j];
                target[j] = tmp;
            }
        }
    }

    float x = angle360(rawAngle);
    for (int i = 0; i < BALL_ANGLE_CAL_COUNT; i++)
    {
        int next = (i + 1) % BALL_ANGLE_CAL_COUNT;
        float x0 = raw[i];
        float x1 = raw[next];
        if (next == 0)
            x1 += 360.0f;

        float sample = x;
        if (sample < x0)
            sample += 360.0f;

        if (sample >= x0 && sample <= x1)
        {
            float ratio = (x1 == x0) ? 0.0f : (sample - x0) / (x1 - x0);
            float delta = norm180(target[next] - target[i]);
            return norm180(target[i] + delta * ratio);
        }
    }

    return norm180(rawAngle);
}

int getCalibratedBallAngle()
{
    if (!cam.valid)
    {
        return 999;
    }
    float local = norm180((float)cam.ballAngle);
    return (int)calibrateBallAngle(local);
}

void parseCameraPort(CameraInput& c, int byteBudget)
{
    while (byteBudget > 0 && c.port->available())
    {
        byteBudget--;
        uint8_t b = c.port->read();
        if (c.rxIdx == 0)
        {
            if (b == 0xA5)
            {
                c.rxBuf[c.rxIdx++] = b;
            }
            continue;
        }
        c.rxBuf[c.rxIdx++] = b;

        if (c.rxIdx == 14)
        {
            c.rxIdx = 0;
            if (c.rxBuf[13] != 0x55)
            {
                continue;
            }

            c.lastPacketTime = millis();

            // Packet: A5, Ball[flags/sign/mag/dist], Yellow[valid/sign/mag/dist], Blue[valid/sign/mag/dist], 55
            bool b_valid = (c.rxBuf[1] & 0x01) != 0;
            bool b_in = (c.rxBuf[1] & 0x02) != 0;
            int b_sign = -1;
            if (c.rxBuf[2] == 1)
            {
                b_sign = 1;
            }
            int b_mag = c.rxBuf[3];
            int b_dist = c.rxBuf[4];

            bool y_valid = (c.rxBuf[5] != 0);
            int y_sign = -1;
            if (c.rxBuf[6] == 1)
            {
                y_sign = 1;
            }
            int y_mag = c.rxBuf[7];
            int y_dist = c.rxBuf[8];

            bool blue_valid = (c.rxBuf[9] != 0);
            int blue_sign = -1;
            if (c.rxBuf[10] == 1)
            {
                blue_sign = 1;
            }
            int blue_mag = c.rxBuf[11];
            int blue_dist = c.rxBuf[12];

            // Update Ball State
            if (b_valid)
            {
                c.valid = true;
                c.ballAngle = b_sign * b_mag;
                c.ballDist = b_dist;
                c.ballIn = b_in;
            }
            else
            {
                c.valid = false;
                c.ballAngle = 999;
                c.ballDist = 255;
                c.ballIn = false;
            }

            // Update Yellow Goal State
            if (y_valid)
            {
                c.yValid = true;
                c.yAngle = y_sign * y_mag;
                c.yDist = y_dist;
                c.yInFront = (abs(c.yAngle) <= 20);
            }
            else
            {
                c.yValid = false;
                c.yAngle = 999;
                c.yDist = 255;
                c.yInFront = false;
            }

            // Update Blue Goal State
            if (blue_valid)
            {
                c.bValid = true;
                c.bAngle = blue_sign * blue_mag;
                c.bDist = blue_dist;
                c.bInFront = (abs(c.bAngle) <= 20);
            }
            else
            {
                c.bValid = false;
                c.bAngle = 999;
                c.bDist = 255;
                c.bInFront = false;
            }
        }
    }
}

void updateCameraData()
{
    static unsigned long lastCamReadTime = 0;
    static unsigned long ballInStartTime = 0;
    unsigned long now = millis();

    if (now - lastCamReadTime >= CAM_READ_INTERVAL_MS)
    {
        lastCamReadTime = now;
        parseCameraPort(cam, CAM_READ_BYTE_BUDGET);
    }

    if (cam.valid)
    {
        camBallAngle = getCalibratedBallAngle();
        camBallDist = cam.ballDist;
        camBallValid = true;
    }
    else
    {
        camBallAngle = -1;
        camBallDist = 255;
        camBallValid = false;
    }

    if (camBallValid && camBallDist <= BALL_IN_DISTANCE)
    {
        if (ballInStartTime == 0)
            ballInStartTime = now;
        ballIn = (BALL_IN_HOLD_MS == 0 || now - ballInStartTime >= BALL_IN_HOLD_MS);
    }
    else
    {
        ballInStartTime = 0;
        ballIn = false;
    }

    // 2. Yellow Goal
    gYGOALInFront = (cam.yValid && cam.yInFront);
    if (cam.yValid)
    {
        gYGOALAngle = norm180(cam.yAngle);
        gYGOALDist = cam.yDist;
    }
    else
    {
        gYGOALAngle = -1;
        gYGOALDist = 255;
    }

    // 3. Blue Goal
    gBGOALInFront = (cam.bValid && cam.bInFront);
    if (cam.bValid)
    {
        gBGOALAngle = norm180(cam.bAngle);
        gBGOALDist = cam.bDist;
    }
    else
    {
        gBGOALAngle = -1;
        gBGOALDist = 255;
    }

    // Dynamic goal assignment based on params[16]:
    // 0 = attack Blue (default), 1 = attack Yellow
    if ((int)params[16] == 1) {
        // 攻黃門：goalAngle = 黃門，己方 = 藍門
        goalAngle = gYGOALAngle;
        goalDistance = gYGOALDist;
        goalValid = (goalAngle != -1);
        goalInFront = gYGOALInFront;
        ownGoalAngle = gBGOALAngle;
        ownGoalDist = gBGOALDist;
        ownGoalValid = (ownGoalAngle != -1);
        ownGoalInFront = gBGOALInFront;
    } else {
        // 攻藍門：goalAngle = 藍門，己方 = 黃門
        goalAngle = gBGOALAngle;
        goalDistance = gBGOALDist;
        goalValid = (goalAngle != -1);
        goalInFront = gBGOALInFront;
        ownGoalAngle = gYGOALAngle;
        ownGoalDist = gYGOALDist;
        ownGoalValid = (ownGoalAngle != -1);
        ownGoalInFront = gYGOALInFront;
    }
}

// --------------------------------------------------
// 後方感測器封包 (Serial8) 統一 8-byte
// --------------------------------------------------
void readPacket()
{
    while (Serial8.available())
    {
        uint8_t b = Serial8.read();

        if (bottomAnalogIdx > 0 || b == 0xCC)
        {
            if (bottomAnalogIdx == 0)
            {
                bottomAnalogBuf[bottomAnalogIdx++] = b;
                continue;
            }

            if (bottomAnalogIdx == 1)
            {
                if (b == 0xDD)
                {
                    bottomAnalogBuf[bottomAnalogIdx++] = b;
                }
                else
                {
                    bottomAnalogIdx = 0;
                }
                continue;
            }

            bottomAnalogBuf[bottomAnalogIdx++] = b;
            if (bottomAnalogIdx == 39)
            {
                if (bottomAnalogBuf[38] == 0x55)
                {
                    for (int i = 0; i < BOTTOM_ANALOG_COUNT; i++)
                    {
                        int pos = 2 + i * 2;
                        bottomAnalogValue[i] = ((int)bottomAnalogBuf[pos] << 8) | bottomAnalogBuf[pos + 1];
                    }
                    bottomAnalogValid = true;
                    lastBottomAnalogPacketTime = millis();
                }
                bottomAnalogIdx = 0;
            }
            continue;
        }

        // Issue 1 fix: 2-byte header (0xAA 0xBB)
        // State 0: waiting for 0xAA
        if (idx == 0)
        {
            if (b == 0xAA)
            {
                buf[idx++] = b;
            }
            // else: discard noise
            continue;
        }

        // State 1: waiting for 0xBB — if wrong, restart sync
        if (idx == 1)
        {
            if (b == 0xBB)
            {
                buf[idx++] = b;
            }
            else
            {
                idx = 0;  // Resync: treat as noise, not a new start
            }
            continue;
        }

        // States 2–10: accumulate payload + end byte
        buf[idx++] = b;
        if (idx == 11)
        {
            if (buf[10] == 0x55)
            {
                lastBottomPacketTime = millis();
                uint8_t status = buf[2];
                liftedRX = (status == 255);

                // --- 防守數據 (Defense Data) ---
                lineDetected = ((status & 2) == 0) && !liftedRX;

                if (status & 1)
                {
                    lineOffset = 0;
                }
                else
                {
                    lineOffset = (float)buf[3] / 255.0f;
                }
                lineAngle = norm180((((int)buf[4] << 8) | buf[5]) - 180);

                // --- 進攻數據 (Offense Data) ---
                edgeStanding = ((status & 4) != 0);
                lineFront = (buf[8] & 1) != 0;  // Front (0°, ±20°)
                lineBack = (buf[8] & 2) != 0;   // Back  (±160°, 180°)
                lineLeft = (buf[9] & 1) != 0;   // Left  (-80°, -100°)
                lineRight = (buf[9] & 2) != 0;  // Right (+80°, +100°)

                uint16_t angleRaw = ((uint16_t)buf[6] << 8) | buf[7];
                if (angleRaw == 65535)
                {
                    backAngle = -1;
                }
                else
                {
                    backAngle = norm180((int)angleRaw);
                }
            }
            idx = 0;
        }
    }
}

// --------------------------------------------------
// 高層級數據彙整
// --------------------------------------------------
void updateSensors()
{
    // 1. Process fast sensors first (Serial/I2C)
    readPacket();
    updateCameraData();
    currentCompass = getCompass();

    if (camBallValid)
    {
        ballAngle = camBallAngle;
        ballDistance = camBallDist;
    }
    else
    {
        ballAngle = -1;
        ballDistance = 255;
    }

    // 3. 世界坐標系轉換 (相對於北方)
    if (ballAngle != -1)
    {
        ballWorldAngle = norm180(ballAngle + currentCompass);
    }
    else
    {
        ballWorldAngle = -1;
    }

    if (goalAngle != -1)
    {
        goalWorldAngle = norm180((int)goalAngle + (int)currentCompass);
    }
    else
    {
        goalWorldAngle = -1;
    }

    if (backAngle != -1)
    {
        backWorldAngle = norm180(backAngle + (int)currentCompass);
    }
    else
    {
        backWorldAngle = -1;
    }

    if (ownGoalAngle != -1)
    {
        ownGoalWorldAngle = norm180(ownGoalAngle + (int)currentCompass);
    }
    else
    {
        ownGoalWorldAngle = -1;
    }

    // 更新最後一次有效的球門角度 (用於 goalValid 為 false 時的判斷)
    if (goalValid)
    {
        lastGoalAngle = goalAngle;
        lastGoalWorldAngle = goalWorldAngle;
        lastGoalDistance = goalDistance;
    }

    if (ownGoalValid)
    {
        lastOwnGoalAngle = ownGoalAngle;
        lastOwnGoalDist = ownGoalDist;
    }
}
