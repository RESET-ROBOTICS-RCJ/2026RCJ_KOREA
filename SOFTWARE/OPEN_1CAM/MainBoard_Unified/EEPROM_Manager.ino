// --------------------------------------------------
// EEPROM 載入
// --------------------------------------------------
void resetEEPROMToDefaults()
{
    // --- 陣列默認值 ---
    for (int i = 0; i < EEP_COUNT; i++)
    {
        params[i] = 0;  // 預設歸零
    }

    params[0] = 55;   // Chase Speed (基礎行駛速度百分比 0-100)
    params[1] = 1.3;  // Scale L (左前側角度倍率)
    params[2] = 1.3;  // Scale R (右前側角度倍率)
    params[3] = 45;   // Offset L (左後側角度偏移)
    params[4] = 45;   // Offset R (右後側角度偏移)
    params[5] = 15;   // Dist Straight (嘴部近距離對齊判定距)
    params[6] = 20;   // Ang Straight (視為「正前方」可以直接前進的度數 +/-)
    params[7] = 25;   // Dist Weak (遠距離螺旋追球判定距)
    params[8] = 25;   // Ang Weak / WEAK_OFFSET (遠距離增加的角度偏移)
    params[9] = 50;   // Ang Mid / ANGLE_MID (繞球區分中偏度數門檻)
    params[10] = 80;  // Drib Fast (運動時吸球速度)
    params[11] = 40;  // Drib Slow (不動時吸球速度)
    params[12] = 45;  // Border Esc (踩線逃脫速度)
    params[13] = 15;  // BallIn D (camera distance threshold)
    params[14] = 100; // BallIn Hold (ms)
    params[15] = 0;   // Play Mode (0: ATK, 1: DEF)
    params[16] = 0;   // Goal Side (0: attack Blue, 1: attack Yellow)
    params[17] = 0.8; // Rot Scale (heading correction strength)

    EEPROM.put(EEPROM_ADDR, params);
}

void loadParams()
{
    EEPROM.get(EEPROM_ADDR, params);

    // 合理性檢查：如果數據被清除或損壞（速度 <= 0, > 100 或 NaN），載入默認值
    if (params[0] <= 0 || params[0] > 100 || isnan(params[0]))
    {
        resetEEPROMToDefaults();
    }

    // 將 EEPROM 參數映射到運行中的全局變數
    CHASE_SPEED = params[0];     // 基礎行駛速度百分比 (0-100)
    SCALE_LEFT = params[1];      // 左前側角度倍率
    SCALE_RIGHT = params[2];     // 右前側角度倍率
    OFFSET_LEFT = params[3];     // 左後側角度偏移
    OFFSET_RIGHT = params[4];    // 右後側角度偏移
    DIST_STRAIGHT = params[5];   // 球在嘴巴附近的距離
    ANGLE_STRAIGHT = params[6];  // 視為「正前方」的度數 +/-
    DIST_WEAK = params[7];       // 球遠時的距離
    WEAK_OFFSET = params[8];     // 球遠時增加的角度偏移
    ANGLE_MID = params[9];       // 開始大角度繞球的角度

    bool dribblerParamsUnset = (params[10] == 0 && params[11] == 0);
    if (dribblerParamsUnset || isnan(params[10]) || params[10] < 0 || params[10] > 180)
    {
        params[10] = 80;
    }
    if (dribblerParamsUnset || isnan(params[11]) || params[11] < 0 || params[11] > 180)
    {
        params[11] = 40;
    }
    DRIBBLER_FAST = (int)params[10];  // 運動時吸球速度
    DRIBBLER_SLOW = (int)params[11];  // 不動時吸球速度

    if (isnan(params[12]) || params[12] <= 0 || params[12] > 100)
    {
        params[12] = 45;
    }
    BORDER_ESCAPE_SPEED = (int)params[12];  // 踩線逃脫速度

    if (isnan(params[13]) || params[13] <= 0 || params[13] > 255)
    {
        params[13] = 15;
    }
    BALL_IN_DISTANCE = (int)params[13];  // 主控判斷吸球的距離門檻

    if (isnan(params[14]) || params[14] < 0 || params[14] > 1000)
    {
        params[14] = 100;
    }
    BALL_IN_HOLD_MS = (unsigned long)params[14];  // 距離達標後需維持的時間

    playMode = params[15];
    // 驗證 params[16] (Goal Side) 合法性
    if (params[16] != 0 && params[16] != 1)
    {
        params[16] = 0;  // 預設攻藍門
    }

    if (isnan(params[17]) || params[17] < 0.0f || params[17] > 2.0f)
    {
        params[17] = 0.8f;
    }
    rotate_scale = params[17];

    // Load Camera Angle Calibration
    EEPROM.get(100, camScaleFactor);
    EEPROM.get(100 + sizeof(camScaleFactor), camAngleBias);
    for (int i = 0; i < 1; i++)
    {
        if (isnan(camScaleFactor[i]) || camScaleFactor[i] <= 0.1f || camScaleFactor[i] > 5.0f)
        {
            camScaleFactor[i] = 1.0f;
        }
        if (isnan(camAngleBias[i]) || camAngleBias[i] < -90.0f || camAngleBias[i] > 90.0f)
        {
            camAngleBias[i] = 0.0f;
        }
    }

    uint32_t ballAngleMagic = 0;
    EEPROM.get(BALL_ANGLE_CAL_ADDR, ballAngleMagic);
    if (ballAngleMagic == BALL_ANGLE_CAL_MAGIC)
    {
        int addr = BALL_ANGLE_CAL_ADDR + sizeof(ballAngleMagic);
        EEPROM.get(addr, ballAngleCalRaw);
        addr += sizeof(ballAngleCalRaw);
        EEPROM.get(addr, ballAngleCalTarget);

        ballAngleCalValid = true;
        for (int i = 0; i < BALL_ANGLE_CAL_COUNT; i++)
        {
            if (isnan(ballAngleCalRaw[i]) || isnan(ballAngleCalTarget[i]) ||
                ballAngleCalRaw[i] < -360.0f || ballAngleCalRaw[i] > 360.0f ||
                ballAngleCalTarget[i] < -360.0f || ballAngleCalTarget[i] > 360.0f)
            {
                ballAngleCalValid = false;
            }
        }
    }
    else
    {
        ballAngleCalValid = false;
    }
}

void saveBallAngleCalibration()
{
    int addr = BALL_ANGLE_CAL_ADDR;
    EEPROM.put(addr, BALL_ANGLE_CAL_MAGIC);
    addr += sizeof(BALL_ANGLE_CAL_MAGIC);
    EEPROM.put(addr, ballAngleCalRaw);
    addr += sizeof(ballAngleCalRaw);
    EEPROM.put(addr, ballAngleCalTarget);
    ballAngleCalValid = true;
}

// --------------------------------------------------
// EEPROM 調整
// --------------------------------------------------
void adjustEepromParam(int dir)
{
    if (eepIndex == 15)
    {
        if (dir > 0)
            params[15] = 0;  // ATK (0)
        else if (dir < 0)
            params[15] = 1;  // DEF (1)
        return;
    }

    if (eepIndex == 16)
    {
        if (dir > 0)
            params[16] = 0;  // Blue (0)
        else if (dir < 0)
            params[16] = 1;  // Yellow (1)
        return;
    }

    float step = 1.0;
    // 根據不同索引設定不同的調整步長
    if (eepIndex == 0 || eepIndex == 10 || eepIndex == 11 || eepIndex == 12)
        step = 5.0;  // 速度每次調整 5
    else if (eepIndex == 14)
        step = 10.0;  // BallIn Hold 每次調整 10ms
    else if (eepIndex == 1 || eepIndex == 2 || eepIndex == 17)
        step = 0.1;  // 倍率調整 0.1
    else
        step = 1.0;  // 其他皆為整數 1

    params[eepIndex] += dir * step;

    // 限制極端值避免當機
    if (eepIndex == 0)
    {
        params[0] = constrain(params[0], 0, 100);
    }
    else if (eepIndex == 10 || eepIndex == 11)
    {
        params[eepIndex] = constrain(params[eepIndex], 0, 180);
    }
    else if (eepIndex == 12)
    {
        params[12] = constrain(params[12], 0, 100);
    }
    else if (eepIndex == 13)
    {
        params[13] = constrain(params[13], 1, 255);
    }
    else if (eepIndex == 14)
    {
        params[14] = constrain(params[14], 0, 1000);
    }
    else if (eepIndex == 17)
    {
        params[17] = constrain(params[17], 0.0, 2.0);
    }
    else if (params[eepIndex] < 0)
    {
        // 預設簡單防止其它數值小於0 (可依照需求刪除或修改)
        // params[eepIndex] = 0;
    }
}

// --------------------------------------------------
// EEPROM UI 迴圈
// --------------------------------------------------
void eepromModeLoop()
{
    int eepUiPos = 0;
    for (int i = 0; i < EEP_UI_COUNT; i++)
    {
        if (eepUiIndices[i] == eepIndex)
        {
            eepUiPos = i;
            break;
        }
    }
    eepIndex = eepUiIndices[eepUiPos];

    // --- 輸入邏輯 ---
    if (!eepEditing)
    {
        if (readBtn(BTN_UP))
        {
            eepUiPos = (eepUiPos - 1 + EEP_UI_COUNT) % EEP_UI_COUNT;
            eepIndex = eepUiIndices[eepUiPos];
            delay(150);
        }
        if (readBtn(BTN_DOWN))
        {
            eepUiPos = (eepUiPos + 1) % EEP_UI_COUNT;
            eepIndex = eepUiIndices[eepUiPos];
            delay(150);
        }
        if (readBtn(BTN_SEL))
        {
            eepEditing = true;
            delay(200);
        }
    }
    else
    {
        // 編輯邏輯
        if (readBtn(BTN_UP))
        {
            adjustEepromParam(+1);
            delay(80);  // 快速重覆
        }
        if (readBtn(BTN_DOWN))
        {
            adjustEepromParam(-1);
            delay(80);
        }
        if (readBtn(BTN_SEL))
        {
            eepEditing = false;
            EEPROM.put(EEPROM_ADDR, params);
            loadParams();
            delay(300);
        }
    }

    // --- 繪製邏輯 ---
    display.clearDisplay();
    display.setTextSize(1);

    // 佈局：每頁 1 欄，7 行 (全螢幕)
    const int rowsPerPage = 7;
    const int rowHeight = 9;
    int currentPage = eepUiPos / rowsPerPage;
    int startIdx = currentPage * rowsPerPage;

    // 在右上角繪製頁碼指示器 (例如 "1/3")
    display.setCursor(120, 0);
    display.print(currentPage + 1);

    for (int i = 0; i < rowsPerPage; i++)
    {
        int uiPos = startIdx + i;
        if (uiPos >= EEP_UI_COUNT)
            break;
        int idx = eepUiIndices[uiPos];

        int yPos = i * rowHeight;
        display.setCursor(0, yPos);

        // 光標邏輯
        if (idx == eepIndex)
        {
            if (eepEditing)
            {
                display.print("*");
            }
            else
            {
                display.print(">");
            }
        }
        else
        {
            display.print(" ");
        }

        // 顯示參數名稱 (照順序印出 eepNames)
        display.print(eepNames[idx]);
        display.print(":");

        // 顯示數值 (更緊湊地與標籤對齊)
        int valueX = 88;
        if (idx == 1 || idx == 2 || idx == 17)
            valueX = 84;

        display.setCursor(valueX, yPos);

        // 簡單判斷：如果是小數步長，就印一位小數
        if (idx == 15)
        {
            if (params[15] == 1)
                display.print("DEF");
            else
                display.print("ATK");
        }
        else if (idx == 16)
        {
            if (params[16] == 1)
                display.print("YEL");
            else
                display.print("BLU");
        }
        else if (idx == 1 || idx == 2 || idx == 17)
        {
            display.print(params[idx], 1);
        }
        else
        {
            display.print((int)params[idx]);
        }
    }

    display.display();
}
