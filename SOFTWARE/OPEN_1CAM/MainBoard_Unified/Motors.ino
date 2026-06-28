// --------------------------------------------------
// 馬達控制 (id: 1-4, speed: -100 ~ 100)
// --------------------------------------------------
void motor(int id, int speedPercent)
{
    if (id < 1 || id > 4)
        return;
    int pwmA = motorPins[id - 1][0];
    int pwmB = motorPins[id - 1][1];
    speedPercent = constrain(speedPercent, -100, 100);
    speedPercent *= -1;
    int pwmValue = map(abs(speedPercent), 0, 100, 0, 255);

    if (speedPercent > 0)
    {
        digitalWrite(pwmA, LOW);
        analogWrite(pwmB, pwmValue);
    }
    else if (speedPercent < 0)
    {
        digitalWrite(pwmA, HIGH);
        analogWrite(pwmB, pwmValue);
    }
    else
    {
        digitalWrite(pwmA, LOW);
        analogWrite(pwmB, 0);
    }
}

void stopAllMotors()
{
    for (int i = 1; i <= 4; i++)
        motor(i, 0);
}

void rotate(int speed, bool m1, bool m2, bool m3, bool m4)
{
    motor(1, m1 ? -speed : 0);
    motor(2, m2 ? -speed : 0);
    motor(3, m3 ? -speed : 0);
    motor(4, m4 ? -speed : 0);
}

bool moveForMs(int angle, int speed, int cmp_offset, int ms, bool abortOnBallLost, bool abortOnFrontLine,
               bool abortOnLeftLine, bool abortOnRightLine)
{
    unsigned long start = millis();
    unsigned long ballLostStart = 0;
    while (millis() - start < (unsigned long)ms)
    {
        updateSensors();
        if (liftedRX || ballAngle == -1)
        {
            stopAllMotors();
            return false;
        }

        if (abortOnBallLost && !ballIn)
        {
            stopAllMotors();
            return false;
        }

        if ((abortOnFrontLine && lineFront) || (abortOnLeftLine && lineLeft) || (abortOnRightLine && lineRight))
        {
            stopAllMotors();
            return true;  // 動作成功回傳true
        }
        move(angle, speed, cmp_offset);
    }
    stopAllMotors();
    return true;
}

bool rotateToAngle(float targetAngle, int speed, bool abortOnBallLost, bool m1, bool m2, bool m3, bool m4)
{
    unsigned long start = millis();
    unsigned long ballLostStart = 0;
    while (millis() - start < 2000)
    {  // 2 秒安全超時
        updateSensors();
        if (liftedRX || ballAngle == -1)
        {
            stopAllMotors();
            return false;
        }

        if (abortOnBallLost && !ballIn)
        {
            stopAllMotors();
            return false;
        }

        float error = norm180(targetAngle - currentCompass);
        if (abs(error) < 4.0f)
        {
            stopAllMotors();
            return true;  // 動作成功回傳true
        }
        if (error > 0)
            rotate(-speed, m1, m2, m3, m4);
        else
            rotate(speed, m1, m2, m3, m4);
    }
    stopAllMotors();
    return false;
}

// --------------------------------------------------
// 指南針移動 (angle: 方向角度 0=前, speed: 速度幅度)
// --------------------------------------------------
void move(int angle, int speed, float cmp_offset)
{
    float dir_rad = angle * PI / 180.0f;
    int py = (int)(speed * cos(dir_rad));
    int px = (int)(speed * sin(dir_rad));

    float heading = norm180(currentCompass - cmp_offset);
    float ts = heading;
    if (ts > 60.0f)
        ts = 60.0f;
    if (ts < -60.0f)
        ts = -60.0f;
    float p = 100.0f * (60.0f - abs(ts)) / 60.0f;
    ts = ts * rotate_scale;
    int wrap_heading = (int)(heading + 315 + 360) % 360;
    float hrad = wrap_heading * PI / 180.0f;
    int m1 = (int)(p * (px * cos(hrad) - py * sin(hrad)) / 100.0f);
    int m2 = (int)(p * (py * cos(hrad) + px * sin(hrad)) / 100.0f);
    int m3 = -m1;
    int m4 = -m2;
    m1 -= (int)ts;
    m2 -= (int)ts;
    m3 -= (int)ts;
    m4 -= (int)ts;
    motor(1, m1);
    motor(2, m2);
    motor(3, m3);
    motor(4, m4);
}

// --------------------------------------------------
// 踢球器控制
// --------------------------------------------------
void kickBall()
{
    unsigned long now = millis();
    // 保護機制：檢查冷卻時間
    if (now - lastKickTime < KICK_COOLDOWN)
        return;
    stopAllMotors();
    dribbler(0);
    currentCompass = 0;
    digitalWrite(KICK_PIN, HIGH);
    delay(KICK_PULSE_MS);
    digitalWrite(KICK_PIN, LOW);
    lastKickTime = now;
}

// --------------------------------------------------
// 吸球控制
// --------------------------------------------------
void dribbler(int speed)
{
    speed = constrain(speed, 0, 180);
    esc.write(speed);
}

// --------------------------------------------------
// 馬達測試模式迴圈
// --------------------------------------------------
void motorTestLoop()
{
    const int ITEM_COUNT = 6;  // 4 motors + kicker + single ESC

    if (!mtEditing)
    {
        if (readBtn(BTN_UP))
        {
            mtSelected = (mtSelected - 1 + ITEM_COUNT) % ITEM_COUNT;
            delay(150);
        }
        if (readBtn(BTN_DOWN))
        {
            mtSelected = (mtSelected + 1) % ITEM_COUNT;
            delay(150);
        }

        if (readBtn(BTN_SEL))
        {
            if (mtSelected == 4)
            {
                kickBall();
            }
            else
            {
                mtEditing = true;
            }
            delay(200);
        }
    }
    else
    {
        if (readBtn(BTN_UP))
        {
            if (mtSelected < 4)
                mtSpeed[mtSelected] = min(100, mtSpeed[mtSelected] + 5);
            else if (mtSelected == 5)
                mtEscSpeed = min(180, mtEscSpeed + 10);
            delay(80);
        }
        if (readBtn(BTN_DOWN))
        {
            if (mtSelected < 4)
                mtSpeed[mtSelected] = max(-100, mtSpeed[mtSelected] - 5);
            else if (mtSelected == 5)
                mtEscSpeed = max(0, mtEscSpeed - 10);
            delay(80);
        }
        if (readBtn(BTN_SEL))
        {
            mtEditing = false;
            delay(200);
        }
    }

    // 運行操作
    for (int i = 0; i < 4; i++)
        motor(i + 1, mtSpeed[i]);
    esc.write(mtEscSpeed);

    // ===== 顯示 =====
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("MOTOR TEST ");
    if (mtEditing)
    {
        display.print("[EDIT]");
    }
    else
    {
        display.print("[SEL]");
    }

    // 顯示馬達 1-4
    for (int i = 0; i < 4; i++)
    {
        display.setCursor(0, 12 + i * 9);
        if (i == mtSelected)
        {
            if (mtEditing)
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
        display.print("M");
        display.print(i + 1);
        display.print(":");
        display.print(mtSpeed[i]);
    }

    // 踢球器行
    display.setCursor(64, 12);
    if (mtSelected == 4)
    {
        if (mtEditing)
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
    display.print("KICK");

    // ESC line
    display.setCursor(64, 21);
    if (mtSelected == 5)
    {
        if (mtEditing)
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
    display.print("ESC:");
    display.print(mtEscSpeed);

    display.display();
}
