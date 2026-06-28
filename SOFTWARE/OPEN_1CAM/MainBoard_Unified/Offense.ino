// --------------------------------------------------
// 主模式
// --------------------------------------------------
int driveAngle = 0;
int driveSpeed = 0;
int driveCmp = 0;
bool driveValid = false;

void moving(int angle, int speed, int cmp = 0)
{
    driveAngle = angle;
    driveSpeed = speed;
    driveCmp = cmp;
    driveValid = true;
}

unsigned long borderReleaseTime = 0;
int lastBorderDangerAngle = 0;
bool wasOnBorder = false;

const unsigned long BORDER_SLOW_MS = 800;
const int BORDER_SLOW_ANGLE = 80;

void handleOffenseMode()
{
    if (liftedRX)
    {
        dribbler(0);
        stopAllMotors();
        return;
    }

    driveValid = false;

    if (backAngle != -1)
    {
        border();
        if (driveValid)
            move(driveAngle, driveSpeed, driveCmp);
        else
            move(0, 0, 0);
        return;
    }

    if (ballAngle != -1)
        chaseBall();
    else
        home();

    // 防止卡禁區, 向前直走
    if (driveAngle > 0 && ownGoalAngle < 130 && ownGoalAngle > 0 && ownGoalDist < 85)
    {
        driveAngle = 0;
        driveSpeed = 40;
    }
    else if (driveAngle < 0 && ownGoalAngle > -130 && ownGoalAngle < 0 && ownGoalDist < 85)
    {
        driveAngle = 0;
        driveSpeed = 40;
    }

    applyBorderSlowdown();

    if (driveValid)
        move(driveAngle, driveSpeed, driveCmp);
    else
        move(0, 0, 0);
}

// --------------------------------------------------
// 無球回防邏輯
// --------------------------------------------------
void home()
{
    dribbler(0);
    ballOutStartTime = 0;

    int spX = 0;
    int spY = 0;
    int speedLimit = (int)(CHASE_SPEED * 0.6f);  // 退防與左右平移速度

    // 1. 左右平移 (目標：將球門鎖定在正前方/正後方 +-10度)
    // 優先使用身後的己方球門來進行精準定位
    if (ownGoalAngle != -1)
    {
        // 己方門在身後 (180度)，計算其相對於 180 度的誤差
        float checkAngle = norm180(180.0f - (ownGoalAngle + currentCompass));
        if (checkAngle > 10)
            spX = speedLimit;
        else if (checkAngle < -10)
            spX = -speedLimit;
    }
    else if (goalValid)
    {
        // 看不見己方門，改看敵方球門
        if (goalWorldAngle > 10)
            spX = speedLimit;
        else if (goalWorldAngle < -10)
            spX = -speedLimit;
    }

    // 2. 前後保持 (優先使用己方球門距離)
    if (ownGoalAngle != -1)
    {
        if (ownGoalDist > 85)
            spY = -speedLimit;
        else if (ownGoalDist < 65)
            spY = speedLimit;
    }
    else if (goalValid)
    {
        // 看不見己方門時，用敵方門輔助
        if (goalDistance < 235)
            spY = -speedLimit;
        else if (goalDistance > 245)
            spY = speedLimit;
    }
    else
    {
        spY = -speedLimit;  // 什麼都看不到，預設退防
    }

    // === 絕對防出界保護 (Penalty Line Border) ===
    if (lineBack && spY < 0)
        spY = 0;
    if (lineLeft && spX < 0)
        spX = 0;
    if (lineRight && spX > 0)
        spX = 0;

    // 3. 混合輸出馬力
    if (spX == 0 && spY == 0)
    {
        moving(0, 0);
    }
    else
    {
        moving((int)(atan2f(spX, spY) * 180.0f / PI), speedLimit);
    }
}

// --------------------------------------------------
// 追球邏輯
// --------------------------------------------------
static unsigned long ballHoldingTime = 0;

void chaseBall()
{
    ballAngle = norm180(ballAngle);
    unsigned long now = millis();
    int sp = CHASE_SPEED;

    // 1. 近距離轉頭 cmp_offset
    cmp_offset = 0;
    // if (ballDistance <= 30 && abs(ballWorldAngle) > 8 && abs(ballWorldAngle) < 30)
    // {
    //     if (ballAngle > 0)
    //         cmp_offset = 10;
    //     else
    //         cmp_offset = -10;
    // }
    // else
    // {
    //     cmp_offset = 0;
    // }

    // 2. 正前方：直衝 + 持球踢球
    if (ballAngle != -1 && abs(ballAngle) <= ANGLE_STRAIGHT)
    {
        dribbler(DRIBBLER_SLOW);

        if (ballIn)
        {
            if (ballHoldingTime == 0)
                ballHoldingTime = now;

            bool goalShootOk = !goalValid || goalWorldAngle == -1 || abs(goalWorldAngle) < 20;
            // 持球時間 + 看不到球筐或球筐在正前方 + 不在角落
            if (now - ballHoldingTime > 100 && goalShootOk)
            {
                kickBall();
            }
        }
        else
        {
            ballHoldingTime = 0;
        }

        moving(0, sp, cmp_offset);
        return;
    }
    ballHoldingTime = 0;

    // 3. 遠距離：偏移接近
    if (ballDistance > DIST_WEAK)
    {
        dribbler(0);
        int approachAngle = ballAngle;
        if (abs(ballAngle) > 90)
        {
            if (ballAngle > 0)
                approachAngle += WEAK_OFFSET * 1.5;
            else
                approachAngle -= WEAK_OFFSET * 1.5;
        }
        else
        {
            if (ballAngle > 0)
                approachAngle += WEAK_OFFSET;
            else
                approachAngle -= WEAK_OFFSET;
        }
        moving((int)approachAngle, sp, cmp_offset);
        return;
    }

    // 4. 正常繞球
    if (abs(lastGoalWorldAngle) < 40)
    {
        dribbler(DRIBBLER_SLOW);
    }
    else
    {
        dribbler(0);
    }
    float run_angle = ballAngle;
    if (abs(ballAngle) <= ANGLE_MID)
    {
        if (ballAngle >= 0)
            run_angle = ballAngle * SCALE_RIGHT;
        else
            run_angle = ballAngle * SCALE_LEFT;
    }
    else
    {
        if (ballAngle > 0)
            run_angle += OFFSET_RIGHT;
        else
            run_angle -= OFFSET_LEFT;
    }

    bool sideWrap = abs(ballAngle) >= 40 && abs(ballAngle) <= 50 && ballDistance < 25;
    if (sideWrap)
    {
        sp = (int)(sp * 0.9f - ((DIST_WEAK - ballDistance) * 2));
        sp = max(30, sp);
    }

    moving((int)run_angle, sp, cmp_offset);
}

// --------------------------------------------------
// 邊角處理邏輯 (Corner Movement)
// --------------------------------------------------
// 備忘：
// moveForMs(angle, speed, cmp_offset, ms, abortOnBall, abortFront, abortLeft, abortRight)
// rotateToAngle(targetAngle, speed, abortOnBall, m1, m2, m3, m4)
void handleCornerMovement()
{
    dribbler(DRIBBLER_FAST - 15);

    // 1. 直行前進 1 秒 (丟球或踩前線則中斷)
    // if (!moveForMs(0, 30, 0, 1000, true, true, false, false))
    //     return;

    // 2. 原地停留 0.5 秒
    if (!moveForMs(0, 0, 0, 500, true))
        return;

    // 3. 往側邊平移 2 秒 (丟球或踩左右線則中斷)
    int slideAngle;
    if (lastGoalWorldAngle > 0)
    {
        slideAngle = 90;
        if (!moveForMs(slideAngle, 30, 0, 2000, true, false, false, true))
            return;
    }
    else
    {
        slideAngle = -90;
        if (!moveForMs(slideAngle, 35, 0, 2000, true, false, true, false))
            return;
    }

    // 4. 原地停留 0.5 秒
    if (!moveForMs(0, 0, 0, 500, true))
        return;

    // 5. 後退0.5秒
    if (!moveForMs(180, 30, 0, 300, true, false, false, false))
        return;

    // 6. 轉面向球門：有看到球門用即時角度，沒看到就用 lastGoalAngle 轉成 world angle
    float targetGoalAngle =
        (goalValid && goalWorldAngle != -1) ? goalWorldAngle : norm180(lastGoalAngle + currentCompass);
    float error = constrain(targetGoalAngle, -60.0f, 60.0f);
    bool rotateDone;
    if (error > 0)
    {
        // 角度為正：使用後輪旋轉 (馬達 3, 4)
        if (!rotateToAngle(error, 20, true, false, false, true, true))
            return;
    }
    else
    {
        // 角度為負：使用前輪旋轉 (馬達 1, 2)
        if (!rotateToAngle(error, 20, true, true, true, false, false))
            return;
    }

    if (ballIn)
    {
        kickBall();
        dribbler(DRIBBLER_SLOW - 10);
    }
}

// --------------------------------------------------
// 邊角擈球 + 射門序列
// --------------------------------------------------
// 備忘：
// rotateToAngle(targetAngle, speed, abortOnBall)
// moveForMs(angle, speed, cmp_offset, ms, abortOnBall)
void cornerCatch()
{
    // 球不見 或球在背後 直接放棄
    if (ballAngle == -1 || abs(ballWorldAngle) > 60)
        return;

    dribbler(DRIBBLER_FAST);

    // 1. 面向球 (自轉)
    if (!rotateToAngle(ballWorldAngle, 14, false))
        return;

    // 2. 朝球前進並修正角度
    if (!moveForMs(0, 20, ballWorldAngle, 550, false))
        return;

    // 3. 往場內退後 (不因丟球中斷)
    if (!moveForMs(180, 20, ballWorldAngle, 500, false))
        return;

    // 4. 緩慢轉回前方 (球不見中斷)
    if (!rotateToAngle(0, 14, true))
        return;
}

// --------------------------------------------------
// 邊界與異常處理
// --------------------------------------------------
unsigned long edgeBallStopStart = 0;

void applyBorderSlowdown()
{
    unsigned long now = millis();

    if (wasOnBorder)
    {
        borderReleaseTime = now;
        wasOnBorder = false;
    }

    unsigned long elapsed = now - borderReleaseTime;
    if (elapsed < BORDER_SLOW_MS && abs(angleDiff(driveAngle, lastBorderDangerAngle)) < BORDER_SLOW_ANGLE)
    {
        float t = (float)elapsed / BORDER_SLOW_MS;
        driveSpeed = (int)(driveSpeed * (0.45f + t * t * 0.45f));
    }

    edgeBallStopStart = 0;
    stuckStartTime = 0;
}

bool border()
{
    unsigned long now = millis();

    lastBorderDangerAngle = norm180(backAngle + 180);
    wasOnBorder = true;

    // 2.1 敵方球筐角落原地停留：等待角落射門前先關吸球
    if (edgeStanding && ballAngle != -1 && abs(ballAngle) < 60 && abs(lastBorderDangerAngle) < 80)
    {
        dribbler(0);
        moving(0, 0);

        if (edgeBallStopStart == 0)
            edgeBallStopStart = now;

        // 已吸球 -> 執行角落射門
        if (ballIn && abs(ballAngle) < 30 && abs(lastGoalWorldAngle) >= 50)
        {
            handleCornerMovement();
            edgeBallStopStart = 0;
        }
        // 卡住超過 2 秒且未吸到球 -> 嘗試角落撈球
        else if (now - edgeBallStopStart > 2000 && ballDistance < 25)
        {
            cornerCatch();
            edgeBallStopStart = 0;
        }
        return true;
    }

    // 2.2 正常踩線逃脫 (往反方向逃跑，並避免撞到球)
    edgeBallStopStart = 0;
    int escapeAngle = backAngle;
    bool ballBehind = ballAngle != -1 && abs(abs(ballAngle) - 180) < 25;
    bool escapingBackward = abs(abs(escapeAngle) - 180) < 45;
    if (ballBehind && escapingBackward)
    {
        if (ballAngle > 0)
            escapeAngle = -150;
        else
            escapeAngle = 150;
    }
    moving(escapeAngle, BORDER_ESCAPE_SPEED, 0);
    return true;
}
