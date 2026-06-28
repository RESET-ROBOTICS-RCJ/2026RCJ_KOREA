// --------------------------------------------------
// 守門員模式
// --------------------------------------------------

// 線校正推力比例 (相對 CHASE_SPEED)；slide (追球/回中橫移) 統一用 CHASE_SPEED 全速，不分段減速。
const float DEF_PUSH_MIN_RATIO = 0.1;     // 線校正推力：lineOffset=0 時
const float DEF_PUSH_MAX_RATIO = 1.0;     // 線校正推力：lineOffset=1 時
const float DEF_CLEAR_CHASE_RATIO = 1.0;  // 主動清球衝刺速度

// 衝出去踢球，再沿己方球門方向退回防線
void handleActiveClear()
{
    dribbler(DRIBBLER_FAST);

    // 1. 朝球衝 (最多 1 秒，球不見或被拿起則中止)
    unsigned long start = millis();
    while (millis() - start < 1000)
    {
        updateSensors();
        if (liftedRX || ballAngle == -1)
            break;
        move((int)(ballAngle * 0.7), (int)(CHASE_SPEED * DEF_CLEAR_CHASE_RATIO));
    }

    kickBall();
    dribbler(0);

    // 2. 退回防線 (踩到線或被拿起就停)
    while (true)
    {
        updateSensors();
        if (liftedRX || lineDetected)
            break;
        move(lastOwnGoalAngle != -1 ? lastOwnGoalAngle : 180, 40);
    }

    stopAllMotors();
}

// 在防守線上的移動：線校正推力push + 追球或對齊中心的橫向滑行slide
// slide 一律用 CHASE_SPEED 全速，死區內 (角度夠小/已對準) 才不滑
void onLineMove(float& angle, float& speed)
{
    float pushX = 0, pushY = 0, slideX = 0, slideY = 0;

    // 線校正：幾乎置中不用校正。感測器只有 18 顆、20° 一格，偏移一格時 lineOffset 至少跳到 ~0.174，
    // 死區設 0.1 (< 0.174) 純粹是排除 lineOffset=0 置中的情況，本身擋不到中間值 (因為量化後不存在)
    if (lineOffset > 0.1)
    {
        float pMin = CHASE_SPEED * DEF_PUSH_MIN_RATIO;
        float pMax = CHASE_SPEED * DEF_PUSH_MAX_RATIO;
        float pSpd = pMin + (pMax - pMin) * lineOffset;
        pushX = pSpd * sinf(lineAngle * PI / 180.0);
        pushY = pSpd * cosf(lineAngle * PI / 180.0);
    }

    if (ballAngle != -1)
    {
        // 追球：diff > 0 → 球在線右側；折算到 0~90° 讓側面偏差最大、正前/正後最小
        float diff = norm180(ballAngle - lineAngle);
        float angErr = fabsf(diff);
        if (angErr > 90)
            angErr = 180 - angErr;

        // 球的角度差要大於 5 度：移動
        if (angErr > 5)
        {
            float sDir = (diff > 0) ? 1.0 : -1.0;
            int slideAngle = (int)norm180(lineAngle + sDir * 90.0);
            slideX = CHASE_SPEED * sinf(slideAngle * PI / 180.0);
            slideY = CHASE_SPEED * cosf(slideAngle * PI / 180.0);
        }
        else
        {
            slideX = 0;
            slideY = 0;
        }
    }
    else if (goalValid)
    {
        // 無球：利用攻擊球門世界角度滑向場地中心
        float errC = 180 - fabsf((float)goalWorldAngle);
        if (errC > 5)
        {
            int slideAngle = (goalWorldAngle > 0) ? (int)norm180(lineAngle - 90) : (int)norm180(lineAngle + 90);
            slideX = CHASE_SPEED * sinf(slideAngle * PI / 180.0);
            slideY = CHASE_SPEED * cosf(slideAngle * PI / 180.0);
        }
    }

    // 離線越遠 (lineOffset 越大)，slide 權重越低，把馬力留給 push 校正
    float slideRatio = 1.0 - constrain(lineOffset, 0.0, 1.0);
    float spX = pushX + slideX * slideRatio; 
    float spY = pushY + slideY * slideRatio;

    speed = sqrtf(spX * spX + spY * spY);
    angle = atan2f(spX, spY) * 180.0 / PI;
}

// 已對準球並停住：原地等 2 秒確認球是否真的卡死。
// 期間球若移動就回 false，讓正常追球邏輯立刻接手；撐滿 2 秒回 true。
bool ballStuckAfterHold()
{
    int startAngle = ballAngle, startDist = ballDistance;
    unsigned long start = millis();
    while (millis() - start < 2000)
    {
        updateSensors();
        if (liftedRX || ballAngle == -1)
            return false;
        if (abs(ballAngle - startAngle) > 8 || abs(ballDistance - startDist) > 10)
            return false;
        move(0, 0, cmp_offset);
    }
    return true;
}

void handleDefenseMode()
{
    updateSensors();

    if (liftedRX)
    {
        stopAllMotors();
        return;
    }

    cmp_offset = 0;
    float finalAngle = 0, finalSpeed = 0;

    if (lastOwnGoalDist > 84 && abs(lastOwnGoalAngle) < 125)
    {
        // 己方球門偏側面 → 橫移滑回
        if (lastOwnGoalAngle > 0)
            finalAngle =  90;
        else
            finalAngle = -90;
        finalSpeed = 40;
    }
    else if (lastOwnGoalDist > 80 && lineDetected)
    {
        // 己方球門在前後 → 直接朝它走
        finalAngle = lastOwnGoalAngle;
        finalSpeed = 40;
    }
    else if (lineDetected)
    {
        // 在線上 → 追球或對齊中心
        onLineMove(finalAngle, finalSpeed);

        // 已對準球且幾乎不動：停 2 秒確認球真的卡死才出去清，避免誤觸發
        if (ballAngle != -1 && abs(ballAngle) < 80 && finalSpeed < 8 && ballStuckAfterHold())
        {
            handleActiveClear();
            return;
        }
    }
    else
    {
        // 離線 → 沿己方球門尋線
        finalSpeed = 40;
        if (lastOwnGoalDist < 50)
            finalAngle = norm180(lastOwnGoalAngle + 180);
        else
            finalAngle = lastOwnGoalAngle;
    }

    if (finalSpeed < 8)
        move(0, 0, cmp_offset);
    else
        move((int)finalAngle, (int)finalSpeed, cmp_offset);
}
