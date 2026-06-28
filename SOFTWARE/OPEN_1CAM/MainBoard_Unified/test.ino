void doTest()
{
    updateSensors();
    dribbler(40);
    while (abs(ballAngle > 80))
    {
        updateSensors();

        move(180, 30);
    }
    while (abs(ballAngle > 10))
    {
        updateSensors();

        if (ballAngle > 10)
        {
            move(90, 30);
        }
        else if (ballAngle < 10)
        {
            move(-90, 30);
        }
    }
    while (ballDistance > 20)
    {
        updateSensors();

        move(0, 30);
    }
    while (abs(ballAngle > 10))
    {
        updateSensors();

        if (ballAngle > 10)
        {
            move(90, 30);
        }
        else if (ballAngle < 10)
        {
            move(-90, 30);
        }
    }
    moveForMs(0, 30, 0, 2000);
    while (abs(gYGOALAngle) > 10 || gYGOALAngle == -1)
    {
        updateSensors();
        if (gYGOALAngle > 10)
        {
            move(90, 30);
        }
        else if (gYGOALAngle < 10)
        {
            move(-90, 30);
        }
    }
    kickBall();
}