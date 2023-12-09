package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;

public class MyRunnable1 implements Runnable
{
    private Servo myServo;

    public MyRunnable1 (Servo argServo)
    {
        myServo = argServo;
    }

    public void run()
    {
        double currentPos = myServo.getPosition();
        myServo.setPosition(currentPos);
    }
}