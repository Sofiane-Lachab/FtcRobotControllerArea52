package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;

public class MyRunnable2 implements Runnable
{
    private Servo myServo;
    private double currentPos;

    public MyRunnable2 (Servo argServo)
    {
        myServo = argServo;
    }

    public void run()
    {
        myServo.setPosition(0);
    }
}