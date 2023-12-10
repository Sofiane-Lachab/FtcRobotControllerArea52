package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;

public class MyRunnable3 implements Runnable
{
    private Servo myServo1;
    private Servo myServo2;

    public MyRunnable3 (Servo servo1, Servo servo2)
    {
        myServo1 = servo1;
        myServo2 = servo2;
    }

    public void run()
    {
        boolean valid = true;
        if (valid)
        {
            myServo1.setPosition(0.3);
        }
        if(valid)
        {
            myServo2.setPosition(0.3);
        }

    }
}