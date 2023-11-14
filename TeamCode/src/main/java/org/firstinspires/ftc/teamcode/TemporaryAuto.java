package org.firstinspires.ftc.teamcode;

import com.google.android.gms.fido.fido2.api.common.Algorithm;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous

public class TemporaryAuto extends LinearOpMode{

    DcMotor motorFrontLeft = null;
    DcMotor motorBackLeft = null;
    DcMotor motorFrontRight = null;
    DcMotor motorBackRight = null;
    double drive_power = 0.6;

    public void runOpMode() throws InterruptedException{
        motorFrontLeft = hardwareMap.dcMotor.get("leftFrontMotor");
        motorBackLeft = hardwareMap.dcMotor.get("leftBackMotor");
        motorFrontRight = hardwareMap.dcMotor.get("rightFrontMotor");
        motorBackRight = hardwareMap.dcMotor.get("rightBackMotor");
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);


        // zero power behavior
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        forward(drive_power);
        Thread.sleep(1000);
        strafeRight(drive_power);
        Thread.sleep(1000);
        backward(drive_power);
        Thread.sleep(1000);
        strafeLeft(drive_power);
        Thread.sleep(1000);
        turnRight(drive_power*3);
        stopDriving();
    }

    public void forward(double drive_power){
        motorFrontLeft.setPower(-drive_power);
        motorBackLeft.setPower(-drive_power);
        motorFrontRight.setPower(-drive_power);
        motorBackRight.setPower(-drive_power);
    }
    // drive backward
    public void backward(double drive_power){
        motorFrontLeft.setPower(drive_power);
        motorBackLeft.setPower(drive_power);
        motorFrontRight.setPower(drive_power);
        motorBackRight.setPower(drive_power);
    }
    // stop
    public void stopDriving(){
        forward(0);
    }
    // turn left
    public void turnLeft(double drive_power){
        motorFrontLeft.setPower(drive_power);
        motorBackLeft.setPower(drive_power);
        motorFrontRight.setPower(-drive_power);
        motorBackRight.setPower(-drive_power);
    }

    // turn right
    public void turnRight(double drive_power){
        motorFrontLeft.setPower(-drive_power);
        motorBackLeft.setPower(-drive_power);
        motorFrontRight.setPower(drive_power);
        motorBackRight.setPower(drive_power);
    }

    // strafe left
    public void strafeLeft(double drive_power){
        motorFrontLeft.setPower(drive_power);
        motorBackLeft.setPower(-drive_power);
        motorFrontRight.setPower(-drive_power);
        motorBackRight.setPower(drive_power);
    }
    // strafe right
    public void strafeRight(double drive_power){
        motorFrontLeft.setPower(-drive_power);
        motorBackLeft.setPower(drive_power);
        motorFrontRight.setPower(drive_power);
        motorBackRight.setPower(-drive_power);
    }
}