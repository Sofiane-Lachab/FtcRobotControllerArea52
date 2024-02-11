package org.firstinspires.ftc.teamcode;

import com.google.android.gms.fido.fido2.api.common.Algorithm;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.MyRunnable1;
import org.firstinspires.ftc.teamcode.util.MyRunnable2;
import org.firstinspires.ftc.teamcode.util.MyRunnable3;
import org.firstinspires.ftc.teamcode.util.MyRunnable4;

@Autonomous

public class BlueLong extends LinearOpMode{

    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;
    Servo leftArmServo = null;
    Servo rightArmServo = null;
    Servo clawServo = null;
    double drive_power = 0.4;

    public void runOpMode() throws InterruptedException{
        leftFrontDrive = hardwareMap.dcMotor.get("leftFrontMotor");
        leftBackDrive = hardwareMap.dcMotor.get("leftBackMotor");
        rightFrontDrive = hardwareMap.dcMotor.get("rightFrontMotor");
        rightBackDrive = hardwareMap.dcMotor.get("rightBackMotor");
        leftArmServo = hardwareMap.servo.get("leftArmServo");
        rightArmServo = hardwareMap.servo.get("rightArmServo");
        clawServo = hardwareMap.servo.get("clawServo");
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftArmServo.setDirection(Servo.Direction.REVERSE);
        clawServo.setDirection(Servo.Direction.REVERSE);


        // zero power behavior
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Runnables and Threads
        MyRunnable3 upServos = new MyRunnable3(leftArmServo, rightArmServo);
        MyRunnable4 downServos = new MyRunnable4(leftArmServo, rightArmServo);
        Thread servosUp = new Thread(upServos);
        Thread servosDown = new Thread(downServos);


        waitForStart();
            forward(drive_power);

        stopDriving();
    }

    public void forward(double drive_power){
        leftFrontDrive.setPower(drive_power);
        leftBackDrive.setPower(drive_power);
        rightFrontDrive.setPower(drive_power);
        rightBackDrive.setPower(drive_power);
    }
    // drive backward
    public void backward(double drive_power){
        leftFrontDrive.setPower(-drive_power);
        leftBackDrive.setPower(-drive_power);
        rightFrontDrive.setPower(-drive_power);
        rightBackDrive.setPower(-drive_power);
    }
    // stop
    public void stopDriving(){
        forward(0);
    }
    // turn left
    public void turnLeft(double drive_power){
        leftFrontDrive.setPower(-drive_power);
        leftBackDrive.setPower(-drive_power);
        rightFrontDrive.setPower(drive_power);
        rightBackDrive.setPower(drive_power);
    }

    // turn right
    public void turnRight(double drive_power){
        leftBackDrive.setPower(drive_power);
        leftBackDrive.setPower(drive_power);
        rightFrontDrive.setPower(-drive_power);
        rightBackDrive.setPower(-drive_power);
    }

    // strafe left
    public void strafeLeft(double drive_power){
        leftFrontDrive.setPower(-drive_power);
        leftBackDrive.setPower(drive_power);
        rightFrontDrive.setPower(drive_power);
        rightBackDrive.setPower(-drive_power);
    }
    // strafe right
    public void strafeRight(double drive_power){
        leftFrontDrive.setPower(drive_power);
        leftBackDrive.setPower(-drive_power);
        rightFrontDrive.setPower(-drive_power);
        rightBackDrive.setPower(drive_power);
    }
}