package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class BlueShort extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;
    private DcMotor beltMotor = null;
    private DcMotor wheelIntakeMotor = null;
    private Servo leftBucketServo = null;
    private Servo rightBucketServo = null;
    private CRServo wheelServo = null;
    private Servo planeServo = null;
    double drive_power = 0.4;

    public void runOpMode() throws InterruptedException{
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackMotor");
        leftSlide = hardwareMap.get(DcMotor.class, "leftLinearSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightLinearSlide");
        leftBucketServo = hardwareMap.servo.get("leftBucketServo");
        rightBucketServo = hardwareMap.servo.get("rightBucketServo");
        wheelServo = hardwareMap.get(CRServo.class, "wheelServo");
        planeServo = hardwareMap.servo.get("planeServo");
        wheelIntakeMotor = hardwareMap.get(DcMotor.class, "wheelIntakeMotor");
        beltMotor = hardwareMap.get(DcMotor.class, "beltMotor");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);
        leftBucketServo.setDirection(Servo.Direction.REVERSE);
        wheelIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        beltMotor.setDirection(DcMotor.Direction.FORWARD);
        wheelServo.setDirection(CRServo.Direction.REVERSE);

        // zero power behavior
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int leftEncoderStart = leftSlide.getCurrentPosition();
        int rightEncoderStart = rightSlide.getCurrentPosition();
        int leftLow = leftSlide.getCurrentPosition() + 900;
        int rightLow = rightSlide.getCurrentPosition() + 900;
        int leftLower = leftSlide.getCurrentPosition() + 500;
        int rightLower = rightSlide.getCurrentPosition() + 500;

        waitForStart();
        forward(drive_power);
        Thread.sleep(1500);
        turnLeft(drive_power);
        Thread.sleep(150);
        leftUp(leftBucketServo);
        rightUp(rightBucketServo);
        hangSlides(leftSlide, rightSlide, leftLower, rightLower);
        Thread.sleep(1000);
        forward(drive_power);
        Thread.sleep(3000);
        wheelServo.setPower(-1);
        Thread.sleep(2000);
        backward(drive_power);
        Thread.sleep(500);
        strafeLeft(drive_power);
        Thread.sleep(1500);
        leftDown(leftBucketServo);
        rightDown(rightBucketServo);
        lowerSlides(leftSlide, rightSlide, leftEncoderStart, rightEncoderStart);
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

    public void hangSlides(DcMotor leftHook, DcMotor rightHook, int leftHeight, int rightHeight)
    {
        leftHook.setTargetPosition(leftHeight);
        leftHook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftHook.setPower(1);
        rightHook.setTargetPosition(rightHeight);
        rightHook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightHook.setPower(1);
    }
    public void lowerSlides(DcMotor leftHook, DcMotor rightHook, int leftHeight, int rightHeight)
    {
        leftHook.setTargetPosition(leftHeight);
        leftHook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftHook.setPower(1);
        rightHook.setTargetPosition(rightHeight);
        rightHook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightHook.setPower(1);
    }

    public void leftUp(Servo leftArm)
    {
        leftArm.setPosition(0.4);
    }
    public void rightUp(Servo rightArm)
    {
        rightArm.setPosition(0.4);
    }
    public void leftDown(Servo leftArm)
    {
        leftArm.setPosition(0);
    }
    public void rightDown(Servo rightArm)
    {
        rightArm.setPosition(0);
    }
}