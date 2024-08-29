package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
public abstract class Base extends LinearOpMode {
    protected ElapsedTime runtime = new ElapsedTime();
    protected DcMotor leftFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor rightBackDrive = null;
    protected DcMotor leftSlide = null;
    protected DcMotor rightSlide = null;
    protected DcMotor beltMotor = null;
    protected DcMotor wheelIntakeMotor = null;
    protected Servo leftBucketServo = null;
    protected Servo rightBucketServo = null;
    protected CRServo wheelServo = null;
    protected Servo planeServo = null;

    protected static final String TFOD_MODEL_ASSET = "model_20240229_183617.tflite";
    protected static final String[] LABELS = {"Prop"};
    protected TfodProcessor tfod;
    protected VisionPortal visionPortal;

    public void autoInitHardware() throws InterruptedException {
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
    }
    public void teleInitHardware() throws InterruptedException {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
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

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);
        leftBucketServo.setDirection(Servo.Direction.REVERSE);
        wheelIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        beltMotor.setDirection(DcMotor.Direction.FORWARD);
        wheelServo.setDirection(CRServo.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
