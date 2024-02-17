package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

@Autonomous

public class NewBlueShort extends LinearOpMode{

    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;
    Servo leftArmServo = null;
    Servo rightArmServo = null;
    Servo clawServo = null;
    private HuskyLens huskyLens = null;
    double drive_power = 0.4;

    public void runOpMode() throws InterruptedException{
//        leftFrontDrive = hardwareMap.dcMotor.get("leftFrontMotor");
//        leftBackDrive = hardwareMap.dcMotor.get("leftBackMotor");
//        rightFrontDrive = hardwareMap.dcMotor.get("rightFrontMotor");
//        rightBackDrive = hardwareMap.dcMotor.get("rightBackMotor");
//        leftArmServo = hardwareMap.servo.get("leftArmServo");
//        rightArmServo = hardwareMap.servo.get("rightArmServo");
//        clawServo = hardwareMap.servo.get("clawServo");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");


//        // Reverse the right side motors
//        // Reverse left motors if you are using NeveRests
//        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftArmServo.setDirection(Servo.Direction.REVERSE);
//        clawServo.setDirection(Servo.Direction.REVERSE);

//
//
//        // zero power behavior
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        telemetry.update();

        HuskyLens.Block[] blocks = huskyLens.blocks();
        HuskyLens.Block trueBlock = blocks[0];

        for (int i = 0; i < blocks.length; i++) {
            int currBlockArea = blocks[i].height * blocks[i].width;
            int trueBlockArea = trueBlock.height * trueBlock.width;
            if(currBlockArea > trueBlockArea)
            {
                trueBlock = blocks[i];
            }
        }

        String trueBlockPos;

        if(trueBlock.x > 200)
        {
            trueBlockPos = "LEFT_SPIKE";
        }
        else if(trueBlock.x > 100)
        {
            trueBlockPos = "MIDDLE_SPIKE";
        }
        else
        {
            trueBlockPos = "RIGHT_SPIKE";
        }


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: 70, heading: 270 degrees
        Pose2d startPose = new Pose2d(10, 70, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        switch(trueBlockPos)
        {
            case "LEFT_SPIKE":
                Trajectory toLeftSpike = drive.trajectoryBuilder(startPose)
                        .lineToSplineHeading(new Pose2d(20.5, 32, 0))
                        .build();
                Trajectory toLeftBackdrop = drive.trajectoryBuilder(toLeftSpike.end(), true)
                        .back(6)
                        .splineTo(new Vector2d(52, 43), Math.toRadians(180))
                        .build();
                Trajectory toLeftPark = drive.trajectoryBuilder(toLeftBackdrop.end(), true)
                        .splineTo(new Vector2d(54, 60), Math.toRadians(180))
                        .build();

                waitForStart();
                drive.followTrajectory(toLeftSpike);
                drive.followTrajectory(toLeftBackdrop);
                drive.followTrajectory(toLeftPark);

                break;


            case "MIDDLE_SPIKE":
                Trajectory toMiddleSpike = drive.trajectoryBuilder(startPose)
                        .forward(48)
                        .build();

                Trajectory toMiddleBackdrop = drive.trajectoryBuilder(toMiddleSpike.end(), true)
                        .back(6)
                        .splineTo(new Vector2d(52, 35), Math.toRadians(0))
                        .build();

                Trajectory toMiddlePark = drive.trajectoryBuilder(toMiddleBackdrop.end(), true)
                        .splineTo(new Vector2d(54, 60), Math.toRadians(180))
                        .build();

                waitForStart();
                drive.followTrajectory(toMiddleSpike);
                drive.followTrajectory(toMiddleBackdrop);
                drive.followTrajectory(toMiddlePark);

                break;

            case "RIGHT_SPIKE":
                Trajectory toRightSpike = drive.trajectoryBuilder(startPose)
                        .lineToSplineHeading(new Pose2d(7, 32, Math.toRadians(180)))
                        .build();
                Trajectory toRightBackdrop = drive.trajectoryBuilder(toRightSpike.end(), true)
                        .back(6)
                        .splineTo(new Vector2d(52, 29), Math.toRadians(180))
                        .build();
                Trajectory toRightPark = drive.trajectoryBuilder(toRightBackdrop.end(), true)
                        .splineTo(new Vector2d(54, 60), Math.toRadians(180))
                        .build();

                waitForStart();
                drive.followTrajectory(toRightSpike);
                drive.followTrajectory(toRightBackdrop);
                drive.followTrajectory(toRightPark);

                break;
        }



    }

    public void raiseArms()
    {
        leftArmServo.setPosition(0.3);
        rightArmServo.setPosition(0.3);
    }

    public void lowerArms()
    {
        leftArmServo.setPosition(0);
        rightArmServo.setPosition(0);
    }
}