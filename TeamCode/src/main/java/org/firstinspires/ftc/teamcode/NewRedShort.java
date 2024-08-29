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

@Autonomous

public class NewRedShort extends Base
{
    public void runOpMode() throws InterruptedException
    {
        autoInitHardware();

        int leftEncoderStart = leftSlide.getCurrentPosition();
        int rightEncoderStart = rightSlide.getCurrentPosition();
        int leftLow = leftSlide.getCurrentPosition() + 900;
        int rightLow = rightSlide.getCurrentPosition() + 900;
        int leftLower = leftSlide.getCurrentPosition() + 80;
        int rightLower = rightSlide.getCurrentPosition() + 80;
        telemetry.addData("Initialized: ", "Waiting for Start");
        telemetry.update();

        waitForStart();

        initTfod(); // initialize TensorFlow program

        String Position = null;
        int counter = 0;
        while(Position == null && counter < 150000)
        {
            Position = findPosition();
            if(Position == null) {
                telemetry.addData("No Position Found", "");
                telemetry.update();
            }
            else
            {
                telemetry.addData("Position Found: ", Position);
                telemetry.update();
            }
            counter++;
        }
        if(counter >= 150000 && Position == null)
        {
            Position = "MIDDLE";
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 11.5, y: -70, heading: 90 degrees
        Pose2d startPose = new Pose2d(11.5, -70, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        switch(Position)
        {
            case "LEFT":
                TrajectorySequence leftSeq = drive.trajectorySequenceBuilder(startPose)     // for left randomization
                        .lineToSplineHeading(new Pose2d(8, -42, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {dump();})    // spits out pixel on spike mark
                        .waitSeconds(2.5)
                        .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {stopDump();})    // stop outtake
                        .lineToSplineHeading(new Pose2d(53,-42, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(-2, () -> {leftUp(leftBucketServo); //raises bucket
                            rightUp(rightBucketServo);
                            hangSlides(leftSlide, rightSlide, leftLower, rightLower);})  // raises slides
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {bucketOut();})
                        .waitSeconds(3)
                        .back(6)
                        .strafeRight(27)
                        .turn(Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {leftDown(leftBucketServo);  // drops bucket
                            rightDown(rightBucketServo);
                            lowerSlides(leftSlide, rightSlide, leftEncoderStart, rightEncoderStart);})  // drops slides
                        .waitSeconds(1)
                        .build();

//                waitForStart();
                if(!isStopRequested()) {
                    drive.followTrajectorySequence(leftSeq);
                }
                break;


            case "MIDDLE":
                TrajectorySequence middleSeq = drive.trajectorySequenceBuilder(startPose)     // for left randomization
                        .back(30)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {dump();})    // spits out pixel on spike mark
                        .waitSeconds(2.5)
                        .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {stopDump();})    // stop outtake
                        .lineToSplineHeading(new Pose2d(52,-46, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(-2, () -> {leftUp(leftBucketServo); //raises bucket
                            rightUp(rightBucketServo);
                            hangSlides(leftSlide, rightSlide, leftLower, rightLower);})  // raises slides
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {bucketOut();})
                        .waitSeconds(3)
                        .back(6)
                        .strafeRight(24)
                        .turn(Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {leftDown(leftBucketServo);  // drops bucket
                            rightDown(rightBucketServo);
                            lowerSlides(leftSlide, rightSlide, leftEncoderStart, rightEncoderStart);})  // drops slides
                        .waitSeconds(1)
                        .build();

//                waitForStart();
                if(!isStopRequested()){
                    drive.followTrajectorySequence(middleSeq);
                }
                break;

            case "RIGHT":
                TrajectorySequence rightSeq = drive.trajectorySequenceBuilder(startPose)     // for left randomization
                        .lineToSplineHeading(new Pose2d(31, -44, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {dump();})    // spits out pixel on spike mark
                        .waitSeconds(2)
                        .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {stopDump();})    // stop outtake
                        .forward(3)
                        .strafeRight(6)
                        .lineToSplineHeading(new Pose2d(52, -51, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(-2, () -> {leftUp(leftBucketServo); //raises bucket
                            rightUp(rightBucketServo);
                            hangSlides(leftSlide, rightSlide, leftLower, rightLower);})  // raises slides
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {bucketOut();})
                        .waitSeconds(3)
                        .back(6)
                        .strafeRight(21)
                        .turn(Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {leftDown(leftBucketServo);  // drops bucket
                            rightDown(rightBucketServo);
                            lowerSlides(leftSlide, rightSlide, leftEncoderStart, rightEncoderStart);})  // drops slides
                        .waitSeconds(1)
                        .build();

//                waitForStart();
                if(!isStopRequested()){
                    drive.followTrajectorySequence(rightSeq);
                }
                break;
        }



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
    public void dump()
    {
        wheelIntakeMotor.setPower(-1);
        beltMotor.setPower(-1);
    }
    public void stopDump()
    {
        wheelIntakeMotor.setPower(0);
        beltMotor.setPower(0);
    }
    public void bucketOut()
    {
        wheelServo.setPower(-1);
    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.6f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private String findPosition()
    {
        String Position = null; // defaults to middle because it's the most difficult to recgonize (farthest away)

        List<Recognition> currentRecognitions = tfod.getRecognitions(); // list of all TensorFlow recognitions (should just be one)
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {   // traverses through all of the recognitions
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;   // x-coordinate of bounding box center
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;  // y-coordinate of bounding box center

                if(x < 280) {   // checks if in left third of the frame
                    Position = "LEFT";}
                else if(x < 400) {  // checks if in middle third of the frame
                    Position = "MIDDLE";}
                else {  // checks if in right third of the frame
                    Position = "RIGHT";}

            telemetry.addData("Position: ", Position);  // displays on driver hub
        }   // end for() loop

        return Position;    // returns the object's position
    }   // end method telemetryTfod()
}
