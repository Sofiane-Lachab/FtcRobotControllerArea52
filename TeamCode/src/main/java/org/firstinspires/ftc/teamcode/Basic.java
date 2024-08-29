/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.MyRunnable1;
import org.firstinspires.ftc.teamcode.util.MyRunnable2;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
//hi this yo mom
@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")

public class Basic extends Base
{
    @Override
    public void runOpMode() throws InterruptedException {

        teleInitHardware();

        int leftEncoderStart = leftSlide.getCurrentPosition();
        int rightEncoderStart = rightSlide.getCurrentPosition();
        int leftTruss = leftSlide.getCurrentPosition() + 1900;
        int rightTruss = rightSlide.getCurrentPosition() + 1900;
        int leftLow = leftSlide.getCurrentPosition() + 600;
        int rightLow = rightSlide.getCurrentPosition() + 600;
        int leftLower = leftSlide.getCurrentPosition() + 300;
        int rightLower = rightSlide.getCurrentPosition() + 300;
        int leftLowest = leftSlide.getCurrentPosition() + 50;
        int rightLowest = rightSlide.getCurrentPosition() + 50;

        double leftInitPos = leftBucketServo.getPosition();
        double rightInitPos = rightBucketServo.getPosition();

        telemetry.addData("Initialized: ", "Waiting for Start");
        telemetry.update();

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Left Encoder Position: ", leftSlide.getCurrentPosition());
            telemetry.addData("Right Encoder Position: ", rightSlide.getCurrentPosition());
            telemetry.addData("Left Start Position: ", leftInitPos);
            telemetry.addData("Right Start Position: ", rightInitPos);
            telemetry.addData("Left Servo Position: ", leftBucketServo.getPosition());
            telemetry.addData("Right Servo Position: ", rightBucketServo.getPosition());

            telemetry.update();

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            if(gamepad1.left_bumper)
            {
                yaw = -gamepad1.right_stick_x;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }


            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Slows the wheels down by decreasing the variables for the power to the motors
//            if (gamepad1.left_trigger > 0) {
//                leftFrontPower *= 0.2;
//                rightFrontPower *= 0.2;
//                leftBackPower *= 0.2;
//                rightFrontPower *= 0.2;
//            }

            if (gamepad1.left_bumper)
            {
                leftFrontPower *= -1;
                rightFrontPower *= -1;
                leftBackPower *= -1;
                rightBackPower *= -1;
            }

//            if(gamepad1.left_bumper)
//            {
//                leftFrontPower *= 0.4;
//                rightFrontPower *= 0.4;
//                leftBackPower *= 0.4;
//                rightFrontPower *= 0.4;
//            }

//            if(gamepad1.right_trigger > 0)
//            {
//                leftFrontPower *= 0.6;
//                rightFrontPower *= 0.6;
//                leftBackPower *= 0.6;
//                rightFrontPower *= 0.6;
//            }

            if (gamepad1.right_trigger > 0.5) {
                leftFrontPower *= 0.7;
                rightFrontPower *= 0.7;
                leftBackPower *= 0.7;
                rightFrontPower *= 0.7;
            }


            boolean lastY2 = false;
            boolean lastA2 = false;
            boolean lastX2 = false;
            boolean lastB2 = false;
            boolean lastLeftStick2 = false;
            boolean currentY2 = gamepad2.y;
            boolean currentA2 = gamepad2.a;
            boolean currentX2 = gamepad2.x;
            boolean currentB2 = gamepad2.b;
            boolean currentLeftStick2 = gamepad2.left_stick_button;
            if (currentY2 && !lastY2)
            {
                hangSlides(leftSlide, rightSlide, leftTruss, rightTruss);
            }
            if(currentX2 && !lastX2)
            {
                hangSlides(leftSlide, rightSlide, leftLow, rightLow);
            }
            if (currentA2 && !lastA2)
            {
                lowerSlides(leftSlide, rightSlide, leftEncoderStart + 13, rightEncoderStart + 6);
            }
            if(currentB2 && !lastB2)
            {
                hangSlides(leftSlide, rightSlide, leftLower, rightLower);
            }
            if(currentLeftStick2 && !lastLeftStick2)
            {
                hangSlides(leftSlide, rightSlide, leftLowest, rightLowest);
            }

            if(leftSlide.getCurrentPosition() < 15)
            {
                leftSlide.setPower(0);
            }
            if(rightSlide.getCurrentPosition() < 15)
            {
                rightSlide.setPower(0);
            }
            lastY2 = currentY2;
            lastA2 = currentA2;
            lastX2 = currentX2;
            lastB2 = currentB2;

//            while(leftSlide.getCurrentPosition() < 10 && rightSlide.getCurrentPosition() < 10
//                    && !leftSlide.isBusy() && !rightSlide.isBusy())
//            {
//                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }


//            if(gamepad2.left_trigger > 0)
//            {
//                leftFrontPower *= 0;
//                rightFrontPower *= 0;
//                leftBackPower *= 0;
//                rightFrontPower *= 0;
//            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.update();


//            // Uses player 2's thumbstick to bring the linear slides up and down
//            double liftPower = -gamepad2.left_stick_y;
//
//            // Contains the requests from the triggers to raise or lower the slides
//            leftSlide.setPower(liftPower);
//            rightSlide.setPower(liftPower);

            wheelIntakeMotor.setPower(0);
            beltMotor.setPower(0);

            // Extra convoluted steps to make bucket wheel work
            boolean pressLeftBumper = false;
            boolean pressRight = false;
            if(gamepad2.left_bumper || gamepad1.a)
            {
                pressLeftBumper = true;
            }
            if(gamepad2.dpad_right)
            {
                pressRight = true;
            }
            if(!pressLeftBumper && !pressRight)
            {
                wheelServo.setPower(0);
            }
            if(pressRight)
            {
                wheelServo.setPower(-1);
            }
            if(pressLeftBumper)
            {
                wheelServo.setPower(1);
                wheelIntakeMotor.setPower(1);
                beltMotor.setPower(1);
            }

            if(gamepad1.b)
            {
                wheelIntakeMotor.setPower(-1);
                beltMotor.setPower(-1);
            }


            if(gamepad2.dpad_up)
            {
                leftBucketServo.setPosition(0.40);
            }
            if(gamepad2.dpad_up)
            {
                rightBucketServo.setPosition(0.40);
            }
            if(gamepad2.dpad_left)
            {
                leftBucketServo.setPosition(0.10);
            }
            if(gamepad2.dpad_left)
            {
                rightBucketServo.setPosition(0.10);
            }
            if(gamepad2.dpad_down)
            {
                leftBucketServo.setPosition(0.0085);
            }
            if(gamepad2.dpad_down)
            {
                rightBucketServo.setPosition(0.0085);
            }

            if(gamepad2.right_bumper)
            {
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setPower(-gamepad2.right_stick_y);
                rightSlide.setPower(-gamepad2.right_stick_y);
            }



            if(gamepad1.right_stick_button)
            {
                planeServo.setPosition(0);
            }
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
}
