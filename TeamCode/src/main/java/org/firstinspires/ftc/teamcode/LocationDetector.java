package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous

public class LocationDetector extends LinearOpMode
{
    Servo leftArmServo = null;
    Servo rightArmServo = null;
    private static OpenCvCamera webcam;
    private PropLocationPipeline pipeline;

    public void runOpMode() throws InterruptedException
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new PropLocationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addLine("Camera no work");
                telemetry.update();
            }
        });


        String position = PropLocationPipeline.getLocation();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
        telemetry.addLine("Prop Location: " + position);
        telemetry.update();


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


    static class PropLocationPipeline extends OpenCvPipeline
    {
        boolean viewportPaused = false;
        private static String location; // output
        public Scalar lower = new Scalar(155, 75, 130); // HSV threshold bounds
        public Scalar upper = new Scalar(170, 130, 255);

        private Mat hsvMat = new Mat(); // converted image
        private Mat binaryMat = new Mat(); // imamge analyzed after thresholding
        private Mat maskedInputMat = new Mat();

        // Rectangle regions to be scanned
        private Point topLeft1 = new Point(0, 0), bottomRight1 = new Point(160, 240);
        private Point topLeft2 = new Point(160, 0), bottomRight2 = new Point(320, 240);


        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            // Convert from BGR to HSV
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, lower, upper, binaryMat);

            // Scan both rectangle regions, keeping track of how many
            // pixels meet the threshold value, indicated by the color white
            // in the binary image
            double leftCount = 0, rightCount = 0;
            // process the pixel value for each rectangle  (255 = W, 0 = B)
            for (int i = (int) topLeft1.x; i <= bottomRight1.x; i++) {
                for (int j = (int) topLeft1.y; j <= bottomRight1.y; j++) {
                    if (binaryMat.get(i, j)[0] == 255) {
                        leftCount++;
                    }
                }
            }

            for (int i = (int) topLeft2.x; i <= bottomRight2.x; i++) {
                for (int j = (int) topLeft2.y; j <= bottomRight2.y; j++) {
                    if (binaryMat.get(i, j)[0] == 255) {
                        rightCount++;
                    }
                }
            }

            // Determine object location
            if (leftCount > rightCount)
            {
                location = "Left";
            }
            else
            {
                location = "Right";
            }

            return input;
        }

        public static String getLocation()
        {
            return location;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}