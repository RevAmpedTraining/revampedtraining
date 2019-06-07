package org.firstinspires.ftc.teamcode.Test;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.revAmped.sensors.RevColorDistanceSensor;
import com.revAmped.sensors.VuMarkSensing;
import com.revAmped.util.JewelDetector;
import com.revAmped.util.VuforiaLocalizerImplSubClass;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.*;
import org.opencv.imgcodecs.*;  //Use Imgcodecs.imread, Imgcodecs.imwrite, etc.

import java.util.ArrayList;
import java.util.List;

//Vuforia test
//=======
//Vuforia test
//=======


/**
 * RobotRevAmped Tester
 */
@TeleOp(name="HDCam", group="Test")
public class HDCamTest
        extends LinearOpMode {


    //vuforia test variables
    public static final String TAG = "HDCamSample";

    /**
     * {@link } is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */

    //define the constant
    public int CAP_HEIGHT = 720;
    public int CAP_WIDTH = 1280;

    private Mat hiarchy  = new Mat();

    @Override
    public void runOpMode()
            throws InterruptedException {

        Boolean bloaded = false;
        //check opencv
        if (OpenCVLoader.initDebug()) {
            telemetry.addData(">", "OpenCV lib loaded successfully.");
            bloaded = true;
        }
        telemetry.update();
        //============= refactor camera open to a private function =================
        VideoCapture camera  = new VideoCapture(0);  // must check what is on the phone. on laptop it is 1 for HDCam.
        if (camera .isOpened())
        {
            telemetry.addData("Camera0", "successfully opened");

        }
        else
        {
            telemetry.addData("Camera0", "failed to open");
            telemetry.update();
            //retry
            camera  = new VideoCapture(1);
            //need to check status.
            if (camera.isOpened())
            {
                telemetry.addData("Camera1", "successfully opened");

            }
            else
            {
                telemetry.addData("Camera1", "failed to open");
                telemetry.update();
                //retry
                camera  = new VideoCapture(2);
                //need to check status.
                if (camera.isOpened()) {
                    telemetry.addData("Camera2", "successfully opened");

                }
                else
                {
                    telemetry.addData("Camera2", "failed to open");
                }
            }
        }

        telemetry.update();

        if(camera.isOpened()){ // set the video to be HD
            camera.set(Videoio.CV_CAP_PROP_FRAME_WIDTH, CAP_WIDTH);
            camera.set(Videoio.CV_CAP_PROP_FRAME_HEIGHT, CAP_HEIGHT);
        }

        //========================================================================

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        //declare local variables
        Mat frame = new Mat();
        Mat mask = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        List<Mat> channels = new ArrayList<>();

        while (opModeIsActive() && bloaded) {
            telemetry.addData("CaptureVideo", "");

            if (camera.read(frame)) {
                if (!frame.empty()) {
                    //Write the image to disk.  Not ncessary, but useful for debugging.  should comment it out when release to phone.
                    //Imgcodecs.imwrite("C:\\temp\\videoframe.jpg", frame);
                    //Clone the image.
                    Mat imgGray = new Mat();
                    //Convert to GrayScale Image
                    Imgproc.cvtColor(frame, imgGray, Imgproc.COLOR_BGR2GRAY);

                    //Use GaussianBlue for better edge detection.
                    Imgproc.GaussianBlur(imgGray, imgGray, new Size(5, 5), 0);

                    mask = new Mat();
                    //do threshold. using OSTU
                    Imgproc.threshold(imgGray, mask, 0, 255, Imgproc.THRESH_OTSU);
                    //remove small dots.
                    Mat maskOpenFill = new Mat();
                    Mat maskCloseFill = new Mat();
                    Mat kernel = new Mat(new Size(5, 5), CvType.CV_8UC1, new Scalar(255));
                    //# remove small dots and fill the gaps.
                    Imgproc.morphologyEx(mask, maskOpenFill, Imgproc.MORPH_OPEN,kernel );
                    Imgproc.morphologyEx(maskOpenFill, maskCloseFill, Imgproc.MORPH_CLOSE,kernel );

                    //find the contour
                    Imgproc.findContours(maskCloseFill, contours, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
                    //check each coutours:  if it matches the criteria,  contour area / circle > RATIO,  MIN < radius < MAX,  X> X_MIN,  Y>Y_MIN
                    //RobotLog.vv("BLUEContours", "%d", contours.size());
                    for (MatOfPoint c : contours) {

                    }
                }
            }

        }


    }



}
