package org.firstinspires.ftc.teamcode.Test;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.revAmped.sensors.VuMarkSensing;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.revAmped.sensors.RevColorDistanceSensor;
import com.revAmped.util.JewelDetector;
import com.revAmped.util.VuforiaLocalizerImplSubClass;

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
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.android.OpenCVLoader;

//Vuforia test
//=======
//Vuforia test
//=======


/**
 * RobotRevAmped Tester
 */
//@Disabled
@TeleOp(name="VuforiaId", group="Test")
public class VuforiaTest
        extends LinearOpMode {


    //vuforia test variables
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    /**
     * {@link } is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */

    //define the constant
    public int LEFT_JEWEL_BLUE = 0;
    public int LEFT_JEWEL_RED = 1;
    public int GRYTH_LEFT = 0;
    public int GRYTH_CENTER = 1;
    public int GRYTH_RIGHT = 2;

    //initialize the jewel color and gryth key.
    public int jewelColor = 0;
    public int grythKey = 0;


    //VuforiaLocalizer vuforia;

    @Override
    public void runOpMode()
            throws InterruptedException {

        //vuforiaTest();


        VuMarkSensing vuSensor = new VuMarkSensing(this.hardwareMap);
        Boolean bInit = vuSensor.initialize();
        if (bInit) {
            telemetry.addData("Vuforia", "Initalized");
        }
        else
        {
            telemetry.addData("Vuforia", "Failed to Init");
        }

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (bInit) { // only call OpenCV if bInit is true. otherwise, it caused robot to fail, unless try catch it.
                Bitmap bitmap = vuSensor.vuforia.getBitmap();
                Mat origimg = new Mat(bitmap.getHeight(), bitmap.getWidth(), CvType.CV_8UC4);   // 8 bits  3 channel
                Utils.bitmapToMat(bitmap, origimg);

                JewelDetector jd = new JewelDetector();
                RevColorDistanceSensor.COLORTYPE jewel = jd.GetJewelColor(origimg);
                telemetry.addData("REVAMPED_JEWEL", "%s", jewel);
                jd.processFrame(origimg, true);
                JewelDetector.JewelOrder order = jd.getCurrentOrder();
                telemetry.addData("DOGCV_JEWEL", "%s", order);

                telemetry.addData("Jewel", "%s", jewel);
                telemetry.addData("DOGECV", "%s", order);
                telemetry.update();
                idle();
                origimg.release();
            }
        }
    }

    private void vuforiaTest() {
                        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "ATZtEnj/////AAAAGT+CwB7OMkQJi55eZzs9StIAUPU0rdfyi9dSNXcaDLjU1P5Jl9flS5IgZCPhTFVEGGKIXukueuwprx3aNQeXujJjeXhMjEB3pcLLivaZY/EaRvZhuJPVPcAcF3RX7AVO4gPPpgvd4BVnSMlGqtoCUzAcOatW8xFTSPH5Z/IaAgw1OPQ9o6hv8DoxKcR8cVwO2NubyFRaGrl+8bDUCcuxt6PMcXF4wQ27fMRmsmc7LSRBdSOthfyn2O90chXpGrg42JqCPKKyctryWj30NEtTKnlIsZPkA3T5C4Lx/rxaFkg8fZ0MiGvh37WBuPVVh+njEHR0xJD7H0x4bZNBffsvK3aLmfDoVHVlLoBZ8DdhPcGg";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        //VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaLocalizerImplSubClass vuforia = new VuforiaLocalizerImplSubClass(parameters);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        relicTrackables.activate();

        //check opencv
        if (OpenCVLoader.initDebug()) {
            telemetry.addData(">", "OpenCV lib loaded successfully.");
        }

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        RobotLog.vv("Tester", "PressStart");
        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */

        long start_time = System.currentTimeMillis();
        RelicRecoveryVuMark vuMark;
        while (opModeIsActive() && (System.currentTimeMillis() - start_time) < 25000) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);
                RobotLog.vv(TAG, "VuMark is : %s", vuMark);
                switch (vuMark) {
                    case CENTER:
                        grythKey = GRYTH_CENTER;
                        break;
                    case LEFT:
                        grythKey = GRYTH_LEFT;
                        break;
                    case RIGHT:
                        grythKey = GRYTH_RIGHT;
                    default:
                        break;
                }


                Bitmap bitmap = vuforia.getBitmap();
                Mat origimg = new Mat(bitmap.getHeight(), bitmap.getWidth(), CvType.CV_8UC4);   // 8 bits  3 channel
                Utils.bitmapToMat(vuforia.bm, origimg);

                JewelDetector jd = new JewelDetector();
                RevColorDistanceSensor.COLORTYPE jewel = jd.GetJewelColor(origimg);
                telemetry.addData("REVAMPED_JEWEL", "%s", jewel);
                jd.processFrame(origimg, true);
                JewelDetector.JewelOrder order = jd.getCurrentOrder();
                telemetry.addData("DOGCV_JEWEL", "%s", order);

                RobotLog.vv("JEWEL", "%s ", jewelColor);

                // for fun,  detect the hexgons.
                //jd.getHexgonPattern(origimg);
                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                //OpenGLMatrix pose = vuforiaListener.getPose();

                //OpenGLMatrix pose = vuforiaListener.getRawPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYX.XYZ, AngleUnit.DEGREES.DEGREES);


                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);
                    telemetry.addData("X=", toString().valueOf(tX));
                    telemetry.addData("Y=", toString().valueOf(tY));
                    telemetry.addData("Z=", toString().valueOf(tZ));
                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                    telemetry.addData("rX=", toString().valueOf(rX));
                    telemetry.addData("rY=", toString().valueOf(rY));
                    telemetry.addData("rZ=", toString().valueOf(rZ));
                }
               //break;
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
            idle();
        }

        while (opModeIsActive()) {

            ///disable camera and trackable.
            //CameraDevice.getInstance().stop();
            telemetry.addData("Camera", "to be stopped");
            //relicTrackables.deactivate();  // try to shutodwn the tracking. disable the camera
            telemetry.addData("Trackables", "to be stopped");
            telemetry.update();
            idle();
            break;
        }

    }

    private String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
