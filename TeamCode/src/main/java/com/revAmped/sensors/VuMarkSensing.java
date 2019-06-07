package com.revAmped.sensors;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.OpenCVLoader;

import com.revAmped.util.VuforiaLocalizerImplSubClass;
import com.vuforia.CameraDevice;

/**
 * Created by John Wang on 11/23/2017.
 */

public class VuMarkSensing  {
    HardwareMap hardwareMap;
    private final String TAG = "Vuforia";
    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
    VuforiaTrackable relicTemplate;
    VuforiaTrackables relicTrackables = null;
    public VuforiaLocalizerImplSubClass vuforia = null;
    public VuMarkSensing (HardwareMap hwMap) {

        hardwareMap = hwMap;

    }

    public Boolean initialize()
    {
        Boolean ret = false;
        try {
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
            vuforia = new VuforiaLocalizerImplSubClass(parameters);
            /**
             * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
             * in this data set: all three of the VuMarks in the game were created from this one template,
             * but differ in their instance id information.
             * @see VuMarkInstanceId
             */
            relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
            relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

            relicTrackables.activate();

            RobotLog.vv("VuforiaInit", "Success");
            if (OpenCVLoader.initDebug()) {
                RobotLog.vv("VuforiaInit", "OpenCV lib loaded successfully.");
                ret = true;
            }
            else
            {
                RobotLog.vv("VuforiaInit", "OpenCV lib failed to load.");
            }
        }
        catch(Exception e) {
            RobotLog.ee("VuforiaInit", e.getMessage());
            ret = false;
        }
        return ret;
    }

    /*
  * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
  * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
  */
    public  RelicRecoveryVuMark getGlyphColumnKey() {
        try {




            // if the vumark is not seen, continue;  but we have to keep in mind that it cannot loop here forever.
            // the right thing is to use timeout:  see 3 sec. if cannot see vumark in 3 sec,
            // return to autonomous to do other tasks
            vuMark = RelicRecoveryVuMark.UNKNOWN;
            int timeout = 3000;
            boolean vuMarkFound = false;
            long startTime = System.currentTimeMillis();
            long TimeElapsed = System.currentTimeMillis() - startTime;
            while (!vuMarkFound && TimeElapsed < timeout) {

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
                    // telemetry.addData("VuMark", "%s visible", vuMark);


                    //todo:  base the value of vuMark,  assign the key column.

                    vuMarkFound = true;  // to exist the loop.
                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                /*OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));*/

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
               /* if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }*/
                } /*else {
                telemetry.addData("VuMark", "not visible");
            }*/
                TimeElapsed = System.currentTimeMillis() - startTime;
                //telemetry.update();
                // idle();
            }
        }
        catch(Exception e)
        {
            /*RobotLog.e(TAG, "exception: %s ", e.getMessage());
            if (vuMark == RelicRecoveryVuMark.UNKNOWN)
            {
                vuMark = RelicRecoveryVuMark.RIGHT;
            }*/
        }
        /*try
        {
            ///disable camera and trackable.
            CameraDevice.getInstance().stop();
            if (relicTrackables !=null) relicTrackables.deactivate();  // try to shutodwn the tracking. disable the camera
        }
        catch (Exception e){}*/
        return vuMark;
    }
    public void stop()
    {
        try
        {
            ///disable camera and trackable.
            CameraDevice.getInstance().stop();
          //  telemetry.addData("Camera", "to be stopped");
            relicTrackables.deactivate();  // try to shutodwn the tracking. disable the camera
        }
        catch(Exception e)
        {

        }
    }
}