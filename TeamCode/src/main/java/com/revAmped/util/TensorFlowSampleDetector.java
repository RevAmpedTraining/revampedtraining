/* Copyright (c) 2018 FIRST. All rights reserved.
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

package com.revAmped.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
//@TeleOp(name = "TFSampleOrder", group = "Concept")
//@Disabled
public class TensorFlowSampleDetector {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

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
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AUnaLq//////AAABmT0D69LQfkgFoITn3eo0Ceto4F3joKGgF3NenQQyr2bF3HtVceptuXmulwa1WsiGXvJoxJVYYVdJ65WVd3muN3usESQgiDsAmdvRMgXO+ZNdXGisovUPG1TT21+q0R82rqs+I4bcRKB52wNdOT7haM1w87Os3C/wE2McMlNaJka+q3TxxsRNyUfXMtAMxt48STqlNUQSqaCtrdq+FTQp2oN7ifytgkK58hN/dLRspApj5uVaT0W06iXPrYLG6JZjfwLMGr0EVN/PTeefyzVm8LftOO1+KRFjMfw/NcQTsYIZ1YWr2ifL55Ai4sC+0RlGUiIj3c8lQSlvXBfd4ILATKkLBfF8rNmlH0sgpspdVfZ6";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private boolean b_init = false;
    SampleOrderDetector.GoldenOrder  goldenOrder= SampleOrderDetector.GoldenOrder.UNKNOWN;  // default is CENTER, there is no panelty if wrong

    HardwareMap hardwareMap;
    public TensorFlowSampleDetector(HardwareMap hw)
    {
        hardwareMap = hw;
    }
    public SampleOrderDetector.GoldenOrder GetGoldLocation() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        if(!b_init) {
            try {
                initVuforia();
            } catch (Exception e) {
                RobotLog.ee("TFSample", "Failed to initVuforia=%s", e.getMessage());
                //throw e;
            }
            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
                RobotLog.vv("TFSample", "Successfully Initialized TF OD");
            } else {
                RobotLog.vv("TFSample", "This device is not compatible with TFOD");
            }

            if (tfod != null) {
                tfod.activate();
            }

            b_init = true;
        }

         if (tfod != null) {
             // getUpdatedRecognitions() will return null if no new information is available since
             // the last time that call was made.
             List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
             if (updatedRecognitions != null) {
                 RobotLog.vv("TFSample", "NumberOfMinerals=%d", updatedRecognitions.size());
                 //if (updatedRecognitions.size() == 3) {
                 if (updatedRecognitions.size() == 2) {   //camera only see two
                     int goldMineralX = -1;
                     int silverMineral1X = -1;
                     int silverMineral2X = -1;
                     for (Recognition recognition : updatedRecognitions) {
                         if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                             goldMineralX = (int) recognition.getLeft();
                         } else if (silverMineral1X == -1) {
                             silverMineral1X = (int) recognition.getLeft();
                         } else {
                             silverMineral2X = (int) recognition.getLeft();
                         }
                     }

                     if (goldMineralX != -1) {
                         if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                             goldenOrder = SampleOrderDetector.GoldenOrder.RIGHT;
                             RobotLog.vv("TFSample", "Gold->Right");
                         } else if (goldMineralX > silverMineral1X || goldMineralX > silverMineral2X) {
                             goldenOrder = SampleOrderDetector.GoldenOrder.CENTER;
                             RobotLog.vv("TFSample", "Gold->Center");
                         }
                     } else {
                         if (silverMineral1X != -1 && silverMineral2X != -1) {
                             goldenOrder = SampleOrderDetector.GoldenOrder.LEFT;
                             RobotLog.vv("TFSample", "Gold->Left");
                         } else {
                             goldenOrder = SampleOrderDetector.GoldenOrder.UNKNOWN;
                             RobotLog.vv("TFSample", "Gold->Unknown");
                         }
                     }
                        /*
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                          if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                              goldenOrder = SampleOrderDetector.GoldenOrder.LEFT;
                              RobotLog.vv("TFSample", "Gold->Left");
                          } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                              RobotLog.vv("TFSample", "Gold->Right");
                              goldenOrder = SampleOrderDetector.GoldenOrder.RIGHT;
                          } else {
                              RobotLog.vv("TFSample", "Gold->Center");
                              goldenOrder = SampleOrderDetector.GoldenOrder.CENTER;
                          }


                        }*/
                 }
             }
         }

        return goldenOrder;
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.60;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void  Shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
