package com.revAmped.test;

import com.revAmped.components.Button;
import com.revAmped.sensors.AdafruitBNO055;
import com.revAmped.sensors.AdafruitBNO055IMU;
import com.revAmped.sensors.EulerAngle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cDevice;

// Example program on how to use the AdafruitBNO055 class

public class AdafruitIMUTest
    extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        I2cDevice adafruitIMUDevice = hardwareMap.i2cDevice.get("i2cdev");
        AdafruitBNO055IMU imu = new AdafruitBNO055IMU(this,
                                                      adafruitIMUDevice);
        try {
            //telemetry.setSorted(false);

            waitForStart();

            imu.resetHeading();
            while (opModeIsActive()) {
                long timeStamp = System.currentTimeMillis();

                EulerAngle angles = imu.getAngularOrientation();
                telemetry.addData("Euler", angles);

                int heading = imu.getHeading();
                telemetry.addData("Head", Integer.toString(heading));

                AdafruitBNO055.CalibrationStatus calStat = imu.getCalibrationStatus();
                telemetry.addData("Calib", calStat);

                if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp) &&
                    calStat.isAllCalibrated()) {
                        imu.dumpCalibrationData();
                        telemetry.addData("Dumped", "Dumped calibration data");
                        sleep(500);
                }
                else {
                    telemetry.addData("Dumped", "");
                }

                if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                    imu.resetHeading();
                }

                telemetry.addData("Reset", "Start");
                telemetry.addData("Dump", "Back");
                idle();
            }
        }
        finally {
           imu.close();
        }
    }
}
