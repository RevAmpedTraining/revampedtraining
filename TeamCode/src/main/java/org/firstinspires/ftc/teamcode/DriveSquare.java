package org.firstinspires.ftc.teamcode;

import android.gesture.GestureOverlayView;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.revAmped.components.HwBnoGyro;
import com.revAmped.components.HwGyro;

/**
 * Created by swang4 on 6/11/2019.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Test Square", group = "Test")
public class DriveSquare extends OpMode {

    private DcMotor leftfront = null;
    private DcMotor rightfront = null;
    private DcMotor leftback = null;
    private DcMotor rightback = null;

    private HwGyro gyrosensor = null;

    long time = System.currentTimeMillis();

    @Override
    public void init() {
        leftfront = hardwareMap.get(DcMotor.class, "leftfront");
        rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        leftback = hardwareMap.get(DcMotor.class, "leftback");
        rightback = hardwareMap.get(DcMotor.class, "rightback");

        gyrosensor = new HwBnoGyro(hardwareMap,
                "gyrosensor");
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addLine("Status: Initialiazed");
        telemetry.update();
    }
    @Override
    public void init_loop() {
        time = System.currentTimeMillis();

        gyrosensor.resetHeading();
    }
    @Override
    public void loop() {
        try {
            driveForward();
            turn();
            driveForward();
            turn();
            driveForward();
            turn();
            driveForward();
        } catch (Exception e) {

        }



    }
    public void driveForward()
        throws InterruptedException {
        int targetTicks = 6000;

        leftfront.setTargetPosition(targetTicks);
        rightfront.setTargetPosition(targetTicks);
        leftback.setTargetPosition(targetTicks);
        rightback.setTargetPosition(targetTicks);

        float p = 0.5f;

        leftfront.setPower(p);
        rightfront.setPower(p);
        leftback.setPower(p);
        rightback.setPower(p);

        while (leftfront.isBusy() && rightfront.isBusy()
                &&leftback.isBusy() && rightback.isBusy()) {
            //motors moving
        }

        leftfront.setPower(0);
        rightfront.setPower(0);
        leftback.setPower(0);
        rightback.setPower(0);
    }
    public void turn() {
        double heading = gyrosensor.getHeading();

        float p = 0.5f;

        while (heading < 90) {
            leftfront.setPower(p);
            rightfront.setPower(-p);
            leftback.setPower(p);
            rightback.setPower(-p);

            heading = gyrosensor.getHeading();
        }

    }

    @Override
    public void stop() {}


}
