package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LinearOpModeMotor extends LinearOpMode {

    private DcMotor motor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "motor");

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(0.5f);
        }

        motor.close();
    }
}
