package org.firstinspires.ftc.teamcode.MotorTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LinearOpModeMotor_FirstAttempt_NeilSurya extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(0.25f);
        }

        motor.close();
    }

}
