package org.firstinspires.ftc.teamcode.MotorTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LinearOpModeMotorFirstAttempt extends LinearOpMode {

    private DcMotor motor;
    private ElapsedTime elapsedTime;

    public void runOpMode() {
        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");

        //while (runOpModeIsActive())
        //motor.setpower(0.5f);
    }

}
