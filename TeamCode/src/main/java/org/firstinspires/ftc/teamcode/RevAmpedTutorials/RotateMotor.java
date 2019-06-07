package org.firstinspires.ftc.teamcode.RevAmpedTutorials;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

/**
 * RevAmped Test Motor 6/6/2019
 * Rotate a simple Dc Motor
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Motor Test", group = "2019 Test")
public class RotateMotor extends OpMode{
    //declare object
    private DcMotor testMotor = null;
    private DistanceSensor sonar = null;

    public void init() {
        //initialize object
        testMotor = hardwareMap.get(DcMotor.class, "test_drive");
        sonar = hardwareMap.get(DistanceSensor.class, "test_sonar");
        //set direction
        testMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Status: Initialized");
        telemetry.addLine("Press Play to Begin");
        telemetry.update();
    }
    public void loop() {
        //as long as the program is running, set motor power to 0.5f
        testMotor.setPower(0.5f);
    }
    public void stop() {

    }
}
