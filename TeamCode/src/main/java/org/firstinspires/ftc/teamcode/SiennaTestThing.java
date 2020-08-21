package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.revAmped.components.Button;
import com.revAmped.components.RobotRevAmpedSkystone;

public class SiennaTestThing extends OpMode {

    private RobotRevAmpedSkystone robot;

    @Override
    public void init() {

        robot = new RobotRevAmpedSkystone(this, false);
        gamepad1.reset();

    }

    @Override
    public void loop() {

        long timestamp = System.currentTimeMillis();
        if(gamepad1.start && Button.BTN_START.canPress(timestamp)) {
            robot.gyroSensor.getHeading();
        }
        else if(gamepad1.left_stick_button && Button.BTN_BACK.canPress(timestamp)) {
            stop();
        }
        telemetry.addData("Heading", robot.gyroSensor.getHeading());
        telemetry.addData("Reset", "Start");
        telemetry.addData("Stop", "Left_Trigger");
        telemetry.update();

    }

    @Override
    public void stop() {
        super.stop();
    }
}
