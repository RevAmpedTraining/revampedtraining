package org.firstinspires.ftc.teamcode.Test;

import com.revAmped.components.Button;
import com.revAmped.components.HwSonarAnalog;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Parthiv on 1/29/2018.
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ParthivSonarSensor", group = "Individual")
public class SonarTester extends OpMode {
    private static final Button BTN_BACK = new Button();
    public HwSonarAnalog sonarL;
    public HwSonarAnalog sonarR;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void init() {
        sonarL = new HwSonarAnalog(hardwareMap,
                "sonarL",
                HwSonarAnalog.SCALE_MAX_XL);
        sonarR = new HwSonarAnalog(hardwareMap,
                "sonarR",
                HwSonarAnalog.SCALE_MAX_XL);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        float distanceL = sonarL.getDistance();
        float distanceR = sonarR.getDistance();
        telemetry.addLine("Sensor " + sonarL);
        telemetry.addData("Distance", Float.toString(distanceL));
        telemetry.addLine("Sensor " + sonarR);
        telemetry.addData("Distance", Float.toString(distanceR));
        telemetry.addData("Stop", "Back");
        telemetry.update();
    }

}
