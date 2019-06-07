package org.firstinspires.ftc.teamcode.LastYear.RevAmpedTest;
import com.revAmped.components.HwSonarAnalog;
import com.revAmped.linear.components.RobotRevAmpedLinear2;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by John Wang on 2/20/2018.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RevAmped Alignment2", group = "Test")
public class RevAmpedAlignment extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private RobotRevAmpedLinear2 robot = null;
    public HwSonarAnalog sonarL;
    @Override
    public void init() {
        sonarL = new HwSonarAnalog(hardwareMap,
                "sonar_l",
                HwSonarAnalog.SCALE_MAX_XL);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        runtime.reset();
    }
    @Override
    public void loop() {
        float wallDistanceL;
        long timeStamp = System.currentTimeMillis();
        wallDistanceL = sonarL.getDistance();
        telemetry.addData("wallDistance", wallDistanceL);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        if (wallDistanceL>21 && wallDistanceL<22) {
            telemetry.addLine("Aligned....Sai!!!!!!!!!!!");
        }
        telemetry.update();
    }
    @Override
    public void stop() {

    }
}
