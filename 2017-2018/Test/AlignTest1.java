package org.firstinspires.ftc.teamcode.Test;

import com.revAmped.linear.components.RobotOverchargedLinear2;
import com.revAmped.linear.components.SwerveDriveLinear;
import com.revAmped.sensors.RevColorDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "OverchargedAlignTest1", group = "Test")
public class AlignTest1 extends OpMode {
    private ElapsedTime     runtime = new ElapsedTime();
    ///Overcharged Autonomous Robot class
    private RobotOverchargedLinear2 robot;
    ///Overcharged Swirve Drive class
    private SwerveDriveLinear drive;
    RevColorDistanceSensor sensorAlign;

    @Override
    public void init() {
        // init
        //robot = new RobotOverchargedLinear2(this);
        //drive = robot.getSwerveDriveLinear();

        // get a reference to the distance sensor that shares the same name.
        sensorAlign =  new RevColorDistanceSensor(hardwareMap, "sensor_align");

		/*runtime.reset();
		while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }*/
    }

    public void loop()
    {
        boolean isRed = false;
        ///distance from the Dumper edge = 4 inches = 10.16 CM
        ///So if the distance is 10.16  alignement is correct and distance to move is 0 i.e. don't move
        RevColorDistanceSensor.COLORTYPE columnColor = sensorAlign.getColor();
        boolean red = false;
        boolean blue = false;
        if (columnColor == RevColorDistanceSensor.COLORTYPE.BLUE) {
            blue = true;
        } else if (columnColor == RevColorDistanceSensor.COLORTYPE.RED) {
            red = true;
        }
        double currentReading = sensorAlign.getDistance();
        telemetry.addData("Color", (isRed ? "Red" : (blue ? "Blue" : "-")));
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", currentReading));
        //if ((isRed && red) !! (!isRed && blue)) {
        ///do the correction only when the column matches
        float expectedDistance = 10.16f;
        double distance = currentReading / expectedDistance;
        if (currentReading > expectedDistance) {
            ///greater that 10.16 means robot moves to the left
            telemetry.addData("Move", String.format(Locale.US, "Left %.02f", distance));
            //drive.moveToEncoderInch(TurnType.STRAFE,
            //-distance, 0.2f, 30000, false,
            //false,
            //true);
        } else if (currentReading < expectedDistance) {
            ///lesser than 10.16 means robot moves to the right
            telemetry.addData("Move", String.format(Locale.US, "Right %.02f", distance));
            //drive.moveToEncoderInch(TurnType.STRAFE,
            //distance, 0.2f, 30000, false,
            //false,
            //true);
        }
        //}
    }
}