package org.firstinspires.ftc.teamcode.Robotin30Code;

import com.revAmped.components.Button;
import com.revAmped.components.TankDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.revAmped.components.RobotRevAmped;
import com.qualcomm.robotcore.util.Range;
import com.revAmped.linear.components.RobotRevAmpedLinear;


/**
 * Created by John Wang on 9/2/2018.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="6 Wheel TeleOp", group="Test")
public class TeleOp6WheelTest
        extends OpMode {
    private RobotRevAmped robot;

    private TankDrive drive;

    private float pwrLeft, pwrRight;

    private boolean isSlow;

    private static final Button BTN_HALF_GEAR = new Button();

    @Override
    public void init() {
        robot = new RobotRevAmped(this, false);
        drive = robot.getTankDrive();
        this.gamepad1.reset();
        this.gamepad1.setJoystickDeadzone(0.1f);
        this.gamepad2.reset();
        this.gamepad2.setJoystickDeadzone(0.1f);
    }
    @Override
    public void stop() {
        robot.close();
    }

    @Override
    public void loop() {
        long timestamp = System.currentTimeMillis();

        long timeLeft = (120000-timestamp)/1000;
        if (timeLeft <= 0 ) {
            telemetry.addLine("TeleOp over");
            timeLeft = 0;
        }

        telemetry.addData("Time Left in TeleOp", timeLeft);

        //y axis is reversed by default
        float x1 = gamepad1.left_stick_x;
        float y1 = -gamepad1.left_stick_y;
        float x2 = gamepad1.right_stick_x;
        float y2 = -gamepad1.right_stick_y;

        //slow mode (45% of max speed)
        if (gamepad1.y && BTN_HALF_GEAR.canPress(timestamp) ) {
            isSlow = !isSlow;
        }
        if (isSlow) {
            float spdMult = 0.45f;
            y1 *= spdMult;
            y2 *= spdMult;
            robot.ledYellow.on();
        } else {
            robot.ledYellow.off();
        }

        //Make sure left and right powers are within motor power range -1 to 1
        pwrLeft = Range.clip(y1, -1f, 1f);
        pwrRight = Range.clip(y2, -1f, 1f);

        //Drive Function
        drive.setPower(pwrLeft, pwrRight);



    }
}
