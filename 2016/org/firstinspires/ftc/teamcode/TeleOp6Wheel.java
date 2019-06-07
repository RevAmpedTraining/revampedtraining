package org.firstinspires.ftc.teamcode;

import com.revAmped.components.Button;
import com.revAmped.components.Robot;
import com.revAmped.components.SwerveDrive;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotConstants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp
 */
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp6Wheel2", group="Game")
public class TeleOp6Wheel
    extends OpMode {

    private Robot robot;

    private SwerveDrive drive;

    @Override
    public void init() {
        robot = new Robot(this,
                          false);
        drive = robot.getSwerveDrive();

        this.gamepad1.reset();
        this.gamepad1.setJoystickDeadzone(0.15f);
        this.gamepad2.reset();
        this.gamepad2.setJoystickDeadzone(0.15f);
    }

    @Override
    public void stop() {
        robot.close();
    }

    @Override
    public void loop() {

        long timestamp = System.currentTimeMillis();

        // is reversed by default
        float x1 = gamepad1.left_stick_x;
        float y1 = -gamepad1.left_stick_y;
        float x2 = gamepad1.right_stick_x;
        float y2 = -gamepad1.right_stick_y;

        float tx1 = gamepad2.left_stick_x;
        float ty1 = -gamepad2.left_stick_y;
        float tx2 = gamepad2.right_stick_x;
        float ty2 = -gamepad2.right_stick_y;

        x1 = Button.scaleInput(x1);
        x2 = Button.scaleInput(x2);
        y1 = Button.scaleInput(y1);
        y2 = Button.scaleInput(y2);

        x1 = Range.clip(x1, -1f, 1f);
        x2 = Range.clip(x2, -1f, 1f);
        y1 = Range.clip(y1, -1f, 1f);
        y2 = Range.clip(y2, -1f, 1f);

        drive.setPower(y1, y2, TurnType.FORWARD);
    }
}

/*package org.firstinspires.ftc.teamcode;

        import com.hotwired.components.Button;
        import com.hotwired.components.Robot;
        import com.hotwired.components.SwerveDrive;
        import com.hotwired.components.TurnType;
        import com.hotwired.config.RobotConstants;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.Range;


        extends OpMode {

    private Robot robot;

    public enum EEncoderStatus
    {
        PRE_USE_ENCODER,
        USE_ENCODER,
        PRE_WITHOUT_ENCODER,
        WITHOUT_ENCODER;
    }

    @Override
    public void init() {
        robot = new Robot(this,
                false);
        leftFrontMotor = hardwareMap.dcMotor.get("lf_motor");
        rightFrontMotor = hardwareMap.dcMotor.get("rf_motor");
        leftBackMotor = hardwareMap.dcMotor.get("lb_motor");
        rightBackMotor = hardwareMap.dcMotor.get("rb_motor");

        this.gamepad1.reset();
        this.gamepad1.setJoystickDeadzone(0.15f);
        this.gamepad2.reset();
        this.gamepad2.setJoystickDeadzone(0.15f);
    }

    @Override
    public void stop() {
        robot.close();
    }

    @Override
    public void loop() {

        long timestamp = System.currentTimeMillis();

        // is reversed by default
        float x1 = gamepad1.left_stick_x;
        float y1 = -gamepad1.left_stick_y;
        float x2 = gamepad1.right_stick_x;
        float y2 = -gamepad1.right_stick_y;

        float tx1 = gamepad2.left_stick_x;
        float ty1 = -gamepad2.left_stick_y;
        float tx2 = gamepad2.right_stick_x;
        float ty2 = -gamepad2.right_stick_y;

        x1 = Button.scaleInput(x1);
        x2 = Button.scaleInput(x2);
        y1 = Button.scaleInput(y1);
        y2 = Button.scaleInput(y2);

        // drive
        x1 = Range.clip(x1, -1f, 1f);
        x2 = Range.clip(x2, -1f, 1f);
        y1 = Range.clip(y1, -1f, 1f);
        y2 = Range.clip(y2, -1f, 1f);

        //telemetry.addData("Reverse", Boolean.toString(isReverse));
        //telemetry.update();
    }
}*/
