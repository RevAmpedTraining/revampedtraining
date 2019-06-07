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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpSwerve", group="Game")
public class TeleOpSwerve
    extends OpMode {

    private Robot robot;

    private SwerveDrive drive;

    private boolean isStrafe = false;

    private final static float TURN_MULT = 0.5f;

    private static final Button BTN_FORWARD = new Button(); //A
    private static final Button BTN_STRAFE = new Button(); //B

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

        // drive

        x1 = Range.clip(x1, -1f, 1f);
        x2 = Range.clip(x2, -1f, 1f);
        y1 = Range.clip(y1, -1f, 1f);
        y2 = Range.clip(y2, -1f, 1f);

        //orientation
        if (gamepad1.b && BTN_STRAFE.canPress(timestamp)) {
            isStrafe = true;
            drive.stop();
            drive.setTurn(TurnType.STRAFE);
        }
        else if (gamepad1.a && BTN_FORWARD.canPress(timestamp)) {
            isStrafe = false;
            drive.stop();
            drive.setTurn(TurnType.FORWARD);
        } else if (y1 == 0 && y2 == 0 && x1 == 0 && x2 == 0) {
            drive.stop();
            if (isStrafe) //resume previous direction
            {
                drive.setTurn(TurnType.STRAFE);
            } else {
                drive.setTurn(TurnType.FORWARD);
            }
        }

        //straight tank
        else if ((Math.abs(y1) > Math.abs(x1) && Math.abs(y2) > Math.abs(x2)) ||
                (Math.abs(y1) > Math.abs(x1) && y2 == 0) ||
                (Math.abs(y2) > Math.abs(x2) && y1 == 0)) {
            isStrafe = false;
            if (Math.signum(y1) == -Math.signum(y2)) {
                drive.setTurn(TurnType.TURN_REGULAR);
                drive.setPower(y1, y2, TurnType.TURN_REGULAR);
            }
            else if (y1 == 0) {
                drive.setTurn(TurnType.TURN_SWERVE_FWD_TURN2);
                drive.setPower(y1*TURN_MULT, y2*TURN_MULT, TurnType.TURN_SWERVE_FWD_TURN2);
            }
            else if (y2 == 0) {
                drive.setTurn(TurnType.TURN_SWERVE_FWD_TURN);
                drive.setPower(y1*TURN_MULT, y2*TURN_MULT, TurnType.TURN_SWERVE_FWD_TURN);
            }
            else {
                drive.setTurn(TurnType.FORWARD);
                drive.setPower(y1, y2, TurnType.FORWARD);
             }
        }
        //sideways tank
        else if ((Math.abs(y1) < Math.abs(x1) && Math.abs(y2) < Math.abs(x2)) ||
                (Math.abs(y1) < Math.abs(x1) && x2 == 0) ||
                (Math.abs(y2) < Math.abs(x2) && x1 == 0)) {
            isStrafe = true;
            if (Math.signum(x1) == -Math.signum(x2)) {      //turning
                drive.setTurn(TurnType.TURN_REGULAR);
                drive.setPower(x2, x1, TurnType.TURN_REGULAR);
            }
            else if (x1 == 0) {
                drive.setTurn(TurnType.TURN_SWERVE_SIDE_TURN);
                drive.setPower(x2*TURN_MULT, x1*TURN_MULT, TurnType.TURN_SWERVE_SIDE_TURN);
            }
            else if (x2 == 0) {
                drive.setTurn(TurnType.TURN_SWERVE_SIDE_TURN2);
                drive.setPower(x2*TURN_MULT, x1*TURN_MULT, TurnType.TURN_SWERVE_SIDE_TURN2);
            }
            else {
                drive.setTurn(TurnType.STRAFE);
                drive.setPower(x2, x1, TurnType.STRAFE);
            }
        }
        else {
            drive.stop();
        }
    }
}
