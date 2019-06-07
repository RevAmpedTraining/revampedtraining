package org.firstinspires.ftc.teamcode.Robotin30Code;

import com.revAmped.components.Button;
import com.revAmped.util.TipOverProtection;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.revAmped.components.RobotRevAmped2;
import com.revAmped.components.MecanumDrive;


/**
 * TeleOp Chezy Test for a 6 wheel bas (works on mecanum)
 * Program for Revamped 12808
 * Created 6/20/2018
 */
//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Chezy Drive Test", group="Test")
public class TeleOp6WheelChezyTest
        extends OpMode {

    private RobotRevAmped2 robot;

    private MecanumDrive drive;

    private float pwrTurn, pwrDrive;
    private float tRight, tLeft;

    private boolean isSlow = false;
    //private boolean isReverse = false;

    //private final static DecimalFormat numberFormatter = new DecimalFormat("######");

    private TipOverProtection forwordProtection = new TipOverProtection(true);
    private TipOverProtection backwordProtection = new TipOverProtection(false);
    /*GamePad Controls:
        Slow Mode                     GP1               Y
        Quick Turn Right              GP1               Right Trigger
        Quick Turn Left               GP1               Left Trigger
        Turning                       GP1               Left X-axis joystick
        Driving                       GP1               Right Y-axis joystick
     */
      private static final Button BTN_HALF_GEAR = new Button();//Y
    //private static final Button BTN_REVERSE_MODE = new Button();
    //private static final Button BTN_CLAW_RELEASE = new Button();
    //private static final Button BTN_CLAW_RECLAMP = new Button();
    //private static final Button BTN_CLAW_IN = new Button();
    //private static final Button BTN_CLAW_OUT = new Button();

    //private final static float TURN_MULT = 0.6f;
    /*private enum clawState {
        IN,
        OUT,
        RELEASE,
        RECLAMP;

    }*/
    //clawState currentClawState = clawState.IN;
    /* private enum slideState {
         SLIDEUP,
         SLIDEDOWN,
         LEVELONE,
         LEVELTWO,
         LEVELTHREE,
         LEVELFOUR;

     }*/ //this is for levels and moving slide up and down
    @Override
    public void init() {
        robot = new RobotRevAmped2(this,
                false);
        drive = robot.getMecanumDrive();
        //reset the joysticks and set the deadzone to 0.1 out of 1
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

        // y2 is reversed by default so joystick corresponds to motor power
        float x1 = gamepad1.left_stick_x;
        float y2 = -gamepad1.right_stick_y;

        //float y1 = -gamepad1.left_stick_y;
        //float x2 = gamepad1.right_stick_x;
        //float tx1 = gamepad2.left_stick_x;
        //float ty1 = gamepad2.left_stick_y;
        //float tx2 = gamepad2.right_stick_x;
        //float ty2 = gamepad2.right_stick_y;

        pwrTurn = Range.clip(x1, -1f, 1f);
        pwrDrive = Range.clip(y2, -1f, 1f);
        //all speeds multiplied by this factor
        float SPEED_FACTOR = 0.85f;

        if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
            if (pwrTurn < 0.1f && pwrTurn > -0.1f) {
                drive.setPower(pwrDrive * SPEED_FACTOR,
                        pwrDrive * SPEED_FACTOR);
            } else if (pwrTurn < -0.3f || pwrTurn > 0.3f) {
                drive.setPower((pwrDrive + pwrTurn * pwrDrive)/2 * SPEED_FACTOR,
                        (pwrDrive - pwrTurn * pwrDrive)/2 * SPEED_FACTOR);
            } else if (pwrTurn < 0.3f && pwrTurn > 0.1f) {
                pwrTurn = 0.3f;
                drive.setPower((pwrDrive + pwrTurn * pwrDrive)/2 * SPEED_FACTOR,
                        (pwrDrive - pwrTurn * pwrDrive)/2 * SPEED_FACTOR);
            } else {
                pwrTurn = -0.3f;
                drive.setPower((pwrDrive + pwrTurn * pwrDrive)/2 * SPEED_FACTOR,
                        (pwrDrive - pwrTurn * pwrDrive)/2 * SPEED_FACTOR);
            }
        }
        //braking when left or right joystick is 0
        if (pwrDrive == 0) {
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        float strafePower = 0.85f;
        if (gamepad1.y && BTN_HALF_GEAR.canPress(timestamp)) {
            isSlow = !isSlow;
        }
        if (isSlow) {
            strafePower = 0.5f;
            robot.ledYellow.on();
        } else {
            robot.ledYellow.off();
        }
        //quickturn buttons for turning in place
        tRight = Button.scaleInput(gamepad1.right_trigger);
        tLeft = Button.scaleInput(gamepad1.left_trigger);
        tRight = Range.clip(tRight, -1f, 1f);
        tLeft = Range.clip(tLeft, -1f, 1f);
        while (tRight > 0.15f) {
            drive.setPower(tRight,
                    -tRight);
            tRight = Button.scaleInput(gamepad1.right_trigger);
            tRight = Range.clip(tRight, -1f, 1f);
        }
        while (tLeft > 0.15f) {
            drive.setPower(-tLeft,
                    tLeft);
            tLeft = Button.scaleInput(gamepad1.left_trigger);
            tLeft = Range.clip(tLeft, -1f, 1f);
        }
       /* if (gamepad2.right_bumper && BTN_CLAW_IN.canPress(timestamp)) {
            currentClawState = clawState.IN;
        } else if (gamepad2.left_bumper && BTN_CLAW_OUT.canPress(timestamp)) {
            currentClawState = clawState.OUT;
        } else if ((gamepad2.right_trigger > 0.9) && BTN_CLAW_RELEASE.canPress(timestamp)) {
            currentClawState = clawState.RELEASE;
        } else if ((gamepad2.left_trigger > 0.9) && BTN_CLAW_RECLAMP.canPress(timestamp)) {
            currentClawState = clawState.RECLAMP;
        }

        if (currentClawState == clawState.IN) {
            robot.servoClawLeft.setPosition(RobotRevAmpedConstants.SERVO_CLAW_LEFT_IN);
            robot.servoClawRight.setPosition(RobotRevAmpedConstants.SERVO_CLAW_RIGHT_IN);
        } else if (currentClawState == clawState.OUT) {
            robot.servoClawLeft.setPosition(RobotRevAmpedConstants.SERVO_CLAW_LEFT_OUT);
            robot.servoClawRight.setPosition(RobotRevAmpedConstants.SERVO_CLAW_RIGHT_OUT);
        } else if (currentClawState == clawState.RELEASE) {

            robot.servoClawLeft.setPosition(RobotRevAmpedConstants.SERVO_CLAW_LEFT_RELEASE);
            robot.servoClawRight.setPosition(RobotRevAmpedConstants.SERVO_CLAW_RIGHT_RELEASE);

        }

        boolean isTouchSwitchSlideDown = robot.switchSlideDown.isTouch();
        if (isTouchSwitchSlideDown) {
            robot.slideLeft.resetPosition(0);
            robot.slideRight.resetPosition(0);
        }
        //slide
        if (gamepad2.dpad_up && !robot.switchSlideUp.isTouch()) {
            robot.slideLeft.setPower(RobotConstants.POWER_SLIDE);
            robot.slideRight.setPower(RobotConstants.POWER_SLIDE);
        } else if (gamepad2.dpad_down && !isTouchSwitchSlideDown) {
            robot.slideLeft.setPower(-RobotConstants.POWER_SLIDE);
            robot.slideRight.setPower(-RobotConstants.POWER_SLIDE);
        } else {
            robot.slideLeft.setPower(0);
            robot.slideRight.setPower(0);
        }*/
        //level buttons

        robot.drawLed();

        //telemetry.addData("Reverse", Boolean.toString(isReverse));
        //telemetry.update();

    }
}
