package org.firstinspires.ftc.teamcode.OldStuff;

import com.revAmped.components.Button;
import com.revAmped.components.RobotRevAmped;
import com.revAmped.components.TankDrive;
import com.revAmped.config.RobotConstants;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.util.TipOverProtection;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;


/**
 * TeleOp Refactored
 * Program for Revamped 12808
 * 10/31/2017
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpRevAmped", group="Game")
public class TeleOpRevAmped
    extends OpMode {

    private RobotRevAmped robot;

    private TankDrive drive;

    private float pwrLeft, pwrRight;
    int levelOne = 350;//random number-calculate later
    int levelTwo = 700;//random number-calculate later
    int levelThree = 1050;//random number-calculate later
    int levelFour = 1400;//random number-calculate later
    int currentPosition = 0;


    private boolean isSlow = false;
    private boolean isReverse = false;

    private final static DecimalFormat numberFormatter = new DecimalFormat("######");

    private TipOverProtection forwordProtection = new TipOverProtection(true);
    private TipOverProtection backwordProtection = new TipOverProtection(false);

    private static final Button BTN_HALF_GEAR = new Button();
    private static final Button BTN_REVERSE_MODE = new Button();
    private static final Button BTN_LEVEL_ONE = new Button();
    private static final Button BTN_LEVEL_TWO = new Button();
    private static final Button BTN_LEVEL_THREE = new Button();
    private static final Button BTN_LEVEL_FOUR = new Button();
    private static final Button BTN_CLAW_RELEASE = new Button();
    private static final Button BTN_CLAW_RECLAMP = new Button();

    private static final Button BTN_CLAW_IN = new Button(); //RB2
    private static final Button BTN_CLAW_OUT = new Button(); //LB2

    private final static float TURN_MULT = 0.6f;
    private enum clawState {
        IN,
        OUT,
        RELEASE,
        RECLAMP;

    }
clawState currentClawState = clawState.IN;
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
        robot = new RobotRevAmped(this,
                                  false);
        drive = robot.getTankDrive();

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
        float ty1 = gamepad2.left_stick_y;
        float tx2 = gamepad2.right_stick_x;
        float ty2 = gamepad2.right_stick_y;

        pwrLeft = Button.scaleInput(y1);
        pwrRight = Button.scaleInput(y2);

        // drive
        //slow mode
        if (gamepad1.right_bumper && BTN_HALF_GEAR.canPress(timestamp)) {
            isSlow = !isSlow;
        }
        if (isSlow) {
            pwrLeft *= TURN_MULT;
            pwrRight *= TURN_MULT;
            robot.ledYellow.on();
        } else {
            robot.ledYellow.off();
        }
        //reverse mode
        if (gamepad1.left_bumper && BTN_REVERSE_MODE.canPress(timestamp)) {
            isReverse = !isReverse;
        }
        if (isReverse) {
            // not ((y1 >= 0 && y2 <= 0) || (y1 <= 0 && y2 >= 0))
            // y1 and y2 are same sign
            pwrLeft = -pwrLeft;
            pwrRight = -pwrRight;
            robot.ledBlue.on();
        }
        else {
            robot.ledBlue.off();
        }

        pwrLeft = Range.clip(pwrLeft, -1f, 1f);
        pwrRight = Range.clip(pwrRight, -1f, 1f);

        int slideRightPosition = robot.slideRight.getCurrentPosition();
        int slideLeftPosition = robot.slideLeft.getCurrentPosition();
        // factor for floating motors
        float floatFactor = 2f*Math.abs(slideRightPosition+slideLeftPosition)/RobotRevAmpedConstants.SLIDE_ENCODER_MAX;
        if (isSlow) {
            floatFactor *= TURN_MULT;
        }

        //prevent tip forwards
        forwordProtection.protect(pwrLeft,
                                  pwrRight);
        backwordProtection.protect(pwrLeft,
                                   pwrRight);
        if (forwordProtection.isPowerAdjusted()) {
            pwrLeft = forwordProtection.getLeftPower();
            pwrRight = forwordProtection.getRightPower();
        }
        else if (backwordProtection.isPowerAdjusted()) {
            pwrLeft = backwordProtection.getLeftPower();
            pwrRight = backwordProtection.getRightPower();
        }
        drive.setPower(pwrLeft*0.7f,
                       pwrRight*0.7f);

        if (pwrLeft == 0) {
            if (isSlow || slideLeftPosition + slideRightPosition < 600) {
                drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else {
                drive.leftGradualBrake.brake(floatFactor);
            }
        }
        if (pwrRight == 0) {
            if (isSlow || slideLeftPosition + slideRightPosition < 600) {
                drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else {
                drive.rightGradualBrake.brake(floatFactor);
            }
        }

        if (gamepad2.right_bumper && BTN_CLAW_IN.canPress(timestamp)) {
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
        }
        //level buttons

        robot.drawLed();

        //telemetry.addData("Reverse", Boolean.toString(isReverse));
        //telemetry.update();
        
    }
}
