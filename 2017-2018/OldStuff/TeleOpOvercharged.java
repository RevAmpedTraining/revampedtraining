package org.firstinspires.ftc.teamcode.OldStuff;

import com.revAmped.components.Button;
import com.revAmped.components.RobotOvercharged;
import com.revAmped.components.SwerveDrive;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotConstants;
import com.revAmped.config.RobotOverchargedConstants;
import com.revAmped.util.TipOverProtection;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static com.revAmped.config.RobotOverchargedConstants.SERVO_CLAW_RIGHT_OUT;

/**
 * TeleOp
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpOvercharged", group="Game")
public class TeleOpOvercharged
    extends OpMode {

    private RobotOvercharged robot;

    private SwerveDrive drive;

    private boolean isStrafe = false;
    private boolean isSlow = false;
    private boolean isReverse = false;

    //private boolean isHold = false;

    private TipOverProtection forwordProtection = new TipOverProtection(true);
    private TipOverProtection backwordProtection = new TipOverProtection(false);

    private static final Button BTN_FORWARD = new Button(); //A
    private static final Button BTN_STRAFE = new Button(); //B
    private static final Button BTN_HALF_GEAR = new Button(); //RB1
    private static final Button BTN_REVERSE_MODE = new Button(); //LB1

    private static final Button BTN_CLAW_IN = new Button(); //RB2
    private static final Button BTN_CLAW_OUT = new Button(); //LB2
    private static final Button BTN_CLAW_MID = new Button(); //LT2

    private final static float TURN_MULT = 0.4f;

    private enum clawState {
        IN,
        OUT,
        MID;
    }

    clawState currentClawState = clawState.IN;

    @Override
    public void init() {
        robot = new RobotOvercharged(this,
                                     false);
        drive = robot.getSwerveDrive();

        this.gamepad1.reset();
        this.gamepad1.setJoystickDeadzone(0.15f);
        this.gamepad2.reset();
        this.gamepad2.setJoystickDeadzone(0.15f);
        //don't do this because its a penalty
        //robot.servoClawLeft.setPosition(RobotOverchargedConstants.SERVO_CLAW_RIGHT_OUT);
        //robot.servoClawRight.setPosition(RobotOverchargedConstants.SERVO_CLAW_RIGHT_OUT);
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
        if (gamepad1.right_bumper && BTN_HALF_GEAR.canPress(timestamp)) {
            isSlow = !isSlow;
        }
        if (isSlow) {
            x1 *= TURN_MULT;
            x2 *= TURN_MULT;
            y1 *= TURN_MULT;
            y2 *= TURN_MULT;
            robot.ledYellow.on();
        } else {
            robot.ledYellow.off();
        }

        if (gamepad1.left_bumper && BTN_REVERSE_MODE.canPress(timestamp)) {
            isReverse = !isReverse;
        }
        if (isReverse) {
            float x = x1;
            float y = y1;
            x1 = -x2;
            y1 = -y2;
            x2 = -x;
            y2 = -y;
            robot.ledBlue.on();
        }
        else {
            robot.ledBlue.off();
        }

        x1 = Range.clip(x1, -1f, 1f);
        x2 = Range.clip(x2, -1f, 1f);
        y1 = Range.clip(y1, -1f, 1f);
        y2 = Range.clip(y2, -1f, 1f);

        int slideRightPosition = robot.slideRight.getCurrentPosition();
        int slideLeftPosition = robot.slideLeft.getCurrentPosition();
        // factor for floating motors
        float floatFactor = 2f*Math.abs(slideRightPosition+slideLeftPosition)/RobotOverchargedConstants.SLIDE_ENCODER_MAX;
        if (isSlow) {
            floatFactor *= TURN_MULT;
        }

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

        //sideways tank
        else if ((Math.abs(y1) < Math.abs(x1) && Math.abs(y2) < Math.abs(x2)) ||
                (Math.abs(y1) < Math.abs(x1) && x2 == 0) ||
                (Math.abs(y2) < Math.abs(x2) && x1 == 0)) {
            isStrafe = true;
            if (Math.signum(x1) == -Math.signum(x2)) {      //turning
                drive.setTurn(TurnType.TURN_REGULAR);
                drive.setPower(isSlow ? x2*TURN_MULT : x2*0.6f,
                               isSlow ? x1*TURN_MULT : x1*0.6f,
                               TurnType.TURN_REGULAR);
            }
            else if (x1 == 0) {
                drive.setTurn(TurnType.TURN_SWERVE_SIDE_TURN);
                drive.setPower(x2*TURN_MULT,
                               x1*TURN_MULT,
                               TurnType.TURN_SWERVE_SIDE_TURN);
            }
            else if (x2 == 0) {
                drive.setTurn(TurnType.TURN_SWERVE_SIDE_TURN2);
                drive.setPower(x2*TURN_MULT,
                               x1*TURN_MULT,
                               TurnType.TURN_SWERVE_SIDE_TURN2);
            }
            else {
                drive.setTurn(TurnType.STRAFE);
                drive.setPower(x2*0.7f,
                               x1*0.7f,
                               TurnType.STRAFE);
            }
            if (x1 == 0) {
                if (isSlow || slideLeftPosition + slideRightPosition < 600) {
                    drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                else {
                    drive.rightGradualBrake.brake(floatFactor);
                }
            }
            if (x2 == 0) {
                if (isSlow || slideLeftPosition + slideRightPosition < 600) {
                    drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                else {
                    drive.leftGradualBrake.brake(floatFactor);
                }
            }
        }
        //straight tank
        else {
        //else if ((Math.abs(y1) >= Math.abs(x1) && Math.abs(y2) >= Math.abs(x2)) ||
        //        (Math.abs(y1) >= Math.abs(x1) && y2 == 0) ||
        //        (Math.abs(y2) >= Math.abs(x2) && y1 == 0)) {
            isStrafe = false;
            if (y1 == 0 && y2 == 0) {
                drive.stop();
            }
            else if (Math.signum(y1) == -Math.signum(y2)) {
                drive.setTurn(TurnType.TURN_REGULAR);
                drive.setPower(isSlow ? y1*TURN_MULT : y1*0.6f,
                               isSlow ? y2*TURN_MULT : y2*0.6f,
                               TurnType.TURN_REGULAR);
            }
            else if (y1 == 0) {
                drive.setTurn(TurnType.TURN_SWERVE_FWD_TURN2);
                drive.setPower(y1*TURN_MULT,
                               y2*TURN_MULT,
                               TurnType.TURN_SWERVE_FWD_TURN2);
            }
            else if (y2 == 0) {
                drive.setTurn(TurnType.TURN_SWERVE_FWD_TURN);
                drive.setPower(y1*TURN_MULT,
                               y2*TURN_MULT,
                               TurnType.TURN_SWERVE_FWD_TURN);
            }
            else {
                drive.setTurn(TurnType.FORWARD);
                //prevent tip forwards
                forwordProtection.protect(y1,
                                          y2);
                backwordProtection.protect(y1,
                                           y2);
                if (forwordProtection.isPowerAdjusted()) {
                    y1 = forwordProtection.getLeftPower();
                    y2 = forwordProtection.getRightPower();
                }
                else if (backwordProtection.isPowerAdjusted()) {
                    y1 = backwordProtection.getLeftPower();
                    y2 = backwordProtection.getRightPower();
                }
                drive.setPower(y1*0.7f,
                               y2*0.7f,
                               TurnType.FORWARD);
            }
            if (y1 == 0) {
                if (isSlow || slideLeftPosition + slideRightPosition < 600) {
                    drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                else {
                    drive.leftGradualBrake.brake(floatFactor);
                }
            }
            if (y2 == 0) {
                if (isSlow || slideLeftPosition + slideRightPosition < 600) {
                    drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                else {
                    drive.rightGradualBrake.brake(floatFactor);
                }
            }
        }

        //Claw
        if (gamepad2.right_bumper && BTN_CLAW_IN.canPress(timestamp)) {
            currentClawState = clawState.IN;
        } else if (gamepad2.left_bumper && BTN_CLAW_OUT.canPress(timestamp)) {
            currentClawState = clawState.OUT;
        } else if ((gamepad2.left_trigger > 0.9) && BTN_CLAW_MID.canPress(timestamp)) {
            currentClawState = clawState.MID;
        }

        if (currentClawState == clawState.IN) {
            robot.servoClawLeft.setPosition(RobotOverchargedConstants.SERVO_CLAW_LEFT_IN);
            robot.servoClawRight.setPosition(RobotOverchargedConstants.SERVO_CLAW_RIGHT_IN);
        } else if (currentClawState == clawState.OUT) {
            robot.servoClawLeft.setPosition(RobotOverchargedConstants.SERVO_CLAW_LEFT_OUT);
            robot.servoClawRight.setPosition(SERVO_CLAW_RIGHT_OUT);
        } else if (currentClawState == clawState.MID) {
            robot.servoClawLeft.setPosition(RobotOverchargedConstants.SERVO_CLAW_LEFT_MID);
            robot.servoClawRight.setPosition(RobotOverchargedConstants.SERVO_CLAW_RIGHT_MID);
        }

        boolean isTouchSwitchSlideDown = robot.switchSlideDown.isTouch();
        if (isTouchSwitchSlideDown) {
            robot.slideLeft.resetPosition(0);
            robot.slideRight.resetPosition(0);
        }
        //slide
        int slidePosition = (robot.slideLeft.getCurrentPosition() + robot.slideRight.getCurrentPosition())/2;
        if (gamepad2.dpad_up && !robot.switchSlideUp.isTouch()) {
            robot.slideLeft.setPower(RobotConstants.POWER_SLIDE);
            robot.slideRight.setPower(RobotConstants.POWER_SLIDE);
        } else if (gamepad2.dpad_down && !isTouchSwitchSlideDown) {
            robot.slideLeft.setPower(-RobotConstants.POWER_SLIDE);
            robot.slideRight.setPower(-RobotConstants.POWER_SLIDE);
        } else if (gamepad2.dpad_right) {
            if (slidePosition > RobotOverchargedConstants.SLIDE_LEVEL_THREE + RobotOverchargedConstants.SLIDE_SPACING && !isTouchSwitchSlideDown) {
                robot.slideLeft.setPower(-RobotConstants.POWER_SLIDE);
                robot.slideRight.setPower(-RobotConstants.POWER_SLIDE);
            } else if (slidePosition < RobotOverchargedConstants.SLIDE_LEVEL_THREE - RobotOverchargedConstants.SLIDE_SPACING && !robot.switchSlideUp.isTouch()) {
                robot.slideLeft.setPower(RobotConstants.POWER_SLIDE);
                robot.slideRight.setPower(RobotConstants.POWER_SLIDE);
            } else {
                robot.slideLeft.setPower(0);
                robot.slideRight.setPower(0);
            }
        } else if (gamepad2.dpad_left) {
            if (slidePosition > RobotOverchargedConstants.SLIDE_LEVEL_FOUR + RobotOverchargedConstants.SLIDE_SPACING && !isTouchSwitchSlideDown) {
                robot.slideLeft.setPower(-RobotConstants.POWER_SLIDE);
                robot.slideRight.setPower(-RobotConstants.POWER_SLIDE);
            } else if (slidePosition < RobotOverchargedConstants.SLIDE_LEVEL_FOUR - RobotOverchargedConstants.SLIDE_SPACING && !robot.switchSlideUp.isTouch()) {
                robot.slideLeft.setPower(RobotConstants.POWER_SLIDE);
                robot.slideRight.setPower(RobotConstants.POWER_SLIDE);
            } else {
                robot.slideLeft.setPower(0);
                robot.slideRight.setPower(0);
            }
        } else {
            robot.slideLeft.setPower(0);
            robot.slideRight.setPower(0);
        }

        robot.drawLed();

        //telemetry.addData("Reverse", Boolean.toString(isReverse));
        //telemetry.update();
    }
}
