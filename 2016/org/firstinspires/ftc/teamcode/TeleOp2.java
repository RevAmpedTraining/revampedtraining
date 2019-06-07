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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Game")
public class TeleOp2
    extends OpMode {

    private Robot robot;

    private SwerveDrive drive;

    private boolean isStrafe = false;
    private boolean isSlow = false;
    private boolean isGamePad2On = false;
    private boolean isReverse = false;
    private boolean isSpinner = false;
    private boolean isTriggerOut = false;

    private ERollerStatus rollStatus = ERollerStatus.ROLL_STOP;
    private ECapBallStatus capBallStatus = ECapBallStatus.CAP_BALL_OPEN;
    private EEncoderStatus encoderStatus = EEncoderStatus.WITHOUT_ENCODER;

    private long backwardStartTime;
    private long protectModeStartTime;
    private boolean isProtectMode = false;

    private static final Button BTN_FORWARD = new Button(); //A
    private static final Button BTN_STRAFE = new Button(); //B
    private static final Button BTN_HALF_GEAR = new Button(); //RB
    private static final Button BTN_REVERSE_MODE = new Button(); //LB

    private static final Button BTN_ROLLER_IN = new Button(); //RB
    private static final Button BTN_ROLLER_OUT = new Button(); //LB
    private static final Button BTN_SPINNER = new Button(); //B
    private static final Button BTN_TRIGGER = new Button(); //X
    private static final Button BTN_CAP_BALL_HOLD = new Button(); //Dpad Up
    private static final Button BTN_CAP_BALL_PUSH = new Button(); //Dpad Down
    private static final Button BTN_SLIDE_UP = new Button(); //A
    private static final Button BTN_SLIDE_DOWN = new Button(); //Y
    private static final Button BTN_BUTTON_IN_MODE = new Button(); // Dpad right
    private static final Button BTN_BUTTON_OUT_MODE = new Button(); // Dpad left

    private final static float TURN_MULT = 0.5f;
    private final static float PROTECT_MODE_PERIOD = 1200f;
    private final static float PROTECT_MODE_POWER = 0.35f;

    public enum ERollerStatus
    {
        ROLL_IN,
        ROLL_STOP,
        ROLL_OUT;
    }

    public enum ECapBallStatus
    {
        CAP_BALL_HOLD,
        CAP_BALL_PUSH,
        CAP_BALL_OPEN;
    }

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

        if (gamepad2.right_trigger > 0.75) {
            x1 = tx1;
            y1 = ty1;
            x2 = tx2;
            y2 = ty2;
            isGamePad2On = true;
            isSlow = true;
        }
        else if (isGamePad2On) {
            isSlow = false;
            isGamePad2On = false;
        }

        x1 = Button.scaleInput(x1);
        x2 = Button.scaleInput(x2);
        y1 = Button.scaleInput(y1);
        y2 = Button.scaleInput(y2);

        // drive
        if (gamepad1.right_bumper && BTN_HALF_GEAR.canPress(timestamp)) {
            isSlow = !isSlow;
        }
        if (isSlow) {
            if (encoderStatus == EEncoderStatus.PRE_USE_ENCODER) {
                if (robot.drive.isMode(DcMotor.RunMode.RUN_USING_ENCODER)) {
                    encoderStatus = EEncoderStatus.USE_ENCODER;
                }
                else {
                    robot.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            } else if (encoderStatus != EEncoderStatus.USE_ENCODER) {
                encoderStatus = EEncoderStatus.PRE_USE_ENCODER;
            }

            x1 *= TURN_MULT;
            x2 *= TURN_MULT;
            y1 *= TURN_MULT;
            y2 *= TURN_MULT;
            robot.ledYellow.on();
        } else {
            robot.ledYellow.off();
            if (encoderStatus == EEncoderStatus.PRE_WITHOUT_ENCODER) {
                if (robot.drive.isMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)) {
                    encoderStatus = EEncoderStatus.WITHOUT_ENCODER;
                }
                else {
                    robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            } else if (encoderStatus != EEncoderStatus.WITHOUT_ENCODER) {
                encoderStatus = EEncoderStatus.PRE_WITHOUT_ENCODER;
            }
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
                drive.setPower(isSlow ? y1*TURN_MULT : y1,
                               isSlow ? y2*TURN_MULT : y2,
                               TurnType.TURN_REGULAR);
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
                if (y1 > -PROTECT_MODE_POWER || y2 > -PROTECT_MODE_POWER) {
                    // non-backward
                    backwardStartTime = timestamp;
                    long protectDuration = timestamp - protectModeStartTime;
                    if (isProtectMode && protectDuration < PROTECT_MODE_PERIOD) {
                        if (y1 > PROTECT_MODE_POWER && y2 > PROTECT_MODE_POWER) {
                            y1 *= 0.6f*protectDuration/PROTECT_MODE_PERIOD;
                            y2 *= 0.6f*protectDuration/PROTECT_MODE_PERIOD;
                        }
                    }
                    else {
                        isProtectMode = false;
                    }
                }
                else {
                    // backwards
                    protectModeStartTime = timestamp;
                    // backward for sometime to start protection
                    isProtectMode = (timestamp - backwardStartTime > 400);
                }
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
                drive.setPower(isSlow ? x2*TURN_MULT : x2,
                               isSlow ? x1*TURN_MULT : x1,
                               TurnType.TURN_REGULAR);
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

        //roller
        if (gamepad2.right_bumper && BTN_ROLLER_IN.canPress(timestamp)) {
            if (rollStatus == ERollerStatus.ROLL_OUT || rollStatus == ERollerStatus.ROLL_STOP) {
                rollStatus = ERollerStatus.ROLL_IN;
            }
            else if (rollStatus == ERollerStatus.ROLL_IN) {
                rollStatus = ERollerStatus.ROLL_STOP;
            }
        }
        else if (gamepad2.left_bumper && BTN_ROLLER_OUT.canPress(timestamp)) {
            if(rollStatus == ERollerStatus.ROLL_IN || rollStatus == ERollerStatus.ROLL_STOP) {
                rollStatus = ERollerStatus.ROLL_OUT;
            }
            else if(rollStatus == ERollerStatus.ROLL_OUT) {
                rollStatus = ERollerStatus.ROLL_STOP;
            }
        }
        if (rollStatus == ERollerStatus.ROLL_IN) {
            robot.roller.setPower(RobotConstants.POWER_ROLLER);
        } else if (rollStatus == ERollerStatus.ROLL_OUT) {
            robot.roller.setPower(-RobotConstants.POWER_ROLLER);
        } else {
            robot.roller.setPower(0);
        }

        // spinner
        if (gamepad2.b && BTN_SPINNER.canPress(timestamp)) {
            isSpinner = !isSpinner;
        }
        if (isSpinner) {
            robot.spinnerRight.setPower(RobotConstants.POWER_SPINNER);
            robot.spinnerLeft.setPower(RobotConstants.POWER_SPINNER);
        }
        else {
            robot.spinnerRight.setPower(0);
            robot.spinnerLeft.setPower(0);
        }

        //trigger
        if (gamepad2.x && isSpinner &&
                BTN_SPINNER.isPreesable(timestamp, 1500)) {
            if (BTN_TRIGGER.canPress(timestamp,
                                     500)) {
                isTriggerOut = !isTriggerOut;
            }
        } else {
            isTriggerOut = false;
        }
        if (isTriggerOut) {
            robot.servoTrigger.setPosition(RobotConstants.SERVO_TRIGGER_OUT);
        }
        else {
            robot.servoTrigger.setPosition(RobotConstants.SERVO_TRIGGER_IN);
        }

        //slide
        if (gamepad2.y && !robot.switchSlideUp.isTouch()) {
            robot.slide.setPower(RobotConstants.POWER_SLIDE);
        } else if (gamepad2.a && !robot.switchSlideDown.isTouch()) {
            robot.slide.setPower(-RobotConstants.POWER_SLIDE);
        } else {
            robot.slide.setPower(0);
        }

        // cap ball holder
        if (gamepad2.dpad_up && BTN_CAP_BALL_HOLD.canPress(timestamp)) {
            if (capBallStatus == ECapBallStatus.CAP_BALL_PUSH || capBallStatus == ECapBallStatus.CAP_BALL_OPEN) {
                capBallStatus = ECapBallStatus.CAP_BALL_HOLD;
            }
            else if (capBallStatus == ECapBallStatus.CAP_BALL_HOLD) {
                capBallStatus = ECapBallStatus.CAP_BALL_OPEN;
            }
        }
        else if (!robot.switchSlideDown.isTouch() &&
                gamepad2.dpad_down && BTN_CAP_BALL_PUSH.canPress(timestamp)) {
            // no switch slide down is to protect servo
            if(capBallStatus == ECapBallStatus.CAP_BALL_HOLD || capBallStatus == ECapBallStatus.CAP_BALL_OPEN) {
                capBallStatus = ECapBallStatus.CAP_BALL_PUSH;
            }
            else if(capBallStatus == ECapBallStatus.CAP_BALL_PUSH) {
                capBallStatus = ECapBallStatus.CAP_BALL_OPEN;
            }
        }
        if (capBallStatus == ECapBallStatus.CAP_BALL_HOLD) {
            robot.servoCapBall.setPosition(RobotConstants.SERVO_CAP_BALL_HOLD);
            robot.ledRed.on();
        } else {
            if (capBallStatus == ECapBallStatus.CAP_BALL_PUSH) {
                robot.ledRed.blink();
                if (robot.switchSlideDown.isTouch()) {
                    // this is to protect servo
                    capBallStatus = ECapBallStatus.CAP_BALL_OPEN;
                }
                else {
                    robot.servoCapBall.setPosition(RobotConstants.SERVO_CAP_BALL_PUSH);
                }
            } else {
                robot.servoCapBall.setPosition(RobotConstants.SERVO_CAP_BALL_OPEN);
                robot.ledRed.off();
            }
        }

        //trigger
        if (gamepad2.dpad_left) {
            robot.servoBeacon.setPosition(RobotConstants.SERVO_BEACON_OUT);
        }
        else if (gamepad2.dpad_right) {
            robot.servoBeacon.setPosition(RobotConstants.SERVO_BEACON_IN);
        }
        else {
            robot.servoBeacon.setPosition(RobotConstants.SERVO_BEACON_STOP);
        }

        // sonar sensor
        float distance = robot.sonarFront.getDistance();
        if (75 <= distance && distance <= 87) {
            robot.ledGreen.on();
        }
        else {
            robot.ledGreen.off();
        }

        robot.drawLed();

        //telemetry.addData("Reverse", Boolean.toString(isReverse));
        //telemetry.update();
    }
}
