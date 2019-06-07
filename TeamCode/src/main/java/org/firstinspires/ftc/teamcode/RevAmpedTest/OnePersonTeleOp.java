package org.firstinspires.ftc.teamcode.RevAmpedTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.revAmped.components.Button;
import com.revAmped.components.HwLed;
import com.revAmped.components.RobotRevAmped;
import com.revAmped.components.RobotRevampedTest;
import com.revAmped.components.SwerveDrive;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.config.SwerveDriveConstants;
import com.revAmped.linear.util.Wait;
import com.revAmped.util.TipOverProtection;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RevAmpedGame.SwerveDriveTeleOp;

/**
 * Created by John Wang on 3/25/2019.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="One Person TeleOp", group="Test")
public class OnePersonTeleOp
        extends OpMode {

    private RobotRevampedTest robot;

    private SwerveDrive drive;

    //Boolean values
    private Boolean isSlow = false;
    private Boolean isDead = false;
    boolean ENCreset  = true;
    boolean p_reset = true;
    boolean encoder_bottom = false;
    boolean autoHang = false;
    boolean goUp = false;
    boolean isBottom = false;
    int newTarget;
    int startPopper;

    int topHangPos = 8800;
    int lowHangPos = 3500;

    int count_per_turn = 1120; // this is Count per turn
    double degree_target_abs = 360; // this is Abs dress target  final position shouls be inside 300-359.99
    double count_current = 0; // this is Abs degree target + N* turn
    double count_to_go = 0;

    //Slow Mode value
    private static final float TURN_MULT = 0.4f;
    //joystick deadzone
    private static final float DEADZONE_SNAKE = 0.1f;

    public static final float DEADZONE_TANK = 0.2f;
    //slow mode deadzone
    private static final float SLOWDZ = 0.03f;

    //Buttons
    /**         Controls/Function                           Button
     *          RETRACT TELESCOPING SLIDE                   GP 1 TRIGGERS
     *          LATCH SLIDE UP                              GP 1 Dpad UP
     *          LATCH SLIDE DOWN                            GP 1 Dpad DOWN
     *          INTAKE SLIDES                               GP 1 Dpad LEFT/RIGHT
     *          INTAKE                                      GP 1 BUMPERS
     *          POPPER                                      GP 1 A
     *          DUMP                                        GP 1 B
     *          SLOW MODE                                   GP 1 Y
     *          STRAFE SHIFT
     */
    private static final Button SLOW_MODE = new Button();
    private static final Button AUTO_HANG = new Button();
    private static final Button INTAKE_FORWARD = new Button();
    private static final Button INTAKE_REVERSE = new Button();
    private static final Button BTN_STRAFE = new Button();
    private static final Button BTN_FORWARD = new Button();
    private static final Button SERVO_DUMP = new Button();
    private static final Button TANK_SNAKE = new Button();

    //Enums for servo states
    private enum IntakeState {
        FORWARD,
        REVERSE,
        STOP
    }

    private enum TelescopeState {
        FORWARD,
        REVERSE,
        STOP
    }

    private enum DumpState {
        DUMP,
        HOLD
    }

    private IntakeState currentIntakeState = IntakeState.STOP;

    private TelescopeState currentState = TelescopeState.STOP;

    private DumpState currentDumpState = DumpState.HOLD;

    private long startTime;

    @Override
    public void init() {
        robot = new RobotRevampedTest(this, false);
        drive = robot.getSwerveDrive();
        this.gamepad1.reset();
        gamepad1.setJoystickDeadzone(0.1f);
        this.gamepad2.reset();
        gamepad2.setJoystickDeadzone(0.1f);
        if (ENCreset) { // only rest once

            robot.motorPopper.setPower(-0.15f);  // -0.o8 not move
            try {
                Thread.sleep(1500);  // need run >2 sec for worst case to get to the point
            } catch (Exception e) {}
            robot.motorPopper.setPower(0);
            robot.motorPopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ENCreset = !ENCreset;
        }
        startPopper = robot.motorPopper.getCurrentPosition();
    }
    @Override
    public void init_loop() {
        // If you are using Motorola E4 phones,
        // you should send telemetry data while waiting for start.
        telemetry.addData("status", "loop test... waiting for start");
        telemetry.update();
        startPopper = robot.motorPopper.getCurrentPosition();
        startTime = System.currentTimeMillis();
    }
    @Override
    public void loop() {
        long timestamp1 = System.currentTimeMillis() - startTime;
        long timestamp = System.currentTimeMillis();
        //telemetry.addData("Time Left", timestamp1-startTime);
        // y axis is reversed by default
        //if auto hanging, skip the rest of the tasks and focus solely on hanging
    if (!autoHang) {
        float x1 = Range.clip(Button.scaleInput(gamepad1.left_stick_x), -1f, 1f);
        float y1 = Range.clip(Button.scaleInput(-gamepad1.left_stick_y), -1f, 1f);
        float x2 = Range.clip(Button.scaleInput(gamepad1.right_stick_x), -1f, 1f);
        float y2 = Range.clip(Button.scaleInput(-gamepad1.right_stick_y), -1f, 1f);
        if (timestamp1 > 60000) {
            robot.ledBlue.on();
            telemetry.addLine("ONE MINUTE LEFT");
            if (timestamp1 > 110000) {
                robot.ledBlue.blink();
                telemetry.addLine("10 SECONDS LEFT");
            }
        } else {
            robot.ledBlue.off();
        }
        //Slow Mode
        if (gamepad1.y && SLOW_MODE.canPress(timestamp)) {
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
        if (x1 == 0 && x2 == 0 && y1 == 0 && y2 == 0) {
            isDead = true;
        } else {
            isDead = false;
        }
        if (isDead) {
            drive.stop();
        } else {
            if ((Math.abs(y1) > DEADZONE_TANK && Math.abs(y2) > DEADZONE_TANK) &&
                    Math.signum(y1) == Math.signum(y2)
                    && (Math.abs(x1) < DEADZONE_TANK && Math.abs(x2) < DEADZONE_TANK)) {
                robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_START);
                robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_START);
                robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_START);
                robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_START);
                drive.setPower(-y1, -y2, TurnType.FORWARD);
            } else if ((Math.abs(y1) < DEADZONE_TANK && Math.abs(y2) < DEADZONE_TANK) &&
                    Math.signum(x1) == Math.signum(x2)
                    && (Math.abs(x1) > DEADZONE_TANK && Math.abs(x2) > DEADZONE_TANK)) {
                robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_END);
                robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_END);
                robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_END);
                robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_END);
                drive.setPower(x1, -x2, TurnType.STRAFE);
            } else if (((y1 > .15f && y2 < -.15f) || (y1 < -.15f && y2 > 0.15f))
                    && Math.abs(x1) < 0.2f && Math.abs(x2) < 0.2f) {
                robot.servoRightBack.setPosition((SwerveDriveConstants.SERVO_RIGHTBACK_START +
                        SwerveDriveConstants.SERVO_RIGHTBACK_END) / 2);
                robot.servoRightFront.setPosition((SwerveDriveConstants.SERVO_RIGHTFRONT_START +
                        SwerveDriveConstants.SERVO_RIGHTFRONT_END) / 2);
                robot.servoLeftBack.setPosition((SwerveDriveConstants.SERVO_LEFTBACK_START +
                        SwerveDriveConstants.SERVO_LEFTBACK_END) / 2);
                robot.servoLeftFront.setPosition((SwerveDriveConstants.SERVO_LEFTFRONT_START +
                        SwerveDriveConstants.SERVO_LEFTFRONT_END) / 2);
                drive.setPower(y1, y2, TurnType.TURN_REGULAR);
            } else if (Math.signum(y2) != Math.signum(y1) && y1 == 0
                    && Math.abs(x1) < DEADZONE_TANK && Math.abs(x2) < DEADZONE_TANK) {
                float calibratef = 20 / 255f;
                float calibrateb = 30/255f;
                if (y2 > 0) {
                    robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABRIGHT - calibratef);
                    robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABRIGHT - calibratef);
                    robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABLEFT + calibratef);
                    robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABLEFT + calibratef);
                } else {
                    robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABRIGHT - calibrateb);
                    robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABRIGHT - calibrateb);
                    robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABLEFT + calibrateb);
                    robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABLEFT + calibrateb);
                }
                drive.setPower(-y2 / 2f, -y2/1.2f , TurnType.FORWARD);
            } else if (Math.signum(y2) != Math.signum(y1) && y2 == 0
                    && Math.abs(x1) < DEADZONE_TANK && Math.abs(x2) < DEADZONE_TANK) {
                float calibratef = 20 / 255f;
                float calibrateb = 30/255f;
                if (y1 > 0) {
                    robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABLEFT + calibratef);
                    robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABLEFT + calibratef);
                    robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABRIGHT - calibratef);
                    robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABRIGHT - calibratef);
                } else {
                    robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABLEFT + calibrateb);
                    robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABLEFT + calibrateb);
                    robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABRIGHT - calibrateb);
                    robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABRIGHT - calibrateb);
                }
                drive.setPower(-y1/1.2f, -y1 /2f, TurnType.FORWARD);
            } else {
                drive.stop();
            }

        }
        telemetry.addData("x1 values", x1);
        telemetry.addData("x2 values", x2);
        telemetry.addData("y1 values", y1);
        telemetry.addData("y2 values", y2);
        //Latch Slide
        int slideposition = robot.motorSlide.getCurrentPosition();
        int hangposition = robot.motorLatch.getCurrentPosition();
        telemetry.addData("slide position", slideposition);
        telemetry.addData("hang position", hangposition);
        //telemetry.update();
        Boolean isTouchSwitchSlideUp = robot.switchSlideUp.isTouch();
        Boolean isTouchSwitchSlideDown = robot.switchSlideDown.isTouch();
        Boolean isSwitchIn = robot.switchSlideIn.isTouch();
        if (gamepad1.dpad_up && !isTouchSwitchSlideUp) {
            robot.motorLatch.setPower(RobotRevAmpedConstants.LATCH_MOTOR);
        } else if (gamepad1.dpad_down && !isTouchSwitchSlideDown) {
            robot.motorLatch.setPower(-RobotRevAmpedConstants.LATCH_MOTOR);
        } else {
            robot.motorLatch.setPower(0);
        }
        //Horizontal Slide
        if (gamepad1.dpad_right) {
            robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
            robot.motorSlide.setPower(-1);
        } else if (gamepad1.dpad_left && !isSwitchIn) {
            robot.motorSlide.setPower(1);
            robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_OUT);
        } else {
            robot.motorSlide.setPower(RobotRevAmpedConstants.STOP);
        }
        //Intake Motor
        if (gamepad1.right_bumper && INTAKE_FORWARD.canPress(timestamp)) {
            if (currentIntakeState == IntakeState.FORWARD) {
                currentIntakeState = IntakeState.STOP;
            } else {
                currentIntakeState = IntakeState.FORWARD;
            }
        } else if (gamepad1.left_bumper && INTAKE_REVERSE.canPress(timestamp)) {
            if (currentIntakeState == IntakeState.REVERSE) {
                currentIntakeState = IntakeState.STOP;
            } else {
                currentIntakeState = IntakeState.REVERSE;
            }
        }
        if (currentIntakeState == IntakeState.FORWARD) {
            robot.motorIntake.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
        } else if (currentIntakeState == IntakeState.REVERSE) {
            robot.motorIntake.setPower(-0.6f);
        } else if (currentIntakeState == IntakeState.STOP) {
            robot.motorIntake.setPower(0);
        }
        telemetry.addData("reset", p_reset);
        //telemetry.update();
        //popper
        int position = robot.motorPopper.getCurrentPosition();
        telemetry.addData("popper position", position-startPopper);
        if (gamepad1.a) {
            telemetry.addLine("popping");
            robot.motorPopper.setPower(1f);
            telemetry.addLine("popped");
            p_reset = false;
        } else if (p_reset == false) {
            telemetry.addLine("reseting");
            robot.motorPopper.setPower(0);  // this seems importnat, other wise, it could trapped into while wait loop
            count_current = (position - startPopper) % count_per_turn;
            if (count_current > 50 && count_current < 900) {// if not in sweet point....
                int newTarget = robot.motorPopper.getTargetPosition() -  (int) count_current + 30;
                robot.motorPopper.setTargetPosition(newTarget);
                robot.motorPopper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorPopper.setPower(-0.8f);
                if (robot.motorPopper.isBusy()) {
                } else {
                    //done
                    robot.motorPopper.setPower(0);
                    robot.motorPopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    p_reset = true;
                }
                //robot.motorPopper.setPower(-0.8f);
            } else {
                //done
                robot.motorPopper.setPower(0);
                robot.motorPopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                p_reset = true;
            }
        }
        telemetry.addData("count to go", count_current);
        //Telescoping Slide
        if (gamepad1.right_trigger > 0.9f) {
            robot.servoTelescopeL.setPower(-0.8f);
            robot.servoTelescopeR.setPower(0.8f);
        } else if (gamepad1.left_trigger > 0.9f) {
            robot.servoTelescopeL.setPower(0.8f);
            robot.servoTelescopeR.setPower(-0.8f);
        } else {
            robot.servoTelescopeL.setPower(0);
            robot.servoTelescopeR.setPower(0);
        }
        //dump the blocks
        if (gamepad1.b && SERVO_DUMP.canPress(timestamp)) {
            if (currentDumpState == DumpState.HOLD) {
                currentDumpState = DumpState.DUMP;
            } else {
                currentDumpState = DumpState.HOLD;
            }
        }
        if (currentDumpState == DumpState.DUMP) {
            robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP);
            robot.ledRed.on();
            robot.ledWhite.off();
        } else if (currentDumpState == DumpState.HOLD) {
            robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP_UP);
            robot.ledWhite.on();
            robot.ledRed.off();
        } else {
            robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP_INIT);
        }
        if (gamepad1.x && AUTO_HANG.canPress(timestamp)) {
            autoHang = !autoHang;
            goUp = !goUp;
        }
    }
        if (autoHang) {
            int positionHang = robot.motorLatch.getCurrentPosition();
            if (positionHang > topHangPos) goUp = false;
            if (positionHang < topHangPos && goUp) {
                robot.motorLatch.setPower(1);
            } else if (!goUp) {
                if (positionHang > lowHangPos) {
                    robot.motorLatch.setPower(-1);
                } else {
                    robot.motorLatch.setPower(0);
                    autoHang = false;
                }
            }
            telemetry.addData("Going Up?", goUp);

        }

        telemetry.update();
        robot.drawLed();
    }

}
