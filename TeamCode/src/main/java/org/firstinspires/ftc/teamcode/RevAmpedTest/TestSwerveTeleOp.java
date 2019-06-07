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
/**
 * Created by John Wang on 9/23/2018.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TesttTeleOp", group="Game")
public class TestSwerveTeleOp
        extends OpMode {

    private RobotRevampedTest robot;

    private SwerveDrive drive;

    private int encoder_count = 0;

    private Boolean isSlow = false;
    private Boolean isDead = false;
    private Boolean isTank = true;
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
    private static final float TURN_MULT = 0.35f;
    //joystick deadzone
    private static final float DEADZONE_SNAKE = 0.1f;

    public static final float DEADZONE_TANK = 0.2f;
    //slow mode deadzone
    private static final float SLOWDZ = 0.03f;

    //Buttons
    /**         Controls/Function                           Button
     *          SLOW_MODE                                   GP 1 Right Bumper
     *          REVERSE_MODE                                GP 1 Left Bumper
     *          DROP                                        GP 2 Y
     *          SUPERSLOW_MODE                              GP 1 Y
     *          Tank Mode                                   GP 1 B
     *          RETRACT TELESCOPING SLIDE                   GP 1 X
     *          LATCH SLIDE UP                              GP 2 Dpad UP
     *          LATCH SLIDE DOWN                            GP 2 Dpad DOWN
     *          INTAKE HOLDING                              GP 2 X
     *          INTAKE_POSITION                             GP 2 B
     */
    private static final Button SLOW_MODE = new Button();
    private static final Button AUTO_HANG = new Button();
    private static final Button INTAKE_FORWARD = new Button();
    private static final Button INTAKE_REVERSE = new Button();
    private static final Button BTN_STRAFE = new Button();
    private static final Button BTN_EXIT = new Button();
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
            //manually reset the popper at the beginning of TeleOp
            robot.motorPopper.setPower(-0.15f);  // -0.o8 not move
            try {
                Thread.sleep(1000);  // need run >2 sec for worst case to gget to the point
            } catch (Exception e) {}
            robot.motorPopper.setPower(0);
            robot.motorPopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ENCreset = !ENCreset;
        }

    }
    @Override
    public void init_loop() {
        // If you are using Motorola E4 phones,
        // you should send telemetry data while waiting for start.
        telemetry.addData("status", "loop test... waiting for start");
        telemetry.update();
        startTime = System.currentTimeMillis();
    }
    @Override
    public void loop() {
        long timestamp1 = System.currentTimeMillis()-startTime;
        long timestamp = System.currentTimeMillis();
        //telemetry.addData("Time Left", timestamp1-startTime);
        // y axis is reversed by default
        if (!autoHang) {
            float x1 = gamepad1.left_stick_x;
            float y1 = -gamepad1.left_stick_y;
            float x2 = gamepad1.right_stick_x;
            float y2 = -gamepad1.right_stick_y;
            //typically not used because gamepad 1 is for driving
        /*float tx1 = gamepad2.left_stick_x;
        float ty1 = gamepad2.left_stick_y;
        float tx2 = gamepad2.right_stick_x;
        float ty2 = gamepad2.right_stick_y;*/
            //Make sure motor inputs are between -1 and 1 and scale inputs
            x1 = Range.clip(Button.scaleInput(x1), -1f, 1f);
            x2 = Range.clip(Button.scaleInput(x2), -1f, 1f);
            y1 = Range.clip(Button.scaleInput(y1), -1f, 1f);
            y2 = Range.clip(Button.scaleInput(y2), -1f, 1f);
        /*if (timestamp1>90000) {
            robot.ledBlue.on();
            telemetry.addLine("ENDGAME");
            if (timestamp1>110000) {
                robot.ledBlue.blink();
                telemetry.addLine("10 SECONDS LEFT");
            }
        } else {robot.ledBlue.off();}*/
            //Slow Mode
            if (gamepad1.right_bumper && SLOW_MODE.canPress(timestamp)) {
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
            if (gamepad1.y && TANK_SNAKE.canPress(timestamp)) {
                isTank=!isTank;
            }
            //telemetry.addData("Tank Drive?", isTank);
            if (x1==0&&x2==0&&y1==0&&y2==0) {
                isDead = true;
            } else {
                isDead = false;
            }
            if (isDead) {
                if (!isSlow) {
                    robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_START);
                    robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_START);
                    robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_START);
                    robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_START);
                }
                drive.stop();
            } else {
                if (isTank) {
                    robot.ledGreen.on();
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
                    } else {
                        drive.stop();
                    }
                } else {
                    robot.ledGreen.off();
                    if ((Math.abs(y1) > DEADZONE_TANK && Math.abs(y2) > DEADZONE_TANK) &&
                            Math.signum(y1) == Math.signum(y2)
                            && (Math.abs(x1) < DEADZONE_TANK && Math.abs(x2) < DEADZONE_TANK)) {
                        robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_START);
                        robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_START);
                        robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_START);
                        robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_START);
                        drive.setPower(-y1, -y2, TurnType.FORWARD);
                    } else if (((Math.abs(y1) < DEADZONE_TANK && Math.abs(y2) < DEADZONE_TANK) &&
                            Math.signum(x1) == Math.signum(x2)
                            && (Math.abs(x1) > DEADZONE_TANK && Math.abs(x2) > DEADZONE_TANK))
                            ) {
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
                        float calibrate = 20 / 255f;
                        float calibrateb = 30 / 255f;
                        if (y2 > 0) {
                            robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABRIGHT - calibrate);
                            robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABRIGHT - calibrate);
                            robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABLEFT + calibrate);
                            robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABLEFT + calibrate);
                        } else {
                            robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABRIGHT - calibrateb);
                            robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABRIGHT - calibrateb);
                            robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABLEFT + calibrateb);
                            robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABLEFT + calibrateb);
                        }
                        drive.setPower(-y2 / 2f, -y2/1.2f , TurnType.FORWARD);
                    }  else if (Math.signum(y2) != Math.signum(y1) && y2 == 0
                            && Math.abs(x1) < DEADZONE_TANK && Math.abs(x2) < DEADZONE_TANK) {
                        float calibrate = 20 / 255f;
                        float calibrateb = 30 / 255f;
                        if (y1 > 0) {
                            robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABLEFT + calibrate);
                            robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABLEFT + calibrate);
                            robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABRIGHT - calibrate);
                            robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABRIGHT - calibrate);
                        } else {
                            robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABLEFT + calibrateb);
                            robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABLEFT + calibrateb);
                            robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABRIGHT - calibrateb);
                            robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABRIGHT - calibrateb);
                        }
                        drive.setPower(-y1/1.2f , -y1 / 2f, TurnType.FORWARD);
                    } else {
                        drive.stop();
                    }
                }
            }
        /*telemetry.addData("x1 values", x1);
        telemetry.addData("x2 values", x2);
        telemetry.addData("y1 values", y1);
        telemetry.addData("y2 values", y2);*/
            //Other controls (gamepad 2)
            //Latch Slide
            int position = robot.motorPopper.getCurrentPosition();
        /*int slideposition = robot.motorSlide.getCurrentPosition();
        int hangposition = robot.motorLatch.getCurrentPosition();
        telemetry.addData("popper position", position);
        telemetry.addData("slide position", slideposition);
        telemetry.addData("hang position", hangposition);*/
            //telemetry.update();
            Boolean isTouchSwitchSlideUp = robot.switchSlideUp.isTouch();
            Boolean isTouchSwitchSlideDown = robot.switchSlideDown.isTouch();
            Boolean isSwitchIn = robot.switchSlideIn.isTouch();
            if (gamepad2.dpad_up && !isTouchSwitchSlideUp) {
                robot.motorLatch.setPower(RobotRevAmpedConstants.LATCH_MOTOR);
            } else if (gamepad2.dpad_down && !isTouchSwitchSlideDown) {
                robot.motorLatch.setPower(-RobotRevAmpedConstants.LATCH_MOTOR);
            } else {
                robot.motorLatch.setPower(0);
            }
            //Horizontal Slide
            if (gamepad2.dpad_right) {
                robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
                robot.motorSlide.setPower(-1);
            } else if (gamepad2.dpad_left && !isSwitchIn) {
                robot.motorSlide.setPower(1);
                robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_OUT);
            } else {
                robot.motorSlide.setPower(RobotRevAmpedConstants.STOP);
            }
            //Intake Motor
            if (gamepad2.right_bumper && INTAKE_FORWARD.canPress(timestamp)) {
                if (currentIntakeState==IntakeState.FORWARD){
                    currentIntakeState=IntakeState.STOP;
                } else {
                    currentIntakeState=IntakeState.FORWARD;
                }
            } else if (gamepad2.left_bumper && INTAKE_REVERSE.canPress(timestamp)) {
                if (currentIntakeState==IntakeState.REVERSE){
                    currentIntakeState=IntakeState.STOP;
                } else {
                    currentIntakeState=IntakeState.REVERSE;
                }
            }
            if (currentIntakeState==IntakeState.FORWARD) {
                robot.motorIntake.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
            } else if (currentIntakeState==IntakeState.REVERSE) {
                robot.motorIntake.setPower(-0.6f);
            } else if (currentIntakeState==IntakeState.STOP) {
                robot.motorIntake.setPower(0);
            }
            //telemetry.addData("reset", p_reset);
            telemetry.update();
            //popper
            telemetry.addData("reset", p_reset);
            //telemetry.update();
            //popper
            position = robot.motorPopper.getCurrentPosition();
            telemetry.addData("popper position", position-startPopper);
            if (gamepad2.a) {
                telemetry.addLine("popping");
                robot.motorPopper.setPower(0.85f);
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
            if (gamepad2.right_trigger>0.9f ) {
                robot.servoTelescopeL.setPower(-0.8f);
                robot.servoTelescopeR.setPower(0.8f);
            } else if (gamepad2.left_trigger>0.9f) {
                robot.servoTelescopeL.setPower(0.8f);
                robot.servoTelescopeR.setPower(-0.8f);
            } else {
                robot.servoTelescopeL.setPower(0);
                robot.servoTelescopeR.setPower(0);
            }
            //dump the blocks
            if (gamepad2.b && SERVO_DUMP.canPress(timestamp)){
                if (currentDumpState==DumpState.HOLD) {
                    currentDumpState=DumpState.DUMP;
                } else {
                    currentDumpState=DumpState.HOLD;
                }
            }
            if (currentDumpState==DumpState.DUMP) {
                robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP);
                robot.ledRed.on();
                robot.ledWhite.off();
            } else if (currentDumpState==DumpState.HOLD) {
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
            if (gamepad1.a && BTN_EXIT.canPress(timestamp)) {
                autoHang = false;
            }
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
            if (gamepad2.b && SERVO_DUMP.canPress(timestamp)){
                if (currentDumpState==DumpState.HOLD) {
                    currentDumpState=DumpState.DUMP;
                } else {
                    currentDumpState=DumpState.HOLD;
                }
            }
            if (currentDumpState==DumpState.DUMP) {
                robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP);
                robot.ledRed.on();
                robot.ledWhite.off();
            } else if (currentDumpState==DumpState.HOLD) {
                robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP_UP);
                robot.ledWhite.on();
                robot.ledRed.off();
            } else {
                robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP_INIT);
            }
            if (gamepad2.a) {
                robot.motorPopper.setPower(1);
            } else {
                robot.motorPopper.setPower(0);
            }
        }
        //telemetry.update();
        robot.drawLed();
    }
}
