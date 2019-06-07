package org.firstinspires.ftc.teamcode.RevAmpedGame;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.revAmped.components.Button;
import com.revAmped.config.RobotConstants;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.linear.util.Wait;
import com.revAmped.util.TipOverProtection;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.revAmped.components.RobotRevAmped2;
import com.revAmped.components.MecanumDrive;

import java.text.DecimalFormat;

import static com.revAmped.config.RobotRevAmpedConstants.POWER_SWEEPER;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_CONTAINER_DOWN;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_CONTAINER_FLAT;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_CONTAINER_UP;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_DOOR_IN;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_DOOR_OUT;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_DUMPER_CLAW_MAX_OUT;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT;


/**
 * TeleOp Chezy Test for a 6 wheel bas (works on mecanum)
 * Program for Revamped 12808
 * Created 6/20/2018
 */
//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Chezy Drive TeleOp", group="Game")
public class ChezyDriveTeleOpMecanum
        extends OpMode {

    private RobotRevAmped2 robot;

    private MecanumDrive drive;

    private float pwrTurn, pwrDrive;
    private float tRight, tLeft;

    private boolean isSlow = false;

    private ElapsedTime runtime = new ElapsedTime();
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");

    private TipOverProtection forwordProtection = new TipOverProtection(true);
    private TipOverProtection backwordProtection = new TipOverProtection(false);

    private static final Button BTN_HALF_GEAR = new Button();//g1 Y
    private static final Button BTN_TURN_RIGHT = new Button();//g1 r_t
    private static final Button BTN_TURN_LEFT = new Button();//g1 l_t
    private static final Button BTN_STRAFE_RIGHT = new Button();//g1 r_b
    private static final Button BTN_STRAFE_LEFT = new Button();//g1 l_b
    private static final Button BTN_BALANCE = new Button();//g1 A
    private static final Button BTN_RELIC_DOWN = new Button();//g1 X
    private static final Button BTN_FIX = new Button();//g1 B

    private static final Button BTN_CONTAINER_UP = new Button(); //g2 Y
    private static final Button BTN_CONTAINER_DOWN = new Button(); //g2 A
    private static final Button BTN_DOOR = new Button(); //g2 B
    private static final Button BTN_ELBOW = new Button(); //g2 X
    private static final Button BTN_CLAW = new Button(); //g2 l_b
    private static final Button BTN_ROLLER_IN = new Button(); //g2 r_b
    private static final Button BTN_ROLLER_OUT = new Button(); //g2 l_b

    private boolean isDoorOpen = true;

    private enum ContainerState {
        DOWN,
        // waiting for door to close
        FLATWAIT,
        // waiting for claw to close
        FLATWAITCLAW,
        FLAT,
        // waiting for claw to open
        UPWAIT,
        UP;
    }

    private ChezyDriveTeleOpMecanum.ContainerState currentContainerState =
            ChezyDriveTeleOpMecanum.ContainerState.DOWN;
    private long timeDoorWait;
    private enum SweeperState {
        IN,
        OUT,
        FIX,
        STOP;
    }

    private enum FixState {
        START,
        FIN,
        FOUT;
    }

    private ChezyDriveTeleOpMecanum.RelicElbowState relicElbowState = ChezyDriveTeleOpMecanum.RelicElbowState.REST;
    private enum RelicElbowState {
        UP,
        DOWN,
        REST;
    }

    private ChezyDriveTeleOpMecanum.RelicClawState relicClawState = ChezyDriveTeleOpMecanum.RelicClawState.REST;
    private enum RelicClawState {
        WIDEOPEN,
        OPEN,
        DROP,
        GRAB,
        REST;
    }

    private ChezyDriveTeleOpMecanum.SweeperState currentSweeperState = ChezyDriveTeleOpMecanum.SweeperState.STOP;
    private ChezyDriveTeleOpMecanum.SweeperState previousSweeperState = currentSweeperState;

    private ChezyDriveTeleOpMecanum.FixState fixState = ChezyDriveTeleOpMecanum.FixState.START;
    private long fixTimeStamp;
    @Override
    public void init() {
        robot = new RobotRevAmped2(this,
                false);
        drive = robot.getMecanumDrive();
        robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_IN);
        this.gamepad1.reset();
        this.gamepad1.setJoystickDeadzone(0.1f);
        this.gamepad2.reset();
        this.gamepad2.setJoystickDeadzone(0.1f);
        runtime.reset();
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
        float y2 = -gamepad1.right_stick_y;
        telemetry.addLine("turn speed" + x1);
        telemetry.addLine("drive speed" + y2);
        telemetry.addLine("Time Elapsed:" + runtime.toString());
        //power for turn based on left joystick, power for drive based on right joystick
        pwrTurn = x1;//Button.scaleInput(x1);
        pwrDrive = y2;//Button.scaleInput(y2);
        //powers must be between -1 and 1
        pwrTurn = Range.clip(pwrTurn, -1f, 1f);
        pwrDrive = Range.clip(pwrDrive, -1f, 1f);
        //all speeds multiplied by this factor
        float SPEED_FACTOR = 0.85f;
        //drive code-turn radius dictated by left joystick, driving speed by right joystick
        //only drive if the quickstrafing buttons are
        if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
            if (pwrTurn < 0.1f && pwrTurn > -0.1f) {
                drive.setPower(pwrDrive * SPEED_FACTOR,
                        pwrDrive * SPEED_FACTOR);
            } else if (pwrTurn > 0.3f) {
                drive.setPower((pwrDrive + pwrTurn * pwrDrive)/2 * SPEED_FACTOR,
                        (pwrDrive - pwrTurn * pwrDrive)/2 * SPEED_FACTOR);
            } else if (pwrTurn < -0.3f) {
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
        //quickturn buttons for strafing
        if (gamepad1.left_bumper && BTN_STRAFE_LEFT.canPress(timestamp)) {
            drive.setStrafePower(-0.85f);

        } else if (gamepad1.right_bumper && BTN_STRAFE_RIGHT.canPress(timestamp)) {
            drive.setStrafePower(0.85f);
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

        boolean isTouchSwitchSlideDown = robot.switchSlideDown.isTouch();
        boolean isTouchSwitchSlideUp = robot.switchSlideUp.isTouch();
        if (isTouchSwitchSlideDown) {
            robot.motorSlide.resetPosition(0);
        }
        //operations for dumping slide
        if (currentContainerState == ChezyDriveTeleOpMecanum.ContainerState.FLAT ||
                currentContainerState == ChezyDriveTeleOpMecanum.ContainerState.UP) {
            //int slidePosition = robot.motorSlide.getCurrentPosition();
            if (gamepad2.dpad_up && !isTouchSwitchSlideUp) {
                robot.motorSlide.setPower(RobotConstants.POWER_SLIDE);
            } else if (gamepad2.dpad_down && !isTouchSwitchSlideDown) {
                robot.motorSlide.setPower(-RobotConstants.POWER_SLIDE);
            } else {
                robot.motorSlide.setPower(0);
            }
        }
        else {
            robot.motorSlide.setPower(0);
        }

        //operation for returning tray to rest position
        if (gamepad2.a && BTN_CONTAINER_DOWN.canPress(timestamp)) {
            currentContainerState = ChezyDriveTeleOpMecanum.ContainerState.DOWN;
        }
        else if (currentContainerState == ChezyDriveTeleOpMecanum.ContainerState.FLATWAIT) {
            // waiting for door to close
            if (System.currentTimeMillis() - timeDoorWait > 750) {
                currentContainerState = ChezyDriveTeleOpMecanum.ContainerState.FLAT;
            }
        }
        else if (currentContainerState == ChezyDriveTeleOpMecanum.ContainerState.FLATWAITCLAW) {
            // waiting for door to close
            if (System.currentTimeMillis() - timeDoorWait > 250) {
                currentContainerState = ChezyDriveTeleOpMecanum.ContainerState.FLAT;
            }
        }
        else if (currentContainerState == ChezyDriveTeleOpMecanum.ContainerState.UPWAIT) {
            // waiting for door to open
            if (System.currentTimeMillis() - timeDoorWait > 250) {
                currentContainerState = ChezyDriveTeleOpMecanum.ContainerState.UP;
            }
        }
        //operation for raising glyphs to flat or dumping glyphs
        else if (gamepad2.y && BTN_CONTAINER_UP.canPress(timestamp)) {
            switch (currentContainerState) {
                case FLAT:
                    currentContainerState = ChezyDriveTeleOpMecanum.ContainerState.UPWAIT;
                    // open door
                    isDoorOpen = true;
                    timeDoorWait = timestamp;
                    break;
                case UP:
                    currentContainerState = ChezyDriveTeleOpMecanum.ContainerState.FLAT;
                    break;
                case DOWN:
                    if (isDoorOpen) {
                        currentContainerState = ChezyDriveTeleOpMecanum.ContainerState.FLATWAIT;
                        timeDoorWait = timestamp;
                        isDoorOpen = false;
                    }
                    else {
                        currentContainerState = ChezyDriveTeleOpMecanum.ContainerState.FLATWAITCLAW;
                        timeDoorWait = timestamp;
                        currentContainerState = ChezyDriveTeleOpMecanum.ContainerState.FLAT;
                    }
                    break;
            }
        }
        switch (currentContainerState) {
            case UP:
                robot.servoContainer.setPosition(SERVO_CONTAINER_UP);
                robot.servoDumperClaw.setPosition(SERVO_DUMPER_CLAW_MAX_OUT);
                break;
            case UPWAIT:
                robot.servoContainer.setPosition(SERVO_CONTAINER_FLAT);
                robot.servoDumperClaw.setPosition(SERVO_DUMPER_CLAW_MAX_OUT);
                break;
            case FLAT:
                robot.servoContainer.setPosition(SERVO_CONTAINER_FLAT);
                robot.servoDumperClaw.setPosition(SERVO_DUMPER_CLAW_IN);
                break;
            case FLATWAIT:
            case FLATWAITCLAW:
                robot.servoContainer.setPosition(SERVO_CONTAINER_DOWN);
                robot.servoDumperClaw.setPosition(SERVO_DUMPER_CLAW_IN);
                break;
            case DOWN:
                robot.servoContainer.setPosition(SERVO_CONTAINER_DOWN);
                robot.servoDumperClaw.setPosition(SERVO_DUMPER_CLAW_OUT);
                break;
        }
        //solution to jammed glyphs by jittering the tray
        if ((gamepad1.b) && BTN_FIX.canPress(timestamp)) {
            isDoorOpen = true;
            for (int i=0; i<10; i++) {
                if (i%2==0) {
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                } else {
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                }
                try {
                    Wait lw = new Wait(null);
                    lw.waitMillis(100);
                } catch (InterruptedException e) {}
            }
        }
        if ((gamepad2.right_trigger > 0.9) && BTN_ROLLER_IN.canPress(timestamp)) {
            if (currentSweeperState == ChezyDriveTeleOpMecanum.SweeperState.IN) {
                currentSweeperState = ChezyDriveTeleOpMecanum.SweeperState.STOP;
            }
            else {
                currentSweeperState = ChezyDriveTeleOpMecanum.SweeperState.IN;
            }
        }
        else if ((gamepad2.left_trigger > 0.9) && BTN_ROLLER_OUT.canPress(timestamp)) {
            if (currentSweeperState == ChezyDriveTeleOpMecanum.SweeperState.OUT) {
                currentSweeperState = ChezyDriveTeleOpMecanum.SweeperState.STOP;
            }
            else {
                currentSweeperState = ChezyDriveTeleOpMecanum.SweeperState.OUT;
            }
        } else if (gamepad2.right_bumper) {
            if (currentSweeperState != ChezyDriveTeleOpMecanum.SweeperState.FIX) {
                previousSweeperState = currentSweeperState;
            }
            currentSweeperState = ChezyDriveTeleOpMecanum.SweeperState.FIX;
        } else {
            currentSweeperState = previousSweeperState;
        }

        if (currentSweeperState != ChezyDriveTeleOpMecanum.SweeperState.FIX) {
            previousSweeperState = currentSweeperState;
            fixState = ChezyDriveTeleOpMecanum.FixState.START;
        }
        //operations for door
        if (gamepad2.b && BTN_DOOR.canPress(timestamp)) {
            isDoorOpen = !isDoorOpen;
        }
        robot.servoDoorRight.setPosition(isDoorOpen ? SERVO_DOOR_OUT : SERVO_DOOR_IN);
        //close the intake if the ray is flat or up
        if (currentContainerState == ChezyDriveTeleOpMecanum.ContainerState.FLAT ||
                currentContainerState == ChezyDriveTeleOpMecanum.ContainerState.UP) {
            robot.sweeperLeft.setPower(0);
            robot.sweeperRight.setPower(0);
        }
        else if (isDoorOpen || timestamp - BTN_DOOR.lastPressTime < 6000) {
            if (currentSweeperState== ChezyDriveTeleOpMecanum.SweeperState.IN) {
                robot.sweeperLeft.setPower(POWER_SWEEPER);
                robot.sweeperRight.setPower(POWER_SWEEPER);
            } else if (currentSweeperState== ChezyDriveTeleOpMecanum.SweeperState.OUT) {
                robot.sweeperLeft.setPower(-POWER_SWEEPER);
                robot.sweeperRight.setPower(-POWER_SWEEPER);
            }
            else if (currentSweeperState == ChezyDriveTeleOpMecanum.SweeperState.FIX) {
                switch (fixState) {
                    case START:
                        fixTimeStamp = System.currentTimeMillis();
                        fixState = ChezyDriveTeleOpMecanum.FixState.FIN;
                        break;
                    case FIN:
                        robot.sweeperLeft.setPower(POWER_SWEEPER);
                        robot.sweeperRight.setPower(POWER_SWEEPER);
                        if (System.currentTimeMillis() - fixTimeStamp > 350) {
                            fixState = ChezyDriveTeleOpMecanum.FixState.FOUT;
                            fixTimeStamp = System.currentTimeMillis();
                        }
                        break;
                    case FOUT:
                        robot.sweeperLeft.setPower(-POWER_SWEEPER);
                        robot.sweeperRight.setPower(-POWER_SWEEPER);
                        if (System.currentTimeMillis() - fixTimeStamp > 350) {
                            fixState = ChezyDriveTeleOpMecanum.FixState.START;
                        }
                        break;
                }
            }
            else {
                robot.sweeperLeft.setPower(0);
                robot.sweeperRight.setPower(0);
            }
        }
        else {
            robot.sweeperLeft.setPower(0);
            robot.sweeperRight.setPower(0);
        }
        //putting the relic back down after scoring and in prep for balance
        if (gamepad1.x && BTN_RELIC_DOWN.canPress(timestamp)) {
            relicElbowState = ChezyDriveTeleOpMecanum.RelicElbowState.REST;
        }

        //relic elbow operations
        if (gamepad2.x && BTN_ELBOW.canPress(timestamp)) {
            switch (relicElbowState) {
                case REST:
                case DOWN:
                    relicElbowState = ChezyDriveTeleOpMecanum.RelicElbowState.UP;
                    break;
                case UP:
                    relicElbowState = ChezyDriveTeleOpMecanum.RelicElbowState.DOWN;
                    break;
                default:
                    relicElbowState = ChezyDriveTeleOpMecanum.RelicElbowState.UP;
                    break;

            }
        }
        switch (relicElbowState) {
            case REST:
                robot.servoRelicElbow.setPosition(RobotRevAmpedConstants.SERVO_RELIC_ELBOW_REST);
                break;
            case DOWN:
                robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_IN+40f/255f);
                robot.servoRelicElbow.setPosition(RobotRevAmpedConstants.SERVO_RELIC_ELBOW_DOWN);
                break;
            case UP:
                robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_IN+40f/255f);
                robot.servoRelicElbow.setPosition(RobotRevAmpedConstants.SERVO_RELIC_ELBOW_UP);
                break;
        }
        //relic claw operations
        if (gamepad2.left_bumper && BTN_CLAW.canPress(timestamp)) {
            switch (relicClawState) {
                case REST:
                    if (relicElbowState != ChezyDriveTeleOpMecanum.RelicElbowState.REST) {
                        relicClawState = ChezyDriveTeleOpMecanum.RelicClawState.OPEN;
                    }
                    break;
                case OPEN:
                    relicClawState = ChezyDriveTeleOpMecanum.RelicClawState.GRAB;
                    break;
                case GRAB:
                    relicClawState = ChezyDriveTeleOpMecanum.RelicClawState.DROP;
                    break;
                case DROP:
                    relicClawState = ChezyDriveTeleOpMecanum.RelicClawState.WIDEOPEN;
                    break;
                case WIDEOPEN:
                    relicClawState = ChezyDriveTeleOpMecanum.RelicClawState.OPEN;
                    break;
            }
        }

        switch (relicClawState) {
            case REST:
                robot.servoRelicClaw.setPosition(RobotRevAmpedConstants.SERVO_RELIC_CLAW_REST);
                break;
            case GRAB:
                robot.servoRelicClaw.setPosition(RobotRevAmpedConstants.SERVO_RELIC_CLAW_GRAB);
                break;
            case DROP:
                robot.servoRelicClaw.setPosition(RobotRevAmpedConstants.SERVO_RELIC_CLAW_DROP);
                break;
            case WIDEOPEN:
                robot.servoRelicClaw.setPosition(RobotRevAmpedConstants.SERVO_RELIC_CLAW_WIDEOPEN);
                break;
            case OPEN:
                robot.servoRelicClaw.setPosition(RobotRevAmpedConstants.SERVO_RELIC_CLAW_OPEN);
                break;
        }
        //relic slide operation
        boolean isTouchSwitchRelicSlideIn = robot.switchRelicSlideIn.isTouch();
        if (isTouchSwitchRelicSlideIn) {
            robot.motorRelicSlide.resetPosition(0);
        }
        if (relicElbowState == ChezyDriveTeleOpMecanum.RelicElbowState.DOWN ||
                relicElbowState == ChezyDriveTeleOpMecanum.RelicElbowState.UP) {
            if (gamepad2.dpad_left) {
                robot.motorRelicSlide.setPower(RobotConstants.POWER_SLIDE);
            } else if (gamepad2.dpad_right && !isTouchSwitchRelicSlideIn) {
                robot.motorRelicSlide.setPower(-RobotConstants.POWER_SLIDE);
            } else {
                robot.motorRelicSlide.setPower(0);
            }
        }
        else {
            robot.motorRelicSlide.setPower(0);
        }

        //automatic balancing
        if (gamepad1.a && BTN_BALANCE.canPress(timestamp)) {
            VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
            double voltage = voltageSensor.getVoltage();
            if (voltage > 13.2 ) {
                drive.setPower(-0.3f, -0.3f, -0.3f, -0.3f);
                //============== 3/18/2018: try to use LinearWait ========
                try {
                    Wait lw = new Wait(null);
                    lw.waitMillis(1560);
                } catch (InterruptedException e) {
                }
                //========================================================
            } else if (voltage > 12.2 && voltage < 13.2){
                drive.setPower(-0.3f, -0.3f, -0.3f, -0.3f);
                try {
                    Wait lw = new Wait(null);
                    lw.waitMillis(1580);
                } catch (InterruptedException e) {
                }
            } else {
                drive.setPower(-0.3f, -0.3f, -0.3f, -0.3f);
                try {
                    Wait lw = new Wait(null);
                    lw.waitMillis(1580+Math.round((int) (12.2-voltage)*100));
                } catch (InterruptedException e) {
                }
            }
            drive.stop();
            try {
                Wait lw = new Wait(null);
                lw.waitMillis(200);
            }
            catch(InterruptedException e){}
        }

        robot.drawLed();
        telemetry.update();
    }
}
