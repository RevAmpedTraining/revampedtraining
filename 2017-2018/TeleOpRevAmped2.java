package org.firstinspires.ftc.teamcode.RevAmpedGame;


import com.qualcomm.robotcore.robot.Robot;
import com.revAmped.components.Button;
import com.revAmped.components.MecanumDrive;
import com.revAmped.components.RobotRevAmped2;
import com.revAmped.config.RobotConstants;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.linear.util.Wait;
import com.revAmped.util.GlyphDetector;
import com.revAmped.util.TipOverProtection;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;


import static com.revAmped.config.RobotRevAmpedConstants.POWER_SWEEPER;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_CONTAINER_DOWN;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_CONTAINER_FLAT;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_CONTAINER_UP;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_DOOR_IN;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_DOOR_OUT;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_DUMPER_CLAW_MAX_OUT;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_STICK_IN;
import static com.revAmped.config.RobotRevAmpedConstants.SERVO_STICK_UP;


/**
 * TeleOp Refactored
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp RevAmped", group="Game")
public class TeleOpRevAmped2
    extends OpMode {

    private RobotRevAmped2 robot;

    private MecanumDrive drive;

    private GlyphDetector jamDetection;

    private boolean isSlow = false;
    private boolean isReverse = false;

    private final static DecimalFormat numberFormatter = new DecimalFormat("######");

    private TipOverProtection forwordProtection = new TipOverProtection(true);
    private TipOverProtection backwordProtection = new TipOverProtection(false);

    private static final Button BTN_HALF_GEAR = new Button();
    private static final Button BTN_REVERSE_MODE = new Button();
    private static final Button BTN_CONTAINETR_UP = new Button(); //Y
    private static final Button BTN_CONTAINETR_DOWN = new Button(); //A

    private static final Button BTN_DOOR = new Button(); //B
    private static final Button BTN_ELBOW = new Button();
    private static final Button BTN_CLAW = new Button();
    private static final Button BTN_ROLLER_IN = new Button(); //RB2
    private static final Button BTN_ROLLER_OUT = new Button(); //LB2
    private static final Button BTN_BALANCE = new Button();
    private static final Button BTN_ENCODER_BALANCE = new Button();
    private static final Button BTN_RELIC_DOWN = new Button();
    private static final Button BTN_FIX = new Button();
    private final static float TURN_MULT = 0.3f;
    private boolean isDoorOpen = true;
    //private boolean isStickDown = false;

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

    private ContainerState currentContainerState = ContainerState.DOWN;
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

    private RelicElbowState relicElbowState = RelicElbowState.REST;
    private enum RelicElbowState {
        UP,
        DOWN,
        REST;
    }

    private RelicClawState relicClawState = RelicClawState.REST;
    private enum RelicClawState {
        WIDEOPEN,
        OPEN,
        DROP,
        GRAB,
        REST;
    }

    private SweeperState currentSweeperState = SweeperState.STOP;
    private SweeperState previousSweeperState = currentSweeperState;

    private FixState fixState = FixState.START;
    private long fixTimeStamp;

    @Override
    public void init() {
        robot = new RobotRevAmped2(this,
                                   false);
        drive = robot.getMecanumDrive();
        robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_IN);
        this.gamepad1.reset();
        this.gamepad1.setJoystickDeadzone(0.15f);
        this.gamepad2.reset();
        this.gamepad2.setJoystickDeadzone(0.15f);
        jamDetection = new GlyphDetector(this.hardwareMap, "sensor_front", "sensor_back", "sonar_rf");
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

        x1 = Button.scaleInput(x1);
        x2 = Button.scaleInput(x2);
        y1 = Button.scaleInput(y1);
        y2 = Button.scaleInput(y2);

        // pwrLeft is y1
        // pwrRight is y2

        // drive
        //slow mode
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
        //reverse mode
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

        int slideRightPosition = robot.motorSlide.getCurrentPosition();
        // factor for floating motors
        float floatFactor = 4f*Math.abs(slideRightPosition)/RobotRevAmpedConstants.SLIDE_ENCODER_MAX;
        if (isSlow) {
            floatFactor *= TURN_MULT;
        }


        if ((Math.abs(y1) < Math.abs(x1) && Math.abs(y2) < Math.abs(x2)) ||
                (Math.abs(y1) < Math.abs(x1) && x2 == 0) ||
                (Math.abs(y2) < Math.abs(x2) && x1 == 0)) {

            //drive.setStrafePower(x2,
            //                     x1);
            float avg = (x1 + x2)/2f;
            drive.setStrafePower(avg,
                                 avg);
        }
        //straight tank
        else {
            //else if ((Math.abs(y1) >= Math.abs(x1) && Math.abs(y2) >= Math.abs(x2)) ||
            //        (Math.abs(y1) >= Math.abs(x1) && y2 == 0) ||
            //        (Math.abs(y2) >= Math.abs(x2) && y1 == 0)) {
                drive.setPower(y1,
                               y2);
        }

        //robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_UP);
        boolean isTouchSwitchSlideDown = robot.switchSlideDown.isTouch();
        boolean isTouchSwitchSlideUp = robot.switchSlideUp.isTouch();
        if (isTouchSwitchSlideDown) {
            robot.motorSlide.resetPosition(0);
        }
        //slide
        if (currentContainerState == ContainerState.FLAT ||
                currentContainerState == ContainerState.UP) {
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
        //level buttons

        if (gamepad2.a && BTN_CONTAINETR_DOWN.canPress(timestamp)) {
            currentContainerState = ContainerState.DOWN;
        }
        else if (currentContainerState == ContainerState.FLATWAIT) {
            // waiting for door to close
            if (System.currentTimeMillis() - timeDoorWait > 750) {
                currentContainerState = ContainerState.FLAT;
            }
        }
        else if (currentContainerState == ContainerState.FLATWAITCLAW) {
            // waiting for door to close
            if (System.currentTimeMillis() - timeDoorWait > 250) {
                currentContainerState = ContainerState.FLAT;
            }
        }
        else if (currentContainerState == ContainerState.UPWAIT) {
            // waiting for door to open
            if (System.currentTimeMillis() - timeDoorWait > 250) {
                currentContainerState = ContainerState.UP;
            }
        }

        else if (gamepad2.y && BTN_CONTAINETR_UP.canPress(timestamp)) {
            switch (currentContainerState) {
                case FLAT:
                    currentContainerState = ContainerState.UPWAIT;
                    // open door
                    isDoorOpen = true;
                    timeDoorWait = timestamp;
                    break;
                case UP:
                    currentContainerState = ContainerState.FLAT;
                    break;
                case DOWN:
                    if (isDoorOpen) {
                        currentContainerState = ContainerState.FLATWAIT;
                        timeDoorWait = timestamp;
                        isDoorOpen = false;
                    }
                    else {
                        currentContainerState = ContainerState.FLATWAITCLAW;
                        timeDoorWait = timestamp;
                        currentContainerState = ContainerState.FLAT;
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
        //robot.servoStick.setPosition(isStickUp? SERVO_STICK_UP:SERVO_STICK_IN);
        if ((gamepad1.right_trigger > 0.9) && BTN_FIX.canPress(timestamp)) {
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
            if (currentSweeperState == SweeperState.IN) {
                currentSweeperState = SweeperState.STOP;
            }
            else {
                currentSweeperState = SweeperState.IN;
            }
        }
        else if ((gamepad2.left_trigger > 0.9) && BTN_ROLLER_OUT.canPress(timestamp)) {
            if (currentSweeperState == SweeperState.OUT) {
                currentSweeperState = SweeperState.STOP;
            }
            else {
                currentSweeperState = SweeperState.OUT;
            }
        } else if (gamepad2.right_bumper) {
            if (currentSweeperState != SweeperState.FIX) {
                previousSweeperState = currentSweeperState;
            }
            currentSweeperState = SweeperState.FIX;
        } else {
            currentSweeperState = previousSweeperState;
        }

        if (currentSweeperState != SweeperState.FIX) {
            previousSweeperState = currentSweeperState;
            fixState = FixState.START;
        }

        if (gamepad2.b && BTN_DOOR.canPress(timestamp)) {
            isDoorOpen = !isDoorOpen;
        }
        robot.servoDoorRight.setPosition(isDoorOpen ? SERVO_DOOR_OUT : SERVO_DOOR_IN);
        //isStickUp = true;
        //robot.servoStick.setPosition(isStickUp? SERVO_STICK_UP: SERVO_STICK_IN);

        if (currentContainerState == ContainerState.FLAT ||
                currentContainerState == ContainerState.UP) {
            robot.sweeperLeft.setPower(0);
            robot.sweeperRight.setPower(0);
        }
        else if (isDoorOpen || timestamp - BTN_DOOR.lastPressTime < 6000) {
            if (currentSweeperState==SweeperState.IN) {
                robot.sweeperLeft.setPower(POWER_SWEEPER);
                robot.sweeperRight.setPower(POWER_SWEEPER);
            } else if (currentSweeperState==SweeperState.OUT) {
                robot.sweeperLeft.setPower(-POWER_SWEEPER);
                robot.sweeperRight.setPower(-POWER_SWEEPER);
            }
            else if (currentSweeperState == SweeperState.FIX) {
                switch (fixState) {
                    case START:
                        fixTimeStamp = System.currentTimeMillis();
                        fixState = FixState.FIN;
                        break;
                    case FIN:
                        robot.sweeperLeft.setPower(POWER_SWEEPER);
                        robot.sweeperRight.setPower(POWER_SWEEPER);
                        if (System.currentTimeMillis() - fixTimeStamp > 350) {
                            fixState = FixState.FOUT;
                            fixTimeStamp = System.currentTimeMillis();
                        }
                        break;
                    case FOUT:
                        robot.sweeperLeft.setPower(-POWER_SWEEPER);
                        robot.sweeperRight.setPower(-POWER_SWEEPER);
                        if (System.currentTimeMillis() - fixTimeStamp > 350) {
                            fixState = FixState.START;
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
            relicElbowState = RelicElbowState.REST;
        }
        /*rangeStickState = StickState.UP;
        if (gamepad1.b && BTN_ENCODER_BALANCE.canPress(timestamp)) {
            switch (rangeStickState) {
                case IN:
                    rangeStickState = StickState.UP;
                    break;
                case UP:
                    rangeStickState = StickState.OUT;
                    break;
                case OUT:
                    rangeStickState = StickState.IN;
                    break;
                default:
                    rangeStickState = StickState.UP;
                    break;
            }
        }
        switch (rangeStickState) {
            case IN:
                robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_IN);
            case UP:
                robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_UP);
            case OUT:
                robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_OUT);
        }*/
        //relic elbow operations
        if (gamepad2.x && BTN_ELBOW.canPress(timestamp)) {
            switch (relicElbowState) {
                case REST:
                case DOWN:
                    relicElbowState = RelicElbowState.UP;
                    break;
                case UP:
                    relicElbowState = RelicElbowState.DOWN;
                    break;
                default:
                    relicElbowState = RelicElbowState.UP;
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
                    if (relicElbowState != RelicElbowState.REST) {
                        relicClawState = RelicClawState.OPEN;
                    }
                    break;
                case OPEN:
                    relicClawState = RelicClawState.GRAB;
                    break;
                case GRAB:
                    relicClawState = RelicClawState.DROP;
                    break;
                case DROP:
                    relicClawState = RelicClawState.WIDEOPEN;
                    break;
                case WIDEOPEN:
                    relicClawState = RelicClawState.OPEN;
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
        //automatic jam detection and reverse intake
        if (gamepad1.y && BTN_CONTAINETR_UP.canPress(timestamp) && isDoorOpen && jamDetection.isJam()) {
            robot.sweeperLeft.setPower(-RobotRevAmpedConstants.POWER_SWEEPER);
            robot.sweeperRight.setPower(-RobotRevAmpedConstants.POWER_SWEEPER);
            try {
                Wait lw = new Wait(null);
                lw.waitMillis(750);
            } catch (InterruptedException e) {
            }
        }
        //relic slide
        boolean isTouchSwitchRelicSlideIn = robot.switchRelicSlideIn.isTouch();
        if (isTouchSwitchRelicSlideIn) {
            robot.motorRelicSlide.resetPosition(0);
        }
        if (relicElbowState == RelicElbowState.DOWN ||
                relicElbowState == RelicElbowState.UP) {
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
            /*float heading = robot.gyroSensor.getHeading();
            while (heading < -1 || heading > 1)  {
                if (heading<-1) {
                    drive.setPower(-0.3f, 0.3f);
                }else {
                    drive.setPower(0.3f, -0.3f);
                }
                heading = robot.gyroSensor.getHeading();
            }
            float roll = robot.gyroSensor.getRoll();
            float pitch = robot.gyroSensor.getPitch();
            Boolean correctPitch = false;
            Boolean correctRoll = false;
            telemetry.addData("pitch", pitch);
            telemetry.addData("roll", roll);
            telemetry.update();
            while (pitch>1.5 || pitch <-1.5 || roll > 1.5 || roll <-1.5) {
                if (roll > -1.5 && roll < 1.5) {
                    correctRoll = true;
                }
                if (pitch < 1.5 && pitch > -1.5) {
                    correctPitch = true;
                }
                if (pitch > 1.5 && roll > 1.5) {
                    drive.setPower(0.25f, 0, 0,0.25f );
                } else if (pitch >1.5 && roll <-1.5) {
                    drive.setPower(0, 0.25f, 0.25f, 0);
                }else if (pitch < -1.5 && roll > 1.5) {
                    drive.setPower(0, -0.25f, -0.25f, 0);
                } else if (pitch < -1.5 && roll < -1.5) {
                    drive.setPower(-0.25f, 0, 0,-0.25f );
                } else if (pitch < -1.5 && correctRoll) {
                    drive.setPower(-0.2f);
                } else if (pitch>1.5 && correctRoll) {
                    drive.setPower(0.2f);
                } else if (roll < -1.5 && correctPitch) {
                    drive.setStrafePower(-0.2f);
                } else if (roll > 1.5 && correctPitch) {
                    drive.setStrafePower(0.2f);
                } else {
                    break;
                }
                pitch = robot.gyroSensor.getPitch();
                roll = robot.gyroSensor.getRoll();
                telemetry.addData("pitch", pitch);
                telemetry.addData("roll", roll);
                telemetry.update();
            }*/
        } /*else if (gamepad1.b && BTN_ENCODER_BALANCE.canPress(timestamp)) {
            int startEncoder = drive.getEncoder();
            int encoderValue = startEncoder;
            timestamp = System.currentTimeMillis();
            long endTimestamp = timestamp;
            while ((encoderValue < startEncoder + 1000) && (endTimestamp-timestamp < 1975)) {
                drive.setPower(-0.3f, -0.3f, -0.3f, -0.3f);
                encoderValue = drive.getEncoder();
                endTimestamp = System.currentTimeMillis();
            }
            telemetry.addLine("done");
            telemetry.update();
        }*/

        robot.drawLed();
        /*if (isStickDown) {
            robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_UP-5);
            isStickDown = !isStickDown;
        } else {
            robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_UP-5);
        }*/
    }
}
