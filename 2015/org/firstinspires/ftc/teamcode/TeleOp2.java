package org.firstinspires.ftc.teamcode;

import com.revAmped.components.Button;
import com.revAmped.components.Robot;
import com.revAmped.components.TankDrive;
import com.revAmped.config.RobotConstants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Refactored
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Game")
public class TeleOp2
    extends OpMode {

    private Robot robot;

    private TankDrive drive;

    private float pwrLeft, pwrRight, pwrSlide;
    private static final float POWER_ROLLER = 0.9f;
    private static final float POWER_SLIDE = 1.0f;

    private boolean isSlow = false;
    private boolean isReverse = false;
    private boolean isFrontDoorOpen = false;
    private boolean isClimberOut = false;
    private boolean isPopper = false;
    private boolean isWingLeft = false;
    private boolean isWingRight = false;
    private boolean isHookUp = true;
    boolean rollerDoor = false;

    private ERollerStatus rollStatus = ERollerStatus.ROLL_STOP;
    private EDumpStatus dumpStatus = EDumpStatus.DUMP_START;
    private EMidPresetStatus midPresetStatus = EMidPresetStatus.SLIDE_MID_PRESET_HIGH;

    private final static int ENCODER_SLIDE_MAX = 10200;
    private final static int SLIDE_HIGH_PRESET = 10200;
    private final static int SLIDE_MID_PRESET = 6300;
    private final static int SLIDE_MID_PRESET_LOW = 3450;
    private final static int SLIDE_BUCKET_PRESET = 5300;
    private final static int SLIDE_LOW_PRESET = 1000;

    private float posBucketTurnMid = RobotConstants.SERVO_BUCKET_TURN_CENTER;
    private long bucketTimestamp;

    private static final Button BTN_ROLLER_IN = new Button();
    private static final Button BTN_ROLLER_OUT = new Button();
    private static final Button BTN_FDOOR_CLOSED = new Button();
    private static final Button BTN_FDOOR_OPEN = new Button();
    private static final Button BTN_HALF_GEAR = new Button();
    private static final Button BTN_REVERSE_MODE = new Button();
    private static final Button BTN_PRESET_LEFT = new Button();
    private static final Button BTN_PRESET_RIGHT = new Button();
    private static final Button BTN_PRESET_CENTER = new Button();
    private static final Button BTN_HOOK = new Button();
    private static final Button BTN_UNHOOK = new Button();
    private static final Button BTN_X = new Button();
    private static final Button BTN_HANGER = new Button();
    private static final Button BTN_PRESET_HIGHR = new Button();
    private static final Button BTN_PRESET_HIGHL = new Button();
    private static final Button BTN_CLIMBER = new Button();
    private static final Button BTN_WING_LEFT = new Button();
    private static final Button BTN_WING_RIGHT = new Button();
    private static final Button BTN_MID_LEFT = new Button();
    private static final Button BTN_MID_RIGHT = new Button();

    public enum ERollerStatus
    {
        ROLL_IN,
        ROLL_STOP,
        ROLL_OUT;
    }

    public enum EDumpStatus
    {
        DUMP_START,
        DUMP_OPEN,
        DUMP_FRONT_PUSH,
        DUMP_BACK_PUSH,
        DUMP_DONE;
    }

    public enum EMidPresetStatus
    {
        SLIDE_MID_PRESET_HIGH,
        SLIDE_MID_PRESET_LOW,
        SLIDE_MID_PRESET_DONE;
    }

    @Override
    public void init() {
        robot = new Robot(this,
                          false);
        drive = robot.getTankDrive();
    }

    @Override
    public void stop() {
        robot.close();
    }

    @Override
    public void loop() {

        long timestamp = System.currentTimeMillis();

        float x1 = gamepad1.left_stick_x;
        float y1 = gamepad1.left_stick_y;
        float x2 = gamepad1.right_stick_x;
        float y2 = gamepad1.right_stick_y;

        float tx1 = gamepad2.left_stick_x;
        float ty1 = gamepad2.left_stick_y;
        float tx2 = gamepad2.right_stick_x;
        float ty2 = gamepad2.right_stick_y;

        pwrLeft = Button.scaleInput(y1);
        pwrRight = Button.scaleInput(y2);

        int slideRightPosition = robot.slideRight.getCurrentPosition();
        int slideLeftPosition = robot.slideLeft.getCurrentPosition();

        // drive
        if(gamepad1.left_trigger > Button.TRIGGER_THRESHOLD && BTN_HALF_GEAR.canPress(timestamp)) {
            isSlow = !isSlow;
        }
        if(isSlow) {
            pwrLeft *= 0.5f;
            pwrRight *= 0.5f;
            robot.ledYellow.on();
        }
        else if (!isPopper) {
            robot.ledYellow.off();
        }

        if(gamepad1.a && BTN_REVERSE_MODE.canPress(timestamp)) {
            isReverse = !isReverse;
        }
        if (isReverse && (y1 * y2 >= 0)) {
            // not ((y1 >= 0 && y2 <= 0) || (y1 <= 0 && y2 >= 0))
            // y1 and y2 are same sign
            pwrLeft = -pwrLeft;
            pwrRight = -pwrRight;
        }

        pwrLeft = Range.clip(pwrLeft, -1f, 1f);
        pwrRight = Range.clip(pwrRight, -1f, 1f);

        robot.driveLeftFront.setPower(Robot.AM40_ENCODER_RATIO * pwrLeft);
        robot.driveLeftBack.setPower(Robot.AM40_ENCODER_RATIO * pwrLeft);
        robot.driveRightFront.setPower(Robot.AM40_ENCODER_RATIO * pwrRight);
        robot.driveRightBack.setPower(Robot.AM40_ENCODER_RATIO * pwrRight);

        // hooks
        if(gamepad1.left_bumper && BTN_UNHOOK.canPress(timestamp)) {
            isHookUp = true;
        }
        else if(gamepad1.right_bumper && BTN_HOOK.canPress(timestamp)) {
            isHookUp = false;
        }
        if(isHookUp) {
            robot.servoHookLeft.setPosition(RobotConstants.SERVO_HOOKL_UP);
            robot.servoHookRight.setPosition(RobotConstants.SERVO_HOOKR_UP);
        }
        else {
            robot.servoHookLeft.setPosition(RobotConstants.SERVO_HOOKL_DOWN);
            robot.servoHookRight.setPosition(RobotConstants.SERVO_HOOKR_DOWN);
        }

        //hanger
        if(gamepad2.y && gamepad2.left_bumper && BTN_HANGER.canPress(timestamp)) {
            isPopper = !isPopper;
        }
        if(isPopper) {
            robot.servoSlide.setPosition(RobotConstants.SERVO_SLIDE_END);
            robot.ledGreen.on();
            robot.ledWhite.on();
            robot.ledYellow.on();
        } else {
            robot.servoSlide.setPosition(RobotConstants.SERVO_SLIDE_START);
            robot.ledGreen.off();
            robot.ledWhite.off();
            robot.ledYellow.off();
        }

        //slide
        pwrSlide = -Button.scaleInput(ty1);

        if(robot.switchSlideDown.isTouch()) {
            robot.slideLeft.resetPosition();
            robot.slideRight.resetPosition();
        }
        else if (robot.switchSlideUp.isTouch()) {
            // in case of restart, slide encodes may be off
            if (robot.slideLeft.getCurrentPosition() < ENCODER_SLIDE_MAX / 2) {
                robot.slideLeft.resetPosition(ENCODER_SLIDE_MAX);
            }
            if (robot.slideRight.getCurrentPosition() < ENCODER_SLIDE_MAX / 2) {
                robot.slideRight.resetPosition(ENCODER_SLIDE_MAX);
            }
        }

        if(robot.switchSlideDown.isTouch() && pwrSlide < 0) {
            pwrSlide = 0;
        }
        else if(robot.switchSlideUp.isTouch() && pwrSlide > 0) {
            pwrSlide = 0;
        }
        /*else if(slideRightPosition < SLIDE_HANG_LIMIT && g_isPopper && pwrSlide > 0) {
            //when almost latched, don't let the slide go up
            pwrSlide = 0;
        }*/
        else if(gamepad2.b || gamepad2.x) {
            if (slideRightPosition < SLIDE_HIGH_PRESET - 250 &&
                slideLeftPosition < SLIDE_HIGH_PRESET - 250) {
                pwrSlide = POWER_SLIDE;
            }
            else if(slideRightPosition > SLIDE_HIGH_PRESET &&
                    slideLeftPosition > SLIDE_HIGH_PRESET) {
                pwrSlide = -POWER_SLIDE;
            }
            else {
                pwrSlide = 0;
            }
        }
        else if(!gamepad2.left_bumper && (gamepad2.dpad_left || gamepad2.dpad_right)) {
            switch (midPresetStatus) {
                case SLIDE_MID_PRESET_HIGH:
                    if (slideRightPosition < SLIDE_MID_PRESET - 250 &&
                        slideLeftPosition < SLIDE_MID_PRESET - 250) {
                        pwrSlide = POWER_SLIDE;
                    }
                    else if(slideRightPosition > SLIDE_MID_PRESET &&
                        slideLeftPosition > SLIDE_MID_PRESET) {
                        pwrSlide = -POWER_SLIDE;
                    }
                    else {
                        pwrSlide = 0;
                        midPresetStatus = EMidPresetStatus.SLIDE_MID_PRESET_LOW;
                    }
                    break;
                case SLIDE_MID_PRESET_LOW:
                    if (slideRightPosition < SLIDE_MID_PRESET_LOW &&
                        slideLeftPosition < SLIDE_MID_PRESET_LOW) {
                        pwrSlide = POWER_SLIDE;
                    }
                    else if(slideRightPosition > SLIDE_MID_PRESET_LOW + 250 &&
                        slideLeftPosition > SLIDE_MID_PRESET_LOW + 250) {
                        pwrSlide = -POWER_SLIDE;
                    }
                    else {
                        pwrSlide = 0;
                        midPresetStatus = EMidPresetStatus.SLIDE_MID_PRESET_DONE;
                    }
                    break;
                default:
                case SLIDE_MID_PRESET_DONE:
                    pwrSlide = 0;
                    break;
            }
        }
        else if(gamepad2.left_bumper && (gamepad2.dpad_left || gamepad2.dpad_right)) {
            if (robot.switchSlideUp.isTouch()) {
                pwrSlide = 0;
            }
            else {
                pwrSlide = POWER_SLIDE;
            }
        }
        else {
            midPresetStatus = EMidPresetStatus.SLIDE_MID_PRESET_HIGH;
        }

        robot.slideLeft.setPower(pwrSlide);
        robot.slideRight.setPower(pwrSlide);

        telemetry.addData("slideEncoderL", Integer.toString(slideLeftPosition));
        telemetry.addData("slideEncoderR", Integer.toString(slideRightPosition));

        if(robot.switchSlideDown.isTouch()) {
            posBucketTurnMid = RobotConstants.SERVO_BUCKET_TURN_CENTER;
        }
        else if (slideLeftPosition > SLIDE_LOW_PRESET &&
            slideRightPosition > SLIDE_LOW_PRESET) {
            tx2 = Button.scaleInput(tx2);
            if(Math.abs(tx2) >= 0.15 && BTN_X.canPress4Short(timestamp)) {
                posBucketTurnMid -= 0.01f * tx2;
            }

            if (slideLeftPosition > SLIDE_BUCKET_PRESET &&
                slideRightPosition > SLIDE_BUCKET_PRESET) {
                if (gamepad2.a && BTN_PRESET_CENTER.canPress(timestamp)) {
                    posBucketTurnMid = RobotConstants.SERVO_BUCKET_TURN_CENTER;
                }
                else if (!gamepad2.left_bumper && gamepad2.dpad_left && BTN_PRESET_LEFT.canPress(timestamp)) {
                    posBucketTurnMid = RobotConstants.SERVO_BUCKET_TURN_LEFT;
                }
                else if (!gamepad2.left_bumper && gamepad2.dpad_right && BTN_PRESET_RIGHT.canPress(timestamp)) {
                    posBucketTurnMid = RobotConstants.SERVO_BUCKET_TURN_RIGHT;
                }
                else if (gamepad2.b && BTN_PRESET_HIGHR.canPress(timestamp)) {
                    posBucketTurnMid = RobotConstants.SERVO_BUCKET_TURN_RIGHT;
                }
                else if (gamepad2.x && BTN_PRESET_HIGHL.canPress(timestamp)) {
                    posBucketTurnMid = RobotConstants.SERVO_BUCKET_TURN_LEFT;
                }
                else if(gamepad2.left_bumper && gamepad2.dpad_left && BTN_MID_LEFT.canPress(timestamp)) {
                    posBucketTurnMid = RobotConstants.SERVO_BUCKET_MID_FRONT_LEFT;
                }
                else if(gamepad2.left_bumper && gamepad2.dpad_right && BTN_MID_RIGHT.canPress(timestamp)) {
                    posBucketTurnMid = RobotConstants.SERVO_BUCKET_MID_FRONT_RIGHT;
                }
            }
        }

        posBucketTurnMid = Range.clip(posBucketTurnMid, 0f, 1f);
        robot.servoBucketTurn.setPosition(posBucketTurnMid);
        telemetry.addData("Bucket Position", Float.toString(posBucketTurnMid));

        if(robot.switchBucket.isTouch()) {
            robot.ledGreen.on();
        }
        else if (!isPopper){
            robot.ledGreen.off();
        }

        if(gamepad2.left_bumper && BTN_FDOOR_CLOSED.canPress(timestamp)) {
            isFrontDoorOpen = false;
        }
        else if(gamepad2.left_trigger > Button.TRIGGER_THRESHOLD && BTN_FDOOR_OPEN.canPress(timestamp)) {
            isFrontDoorOpen = true;
        }

        if(rollStatus == ERollerStatus.ROLL_IN && (robot.switchSlideDown.isTouch())) {
            if (!rollerDoor) {
                isFrontDoorOpen = true;
                rollerDoor = true;
            }
        }
        else {
            rollerDoor = false;
        }

        if (gamepad2.back &&
            slideLeftPosition > SLIDE_BUCKET_PRESET / 2 &&
            slideRightPosition > SLIDE_BUCKET_PRESET / 2) {
            //dump
            switch (dumpStatus) {
                case DUMP_START:
                    bucketTimestamp = timestamp;
                    dumpStatus = EDumpStatus.DUMP_OPEN;
                    break;
                case DUMP_OPEN:
                    robot.servoBucket.setPosition(RobotConstants.SERVO_BUCKET_OPEN);
                    robot.servoFrontDoor.setPosition(RobotConstants.SERVO_FDOOR_EASE);
                    robot.servoBackDoor.setPosition(RobotConstants.SERVO_BDOOR_OPEN);
                    if (timestamp - bucketTimestamp > 500) {
                        dumpStatus = EDumpStatus.DUMP_FRONT_PUSH;
                        bucketTimestamp = timestamp;
                    }
                    break;
                case DUMP_FRONT_PUSH:
                    robot.servoBucket.setPosition(RobotConstants.SERVO_BUCKET_OPEN);
                    robot.servoFrontDoor.setPosition(RobotConstants.SERVO_FDOOR_PUSH);
                    robot.servoBackDoor.setPosition(RobotConstants.SERVO_BDOOR_OPEN);
                    isFrontDoorOpen = true;
                    if (timestamp - bucketTimestamp > 800) {
                        dumpStatus = EDumpStatus.DUMP_BACK_PUSH;
                        bucketTimestamp = timestamp;
                    }
                    break;
                case DUMP_BACK_PUSH:
                    robot.servoBucket.setPosition(RobotConstants.SERVO_BUCKET_OPEN_WIDE);
                    robot.servoFrontDoor.setPosition(RobotConstants.SERVO_FDOOR_CLOSED);
                    robot.servoBackDoor.setPosition(RobotConstants.SERVO_BDOOR_PUSH);
                    isFrontDoorOpen = true;
                    if (timestamp - bucketTimestamp > 1000) {
                        dumpStatus = EDumpStatus.DUMP_DONE;
                        bucketTimestamp = timestamp;
                    }
                    break;
                default:
                case DUMP_DONE:
                    robot.servoBucket.setPosition(RobotConstants.SERVO_BUCKET_OPEN_WIDE);
                    robot.servoFrontDoor.setPosition(RobotConstants.SERVO_FDOOR_OPEN);
                    robot.servoBackDoor.setPosition(RobotConstants.SERVO_BDOOR_PUSH);
                    isFrontDoorOpen = true;
            }
        }
        else if (gamepad2.start) {
            isFrontDoorOpen = true;
            robot.servoBucket.setPosition(RobotConstants.SERVO_BUCKET_CLOSED);
            robot.servoFrontDoor.setPosition(RobotConstants.SERVO_FDOOR_OPEN);
            robot.servoBackDoor.setPosition(RobotConstants.SERVO_BDOOR_PUSH);
        }
        else {
            dumpStatus = EDumpStatus.DUMP_START;
            robot.servoBucket.setPosition(RobotConstants.SERVO_BUCKET_CLOSED);
            robot.servoBackDoor.setPosition(RobotConstants.SERVO_BDOOR_OPEN);

            if (isFrontDoorOpen) {
                robot.servoFrontDoor.setPosition(RobotConstants.SERVO_FDOOR_OPEN);
                if (!isPopper) {
                    robot.ledWhite.off();
                }
            } else {
                robot.servoFrontDoor.setPosition(RobotConstants.SERVO_FDOOR_CLOSED);
                robot.ledWhite.on();
            }
        }

        //roller
        if(gamepad2.right_trigger > Button.TRIGGER_THRESHOLD && BTN_ROLLER_IN.canPress(timestamp)) {
            if(rollStatus == ERollerStatus.ROLL_OUT || rollStatus == ERollerStatus.ROLL_STOP) {
                rollStatus = ERollerStatus.ROLL_IN;
            }
            else if(rollStatus == ERollerStatus.ROLL_IN) {
                rollStatus = ERollerStatus.ROLL_STOP;
            }
        }
        else if(gamepad2.right_bumper && BTN_ROLLER_OUT.canPress(timestamp)) {
            if(rollStatus == ERollerStatus.ROLL_IN || rollStatus == ERollerStatus.ROLL_STOP) {
                rollStatus = ERollerStatus.ROLL_OUT;
            }
            else if(rollStatus == ERollerStatus.ROLL_OUT) {
                rollStatus = ERollerStatus.ROLL_STOP;
            }
        }

        if(Math.abs(pwrSlide) > 0) {
            robot.roller.setPower(0);
        }
        else if (rollStatus == ERollerStatus.ROLL_IN) {
            robot.roller.setPower(POWER_ROLLER);
        } else if (rollStatus == ERollerStatus.ROLL_OUT) {
            robot.roller.setPower(-POWER_ROLLER);
        } else {
            robot.roller.setPower(0);
        }

        //wings
        if(gamepad1.b && BTN_WING_RIGHT.canPress(timestamp)) {
            isWingRight = !isWingRight;
        }
        if(gamepad1.x && BTN_WING_LEFT.canPress(timestamp)) {
            isWingLeft = !isWingLeft;
        }
        if(isWingRight) {
            robot.servoWingRight.setPosition(RobotConstants.SERVO_WINGR_OUT);
            robot.ledRed.on();
        }
        else {
            robot.servoWingRight.setPosition(RobotConstants.SERVO_WINGR_IN);
            robot.ledRed.off();
        }
        if(isWingLeft) {
            robot.servoWingLeft.setPosition(RobotConstants.SERVO_WINGL_OUT);
            robot.ledBlue.on();
        }
        else {
            robot.servoWingLeft.setPosition(RobotConstants.SERVO_WINGL_IN);
            robot.ledBlue.off();
        }

        //climbers
        if(gamepad1.y && BTN_CLIMBER.canPress(timestamp)) {
            isClimberOut = !isClimberOut;
        }
        if(isClimberOut) {
            robot.servoClimber.setPosition(RobotConstants.SERVO_CLIMBER_OUT);
        }
        else {
            robot.servoClimber.setPosition(RobotConstants.SERVO_CLIMBER_IN);
        }

        robot.drawLed();

        telemetry.addData("Reverse", Boolean.toString(isReverse));
    }
}
