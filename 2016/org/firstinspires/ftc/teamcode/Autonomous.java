package org.firstinspires.ftc.teamcode;

import com.revAmped.components.TurnType;
import com.revAmped.config.RobotConstants;
import com.revAmped.config.RobotConstants.COLOR_SENSOR;
import com.revAmped.linear.components.RobotLinear;
import com.revAmped.linear.components.SwerveDriveLinear;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.linear.util.WaitLinear;
import com.revAmped.util.HwLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Autonomous
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Game")
public class Autonomous
    extends LinearOpMode
{
    private RobotLinear robot = null;

    private SwerveDriveLinear drive;

    private final static int MIN_FRONT_SPACING = -1;

    /**
     * autonomous opMode
     */
    @Override
    public void runOpMode()
        throws InterruptedException {
        try {
            // init
            robot = new RobotLinear(this);
            drive = robot.getSwerveDriveLinear();

            run();
        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    /**
     * autonomous function
     * @throws InterruptedException
     */
    public void run ()
        throws InterruptedException
    {
        robot.servoTrigger.setPosition(RobotConstants.SERVO_TRIGGER_IN);
        SelectLinear sl = new SelectLinear(this);
        boolean isRed = sl.selectAlliance();
        boolean isNear = sl.selectPosition();
        int numShoot = sl.adjust("Shoot", 2);
        boolean isRamp = sl.confirm("Ramp", false);
        boolean isBall = false;
        if(!isRamp) {
            isBall = sl.confirm("Ball", true);
        }
        int waitMillis = sl.adjustDelay();

        telemetry.addData("Alliance", isRed ? "Red" : "Blue");
        telemetry.addData("Position", isNear ? "Near" : "Far");
        telemetry.addData("Shoot", Integer.toString(numShoot));
        telemetry.addData("Ramp", isRamp ? "Yes" : "No");
        if(!isRamp) {
            telemetry.addData("Ball", isBall ? "Yes" : "No");
        }
        telemetry.addData("Delay", Integer.toString(waitMillis/1000));
        telemetry.addData("Waiting", "Autonomous");
        telemetry.update();

        if (isRed) {
            robot.ledRed.on();
        }
        else {
            robot.ledBlue.on();
        }
        robot.ledWhite.set(isBall);
        robot.ledGreen.set(isNear);
        robot.drawLed();
        robot.gyroSensor.resetHeading();

        waitForStart();

        long startTime = System.currentTimeMillis();
        robot.servoTrigger.setPosition(RobotConstants.SERVO_TRIGGER_IN);

        robot.ledRed.off();
        robot.ledBlue.off();
        robot.ledGreen.off();
        robot.ledWhite.off();
        robot.ledYellow.off();
        robot.drawLed();

        WaitLinear lp = new WaitLinear(this);
        lp.waitMillis(waitMillis);

        robot.roller.setPower(RobotConstants.POWER_ROLLER*2f/3f);
        robot.spinnerLeft.setPower(RobotConstants.POWER_SPINNER);
        robot.spinnerRight.setPower(RobotConstants.POWER_SPINNER);
        idle();

        drive.moveToEncoderInch(TurnType.FORWARD,
                                isNear ? 30 : 50,
                                1f,
                                5000,
                                -1,
                                true,
                                true,
                                true);

        if (isNear) {
            drive.setTurn(TurnType.STRAFE);
        }
        long timeStamp = System.currentTimeMillis();

        // shoot once or twice
        if (numShoot > 0) {
            robot.servoTrigger.setPosition(RobotConstants.SERVO_TRIGGER_OUT);
            lp.waitMillis(500);
            robot.servoTrigger.setPosition(RobotConstants.SERVO_TRIGGER_IN);
            lp.waitMillis(350);
        }
        if (numShoot > 1) {
            robot.servoTrigger.setPosition(RobotConstants.SERVO_TRIGGER_OUT);
            lp.waitMillis(500);
            robot.servoTrigger.setPosition(RobotConstants.SERVO_TRIGGER_IN);
        }

        robot.roller.setPower(0);
        robot.spinnerLeft.setPower(0);
        robot.spinnerRight.setPower(0);
        idle();

        int nextDistance = 0;
        if (isNear) {
            waitMillis = (int)(drive.TIME_TURN_WHEEL_SERVO - System.currentTimeMillis() + timeStamp);
            if (waitMillis > 0) {
                lp.waitMillis(waitMillis);
            }
            drive.moveToEncoderInch(TurnType.STRAFE,
                                    isRed ? -47 : 47,
                                    1f,
                                    8000,
                                    -1,
                                    true,
                                    true,
                                    true);
            drive.moveToEncoderInch(TurnType.FORWARD,
                                    29,
                                    1f,
                                    8000,
                                    MIN_FRONT_SPACING,
                                    true,
                                    true,
                                    true);
            drive.moveToLine(TurnType.FORWARD,
                             isRed ? COLOR_SENSOR.LOW_LEFT : COLOR_SENSOR.LOW_RIGHT,
                             0.20f,
                             4000,
                             MIN_FRONT_SPACING,
                             true,
                             true);
            drive.moveToSonar(TurnType.STRAFE,
                              isRed ? -0.3f : 0.3f,
                              // blue a bit far away
                              isRed ? 7.5f : 7f,
                              6000,
                              true,
                              true);
            drive.setTurn(TurnType.FORWARD);

            nextDistance = 57;
            int[] crgbB = robot.colorSensor.getCRGB(isRed ? COLOR_SENSOR.LEFT_BACK : COLOR_SENSOR.RIGHT_BACK);
            int[] crgbF = robot.colorSensor.getCRGB(isRed ? COLOR_SENSOR.LEFT_FRONT : COLOR_SENSOR.RIGHT_FRONT);
            Boolean isRedBeacon = robot.colorSensor.isRed(crgbF,
                                                         crgbB);
            HwLog.e("First color is red: " + isRedBeacon);
            if (Boolean.TRUE.equals(isRedBeacon) == isRed) {
                drive.moveToEncoderInch(TurnType.FORWARD,
                                        6,
                                        1f,
                                        4000,
                                        MIN_FRONT_SPACING,
                                        true,
                                        true,
                                        true);
                nextDistance = 49;
            }

            robot.servoBeacon.setPosition(RobotConstants.SERVO_BEACON_OUT);
            lp.waitMillis(2000);
            robot.servoBeacon.setPosition(RobotConstants.SERVO_BEACON_IN);
            lp.waitMillis(300);
            robot.servoBeacon.setPosition(RobotConstants.SERVO_BEACON_OUT);
            lp.waitMillis(700);
            robot.servoBeacon.setPosition(RobotConstants.SERVO_BEACON_IN);
            lp.waitMillis(500);

            drive.moveToEncoderInch(TurnType.FORWARD,
                                    nextDistance,
                                    1f,
                                    8000,
                                    MIN_FRONT_SPACING,
                                    true,
                                    true,
                                    true);
            robot.servoBeacon.setPosition(RobotConstants.SERVO_BEACON_STOP);

            // stop to be the same as the previous moveToLine
            drive.moveToLine(TurnType.FORWARD,
                             isRed ? COLOR_SENSOR.LOW_LEFT : COLOR_SENSOR.LOW_RIGHT,
                             0.20f,
                             4000,
                             MIN_FRONT_SPACING,
                             true,
                             true);

            crgbB = robot.colorSensor.getCRGB(isRed ? COLOR_SENSOR.LEFT_BACK : COLOR_SENSOR.RIGHT_BACK);
            crgbF = robot.colorSensor.getCRGB(isRed ? COLOR_SENSOR.LEFT_FRONT : COLOR_SENSOR.RIGHT_FRONT);
            isRedBeacon = robot.colorSensor.isRed(crgbF,
                                                  crgbB);
            HwLog.i("Second color is red: " + isRedBeacon);
            if (isRedBeacon == null) {
                // not able to determine the color
                drive.moveToSonar(TurnType.STRAFE,
                                  isRed ? -0.3f : 0.3f,
                                  7.5f,
                                  4000,
                                  true,
                                  true);
                drive.setTurn(TurnType.FORWARD);
                crgbB = robot.colorSensor.getCRGB(isRed ? COLOR_SENSOR.LEFT_BACK : COLOR_SENSOR.RIGHT_BACK);
                crgbF = robot.colorSensor.getCRGB(isRed ? COLOR_SENSOR.LEFT_FRONT : COLOR_SENSOR.RIGHT_FRONT);
                isRedBeacon = robot.colorSensor.isRed(crgbF,
                                                      crgbB);
                HwLog.i("Third color is red: " + isRedBeacon);
            }

            nextDistance = -114;

            if (Boolean.TRUE.equals(isRedBeacon) == isRed) {
                drive.moveToEncoderInch(TurnType.FORWARD,
                                        6,
                                        1f,
                                        4000,
                                        MIN_FRONT_SPACING,
                                        true,
                                        false,
                                        true);
                nextDistance = -120;
            }

            drive.setTurn(TurnType.STRAFE);
            robot.servoBeacon.setPosition(RobotConstants.SERVO_BEACON_OUT);
            lp.waitMillis(2000);
            robot.servoBeacon.setPosition(RobotConstants.SERVO_BEACON_IN);
            lp.waitMillis(300);
            robot.servoBeacon.setPosition(RobotConstants.SERVO_BEACON_OUT);
            lp.waitMillis(700);
            robot.servoBeacon.setPosition(RobotConstants.SERVO_BEACON_IN);

            drive.moveToEncoderInch(TurnType.STRAFE,
                                    isRed ? 12 : -12,
                                    1f,
                                    4000,
                                    7,
                                    false,
                                    false,
                                    true);
        }
        else if (isBall) {
            lp.waitMillis((int)((isRamp ? 18000 : 24000) - System.currentTimeMillis() + startTime - waitMillis));
        }

        if (isBall) {
            if (isNear) {
                drive.turn(TurnType.TURN_REGULAR,
                           isRed ? -34 : 34,
                           1f,
                           4000,
                           true);
                robot.servoBeacon.setPosition(RobotConstants.SERVO_BEACON_STOP);
            }
            drive.moveToEncoderInch(TurnType.FORWARD,
                                    isNear ? -72 : 38,
                                    1f,
                                    4000,
                                    -1,
                                    true,
                                    false,
                                    false);
        }
        else if (isRamp) {
            if (isNear) {
                drive.setTurnWait(TurnType.FORWARD);
                robot.servoBeacon.setPosition(RobotConstants.SERVO_BEACON_STOP);
                drive.moveToEncoderInch(TurnType.FORWARD,
                                        nextDistance,
                                        1f,
                                        4200,
                                        -1,
                                        false,
                                        false,
                                        true);
                drive.setTurn(TurnType.STRAFE);
            }
            else {
                drive.turn(TurnType.TURN_REGULAR,
                           isRed ? 30 : -30,
                           1f,
                           4000,
                           true);
                drive.moveToEncoderInch(TurnType.STRAFE,
                                        isRed ? -75 : 75,
                                        1f,
                                        4000,
                                        -1,
                                        false,
                                        false,
                                        true);
                drive.setTurn(TurnType.FORWARD);
            }
            lp.waitMillis(1000);
        }

        robot.servoBeacon.setPosition(RobotConstants.SERVO_BEACON_STOP);

        HwLog.i("Autonomous finish in " + (System.currentTimeMillis() - startTime) + " milliseconds");
    }
}
