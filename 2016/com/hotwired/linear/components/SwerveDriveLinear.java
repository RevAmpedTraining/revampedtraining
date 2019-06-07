package com.revAmped.linear.components;

import com.revAmped.components.Drive;
import com.revAmped.components.HwDevice;
import com.revAmped.components.HwGyro;
import com.revAmped.components.HwMotor;
import com.revAmped.components.HwServo;
import com.revAmped.components.HwSonarAnalog;
import com.revAmped.components.SwerveDrive;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotConstants.COLOR_SENSOR;
import com.revAmped.linear.util.WaitLinear;
import com.revAmped.sensors.MultiplexColorSensor;
import com.revAmped.util.HwLog;
import com.revAmped.util.Stalled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

/**
 * This class is for Autonomous
 */
public class SwerveDriveLinear
    extends SwerveDrive
{
    public final HwGyro gyroSensor;
    public final HwSonarAnalog sonarFront, sonarLeft, sonarRight;
    public final MultiplexColorSensor colorSensor;

    private HwMotorsLinear motors;

    private LinearOpMode op;

    private long  logTimeStamp = 0;

    /** constant for ramping up robot */
    private final static float POWER_STOP = 0.10f;
    private final static float POWER_START = 0.17f;
    private final static int MOVE_STALL_TIME = 500;
    private final static int MOVE_STALL_TIME_SHORT = 200;

    public SwerveDriveLinear(HwMotor driveLeftFront,
                             HwMotor driveLeftBack,
                             HwMotor driveRightFront,
                             HwMotor driveRightBack,
                             HwServo servoLeftFront,
                             HwServo servoLeftBack,
                             HwServo servoRightFront,
                             HwServo servoRightBack,
                             HwGyro gyroSensor,
                             HwSonarAnalog sonarFront,
                             HwSonarAnalog sonarLeft,
                             HwSonarAnalog sonarRight,
                             MultiplexColorSensor colorSensor)
    {
        super(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack,
                servoLeftFront,
                servoLeftBack,
                servoRightFront,
                servoRightBack);

        this.gyroSensor = gyroSensor;
        this.sonarFront = sonarFront;
        this.sonarLeft = sonarLeft;
        this.sonarRight = sonarRight;
        this.colorSensor = colorSensor;
        this.motors = new HwMotorsLinear(this.motorsArray);
    }

    /**
     * sets the LinearOpMode
     * @param op the OpMode to run
     */
    public void setLinearOpMode (LinearOpMode op) {
        this.op = op;
        motors.setLinearOpMode(op);
    }

    /**
     * set motor regulation
     * @param mode motor regulation to set to
     */
    public void setMode2(RunMode mode)
        throws InterruptedException
    {
        motors.setMode(mode);
    }

    /**
     * stops the motors
     * @throws InterruptedException
     */
    public void stop2()
        throws InterruptedException
    {
        motors.stop();
    }

    /**
     * set motor power float until it is confirmed.
     * @throws InterruptedException
     */
    public void setZeroPowerBehavior2(ZeroPowerBehavior behavior)
        throws InterruptedException
    {
        motors.setZeroPowerBehavior(behavior);
    }

    /**
     * set the orientation while waiting
     * @param turnType orientation to turn to
     * @throws InterruptedException
     */
    public void setTurnWait(TurnType turnType)
            throws InterruptedException
    {
        int waitMillis = TIME_TURN_WHEEL_SERVO;

        if (turnType != wheelOrientation) {
            if (turnType == TurnType.TURN_REGULAR || wheelOrientation == TurnType.TURN_REGULAR) {
                waitMillis = waitMillis/2;
            }
            setTurn(turnType);
            WaitLinear lp = new WaitLinear(op);
            lp.waitMillis(waitMillis);
        }
    }

    /**
     * set motor target position until it is confirmed
     * @param position motor target position
     */
    public void setTargetPosition2(int position)
            throws InterruptedException
    {
        motors.setTargetPosition(position);
    }

    /**
     * set motor target position until it is confirmed
     * @param position motor target position
     */
    public void setTargetPosition2(int position,
                                   HwDevice[] motorArry)
            throws InterruptedException
    {
        motors.setTargetPosition(position,
                                 motorArry);
    }

    /**
     * move wheels according to distance in inches
     * @param dir orientation of movement
     * @param distanceInch the distance to move in inches
     * @param maxPower maximum motor power
     * @param timeoutMillis maximum execution time for this function in milliseconds.
     * @param isAccelerate whether to ramp up and down. Use false for very short distance.
     * @param isBreak whether to break
     */
    public void moveToEncoderInch(TurnType dir,
                                  int distanceInch,
                                  float maxPower,
                                  int timeoutMillis,
                                  int minSpacing,
                                  boolean isAccelerate,
                                  boolean isAdjust,
                                  boolean isBreak)
        throws InterruptedException
    {
        int tick = inchToTick(distanceInch);
        moveToEncoder(dir,
                      tick,
                      maxPower,
                      timeoutMillis,
                      minSpacing,
                      isAccelerate,
                      isAdjust,
                      isBreak,
                      null);
    }

    public void moveToEncoderInch(TurnType dir,
                                  int distanceInch,
                                  float maxPower,
                                  int timeoutMillis,
                                  int minSpacing,
                                  boolean isAccelerate,
                                  boolean isAdjust,
                                  boolean isBreak,
                                  MoveAction... actions)
        throws InterruptedException
    {
        int tick = inchToTick(distanceInch);
        moveToEncoder(dir,
                      tick,
                      maxPower,
                      timeoutMillis,
                      minSpacing,
                      isAccelerate,
                      isAdjust,
                      isBreak,
                      actions);
    }

    /**
     * move wheels according to motor encoder ticks
     * @param dir orientation of movement
     * @param tick the distance to move in motor encoder tick
     * @param maxPower maximum motor power
     * @param timeoutMillis maximum execution time for this function in milliseconds
     * @param isAccelerate whether to ramp up and down. Use false for very short distance
     * @param isBreak whether to break
     */
    public void moveToEncoder(TurnType dir,
                              int tick,
                              float maxPower,
                              int timeoutMillis,
                              int minSpacing,
                              boolean isAccelerate,
                              boolean isAdjust,
                              boolean isBreak,
                              MoveAction[] actions)
        throws InterruptedException
    {
        final float MOVE_TIMEOUT_FACTOR = 2f;

        maxPower = Math.abs(maxPower);
        maxPower = Range.clip(maxPower, 0f, 1f);

        if (timeoutMillis < 0) {
            timeoutMillis = Math.abs((int) (tick * Drive.TETRIX_MILLISECOND_PER_TICK * MOVE_TIMEOUT_FACTOR)) + 2000;
        }

        // turn wheels
        setTurnWait(dir);

        resetPosition();
        // is RUN_TO_POSITION
        setMode2(RunMode.RUN_TO_POSITION);
        if (dir == TurnType.STRAFE) {
            setTargetPosition2(tick,
                               new HwDevice[]{driveRightBack, driveLeftFront});
            setTargetPosition2(-tick,
                               new HwDevice[]{driveRightFront, driveLeftBack});
        } else {
            setTargetPosition2(tick);
        }

        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;

        Stalled stalled = new Stalled();
        while (op.opModeIsActive() &&
            // is RUN_TO_POSITION
            isBusy() &&
            (timeStamp - startTimestamp < timeoutMillis)) {

            int average = ((int)Math.signum(tick))*getEncoder(dir);
            int delta = tick - average;
            if (Math.abs(delta) < 50)
            {
                // done
                break;
            }

            float basePower;
            if (!isAccelerate)
            {
                basePower = Math.signum(delta) * maxPower;
            }
            else
            {
                if (Math.abs(average) < Math.abs(tick / 2))
                {
                    // ramp up for 1/3 of distance
                    // minimum power is START_POWER
                    // base is 1600
                    basePower = (average/1200f) + Math.signum(tick) * POWER_START;
                }
                else
                {
                    // ramp down for 2/3 of distance
                    // minimum power is START_POWER
                    // base is 3200
                    basePower = (delta/2400f) + Math.signum(delta) * POWER_STOP;
                }
                basePower = Range.clip(basePower, -maxPower, maxPower);

                /*if (dir == TurnType.STRAFE) {
                    long timeStamp1 = System.currentTimeMillis();
                    if (timeStamp1 - logTimeStamp > 25) {
                        HwLog.i("Avg:" + average + " Delta:" + delta + " Power:" + basePower);
                        logTimeStamp = timeStamp1;
                    }
                }*/
            } // if

            //op.telemetry.addData("power", Float.toString(basePower));
            //op.telemetry.addData("travel", Integer.toString(tickToInch(average)));
            //op.telemetry.addData("delta", Integer.toString(tickToInch(delta)));

            if (stalled.isStalled(average, timeStamp)) {
                WaitLinear w = new WaitLinear(op);
                stop2();
                w.waitMillis(MOVE_STALL_TIME);
                timeoutMillis += MOVE_STALL_TIME;
                stalled.reset();
            }
            else {
                float spacing = getSpacing(dir,
                                           tick,
                                           minSpacing);
                if (spacing < minSpacing) {
                    WaitLinear w = new WaitLinear(op);
                    stop2();
                    w.waitMillis(MOVE_STALL_TIME_SHORT);
                    timeoutMillis += MOVE_STALL_TIME_SHORT;
                }
                else if (isAdjust) {
                    adjustPower(dir,
                                basePower);
                }
                else {
                    setPower(basePower,
                             basePower,
                             dir);
                }
            } // else

            if (actions != null) {
                for (MoveAction action : actions) {
                    if (action == null) {
                        continue;
                    }

                    if (Math.abs(average) > Math.abs(action.distanceInTick())) {
                        action.perform();
                    }
                }
            }

            //op.telemetry.addData("driveLeftFront", driveLeftFront.getCurrentPosition());
            //op.telemetry.addData("driveRightBack", driveRightBack.getCurrentPosition());
            //op.telemetry.addData("driveRightFront", driveRightFront.getCurrentPosition());
            //op.telemetry.addData("driveLeftBack", driveLeftBack.getCurrentPosition());

            //op.telemetry.update();
            op.idle();
            timeStamp = System.currentTimeMillis();
        } // while

        if (isBreak) {
            stop2();
        }

        if (Thread.currentThread().isInterrupted()) {
            HwLog.e("Thread interrupted");
        }
        else if (op.isStopRequested()) {
            HwLog.e("Stop requested");
        }

        // is RUN_TO_POSITION
        setMode2(RunMode.RUN_USING_ENCODER);

        // stop actions
        if (actions != null) {
            for (MoveAction action : actions) {
                if (action == null) {
                    continue;
                }

                action.stop();
                HwLog.i("Actions " + action + " stopped");
            }
            op.idle();
        }

        int inches = tickToInch(getEncoder(dir));
        HwLog.i("Moved " + inches + " inches");
        HwLog.i("Move took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
        //op.telemetry.addData("travel", inches);
        op.idle();
    }

    /**
     * moves for a certain amount of time
     * @param dir orientaiton of movement
     * @param power power to move at
     * @param timeoutMillis timeout
     * @throws InterruptedException
     */
    public void moveToTime (final TurnType dir,
                            float power,
                            int timeoutMillis)
        throws InterruptedException
    {
        final float p = Range.clip(power, -1f, 1f);

        // turn wheels
        setTurnWait(dir);

        setMode2(RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior2(ZeroPowerBehavior.FLOAT);

        // in case skipping
        setPower(p, p, dir);

        WaitLinear lp = new WaitLinear(op);
        lp.waitMillis(timeoutMillis);

        stop2();
        setMode2(RunMode.RUN_USING_ENCODER);
        setZeroPowerBehavior2(ZeroPowerBehavior.BRAKE);

        op.idle();
    }

    /**
     * moves until the color sensor detects the white line
     * @param dir orientation of wheels
     * @param colorPort color sensor to use
     * @param power power to move at
     * @param timeoutMillis maximum execution time for this function in milliseconds.
     * @param minSpacing spacing required for proper readings
     * @param isAdjust whether to course adjust or not
     * @param isBreak whether to break
     * @throws InterruptedException
     */
    public void moveToLine (final TurnType dir,
                            COLOR_SENSOR colorPort,
                            float power,
                            int timeoutMillis,
                            int minSpacing,
                            boolean isAdjust,
                            boolean isBreak)
            throws InterruptedException
    {
        final float p = Range.clip(power, -1f, 1f);

        if (timeoutMillis < 0) {
            timeoutMillis = 10000;
        }

        // turn wheels
        setTurnWait(dir);

        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;

        int lux = 0;
        while (op.opModeIsActive() &&
                (timeStamp - startTimestamp < timeoutMillis)) {

            lux = colorSensor.luminance(colorPort);
            //op.telemetry.addData("lux", lux);
            if (lux > colorSensor.LUX_LINE_WHITE) {
                break;
            }

            float spacing = getSpacing(dir,
                                       power,
                                       minSpacing);
            if (spacing < minSpacing) {
                WaitLinear w = new WaitLinear(op);
                stop2();
                w.waitMillis(MOVE_STALL_TIME_SHORT);
                timeoutMillis += MOVE_STALL_TIME_SHORT;
            }
            else if (isAdjust) {
                adjustPower(dir,
                            p);
            }
            else {
                // in case skipping
                setPower(p, p, dir);
            }

            //op.telemetry.update();
            op.idle();
            timeStamp = System.currentTimeMillis();
        } // while

        if (isBreak) {
            stop2();
        }

        if (Thread.currentThread().isInterrupted()) {
            HwLog.e("Thread interrupted");
        }
        else if (op.isStopRequested()) {
            HwLog.e("Stop requested");
        }

        HwLog.i("Lux: " + lux);
        HwLog.i("Move took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
        op.idle();
    }

    /**
     * move within a certain distance of something
     * @param dir orientation of movement
     * @param power power to move at
     * @param distanceLimit target distance
     * @param timeoutMillis maximum execution time for this function in milliseconds
     * @param isAdjust whether to course adjust or not
     * @param isBreak whether to break
     * @throws InterruptedException
     */
    public void moveToSonar(final TurnType dir,
                            float power,
                            float distanceLimit,
                            int timeoutMillis,
                            boolean isAdjust,
                            boolean isBreak)
            throws InterruptedException
    {
        final float p = Range.clip(power, -1f, 1f);

        // turn wheels
        setTurnWait(dir);

        if (timeoutMillis < 0) {
            timeoutMillis = 30000;
        }

        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;

        float distance = Float.MAX_VALUE;
        while (op.opModeIsActive() &&
                (timeStamp - startTimestamp < timeoutMillis)) {

            if (dir == TurnType.STRAFE) {
                distance = (power < 0 ? sonarLeft.getDistance() : sonarRight.getDistance());
                op.telemetry.addData("distance", distance);

                if (distance < distanceLimit) {
                    break;
                }
            }
            else if (dir == TurnType.FORWARD) {
                if (power > 0) {
                    distance = sonarFront.getDistance();
                    op.telemetry.addData("distance", distance);
                }
                else {
                    break;
                }

                if (distance < distanceLimit) {
                    break;
                }
            }
            else {
                break;
            }

            if (isAdjust) {
                adjustPower(dir,
                            p);
            }
            else {
                setPower(p,
                         p,
                         dir);
            }

            //op.telemetry.update();
            op.idle();
            timeStamp = System.currentTimeMillis();
        } // while

        if (isBreak) {
            stop2();
        }

        if (Thread.currentThread().isInterrupted()) {
            HwLog.e("Thread interrupted");
        }
        else if (op.isStopRequested()) {
            HwLog.e("Stop requested");
        }

        HwLog.i("Sonar: " + distance);
        HwLog.i("Move took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
        op.idle();
    }

    /**
     * turn the robot
     * @param dir orientation of movement
     * @param deg degrees to turn
     * @param maxPower maximum motor power
     * @param timeoutMillis maximum execution time for this function in milliseconds
     * @param isAccelerate * @param isAccelerate whether to ramp up and down. Use false for very short distance
     * @throws InterruptedException
     */
    public void turn (TurnType dir,
                      float deg,
                      float maxPower,
                      int timeoutMillis,
                      boolean isAccelerate)
        throws InterruptedException
    {
        // how much to ramp up
        final float Kp_up = (dir == TurnType.TURN_REGULAR) ? 0.015f : 0.03f; // 2 per degree
        // how much to ramp down
        final float Kp_down = (dir == TurnType.TURN_REGULAR) ? 0.0075f : 0.015f; // 1 per degree

        deg = deg % 360;

        if (timeoutMillis < 0) {
            // 30 milliseconds per degree
            timeoutMillis = (int)Math.abs(deg * 30) + 2000;
        }

        maxPower = Math.abs(maxPower);
        maxPower = Range.clip(maxPower, 0f, 1f);

        stop2();

        // turn wheels
        setTurnWait(dir);

        setMode2(RunMode.RUN_USING_ENCODER);

        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;

        float heading;
        while (op.opModeIsActive() &&
            timeStamp - startTimestamp < timeoutMillis) {

            heading = gyroSensor.getHeading();
            float delta = deg - heading;
            //if (Math.abs(delta) <= 1)
            if (Math.abs(heading) >= Math.abs(deg))
            {
                // done
                break;
            }

            float power;
            if (!isAccelerate)
            {
                power = Math.signum(delta) * maxPower;
            }
            else {
                if (Math.abs(heading) < Math.abs(deg / 3f)) {
                    // ramp up
                    power = Math.signum(deg) * (Kp_up * Math.abs(heading) + POWER_START);
                } else {
                    // ramp down
                    power = Kp_down * delta + Math.signum(delta) * POWER_STOP;
                }
            }

            power = Range.clip(power, -maxPower, maxPower);
            setPower(power,
                     -power,
                     dir);

            //op.telemetry.addData("heading", Integer.toString(heading));
            //op.telemetry.addData("delta", Integer.toString(delta));
            //op.telemetry.addData("power", Float.toString(power));
            //op.telemetry.addData("counter", Integer.toString(++counter));

            op.idle();
            timeStamp = System.currentTimeMillis();
        } // while

        stop2();

        if (Thread.currentThread().isInterrupted()) {
            HwLog.e("Thread interrupted");
        }
        else if (op.isStopRequested()) {
            HwLog.e("Stop requested");
        }

        heading = gyroSensor.getHeading();
        HwLog.i("Turned " + heading + " degree");
        HwLog.i("Turn took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
        //op.telemetry.addData("heading", heading);

        gyroSensor.adjustHeading(deg);
        op.idle();
    }

    /**
     * course adjust the power
     * @param dir orientation of movement
     * @param p power to adjust with
     */
    private void adjustPower (TurnType dir,
                              float p) {
        float adjust = Math.signum(p) * adjustDirection();
        // with RUN_TO_POSITION sign of power is ignored
        // we have to clip so that (1 + adjust) and (1 - adjust) have the same sign
        adjust = Range.clip(adjust, -1, 1);
        float pwrl = p * (1 + adjust);
        float pwrr = p * (1 - adjust);

        float max = Math.max(Math.abs(pwrl), Math.abs(pwrr));
        if (max > 1f) {
            // scale to 1
            pwrl = pwrl / max;
            pwrr = pwrr / max;
        }
        setPower(pwrl,
                 pwrr,
                 dir);
        /*if (dir == TurnType.STRAFE) {
            long timeStamp = System.currentTimeMillis();
            if (timeStamp - logTimeStamp > 25) {
                HwLog.i("Power left:" + pwrl + " right:" + pwrr);
                logTimeStamp = timeStamp;
            }
        }*/
    }

    /**
     * course adjust the direction
     * @return how much to adjust by
     */
    private float adjustDirection () {

        final float RATIO_SERVO_ADJUST = 30f;

        float deltaHeading = (360f - gyroSensor.getHeading()) % 360f;
        if (deltaHeading > 180f) {
            deltaHeading = deltaHeading - 360f;
        }
        float adjust =  deltaHeading / RATIO_SERVO_ADJUST;

        op.telemetry.addData("delta:", deltaHeading);
        op.telemetry.addData("adjust:", adjust);

        return adjust;
    }

    /**
     * determine space
     * @param dir orientation of movement
     * @param tick encoder ticks
     * @param minSpacing minimum space
     * @return space from an obstacle
     */
    private float getSpacing (TurnType dir,
                              float tick,
                              int minSpacing) {
        float spacing = Float.MAX_VALUE;
        if (minSpacing > 0) {
            if (dir == TurnType.STRAFE) {
                spacing = (tick < 0 ? sonarLeft.getDistance() : sonarRight.getDistance());
            } else if (dir == TurnType.FORWARD) {
                if (tick > 0) {
                    spacing = sonarFront.getDistance();
                }
            }
        }
        op.telemetry.addData("spacing", spacing);
        return spacing;
    }
}
