package com.revAmped.linear.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.revAmped.components.Drive;
import com.revAmped.components.HwDevice;
import com.revAmped.components.HwGyro;
import com.revAmped.components.HwMotor;
import com.revAmped.components.HwServo;
import com.revAmped.components.HwSonarAnalog;
import com.revAmped.components.HwSwitch;
import com.revAmped.components.SwerveDrive;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotConstants.COLOR_SENSOR;
import com.revAmped.linear.util.WaitLinear;
import com.revAmped.sensors.MultiplexColorSensor;
import com.revAmped.util.Stalled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

/**
 * This class is for AutoRevamped
 */
public class SwerveDriveLinear
    extends SwerveDrive
{
    public final HwGyro gyroSensor;
    public final HwSonarAnalog sonarFront, sonarLeft, sonarRight;
    public final MultiplexColorSensor colorSensor;
    public final HwMotor  motorLatch, motorPopper, motorIntake;
    public final HwSwitch switchSlideDown;
    public final CRServo servoTelescopeR, servoTelescopeL;

    private HwMotorsLinear motors;

    private LinearOpMode op;

    private long  logTimeStamp = 0;

    /** constant for ramping up robot */
    private final static float POWER_STOP = 0.10f;
    private final static float POWER_START = 0.17f;

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
                             MultiplexColorSensor colorSensor,
                             CRServo servoTelescopeR,
                             CRServo servoTelescopeL,
                             HwMotor motorLatch,
                             HwMotor motorPopper,
                             HwMotor motorIntake,
                             HwSwitch switchSlideDown)
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
        this.servoTelescopeL = servoTelescopeL;
        this.servoTelescopeR = servoTelescopeR;
        this.motorLatch = motorLatch;
        this.motorPopper = motorPopper;
        this.motorIntake = motorIntake;
        this.switchSlideDown = switchSlideDown;
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
    /**
    public void moveToEncoderInch(TurnType dir,
                                  float distanceInch,
                                  float maxPower,
                                  int timeoutMillis,
                                  boolean isAccelerate,
                                  boolean isAdjust,
                                  boolean isBreak)
            throws InterruptedException
    {
        float tick = inchToTick(distanceInch);
        moveToEncoder(dir,
                tick,
                maxPower,
                timeoutMillis,
                isAccelerate,
                isAdjust,
                isBreak,
                null);
    }
    **/
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
                      isAccelerate,
                      isAdjust,
                      isBreak,
                      null);
    }

    public void moveToEncoderInch(TurnType dir,
                                  int distanceInch,
                                  float maxPower,
                                  int timeoutMillis,
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
                              boolean isAccelerate,
                              boolean isAdjust,
                              boolean isBreak,
                              MoveAction[] actions)
        throws InterruptedException
    {
        final float MOVE_TIMEOUT_FACTOR = 2f;
        final int MOVE_STALL_TIME = 500;

        maxPower = Math.abs(maxPower);
        maxPower = Range.clip(maxPower, 0f, 1f);

        if (timeoutMillis < 0) {
            timeoutMillis = Math.abs((int) (tick * Drive.AM20_MILLISECOND_PER_TICK * MOVE_TIMEOUT_FACTOR)) + 2000;
        }

        // turn wheels
        setTurnWait(dir);

        resetPosition();
        // is RUN_TO_POSITION
        /*
        might be right back left back and right front left front due to weird stuffs.
        observed 11/11/2018 and will test later
        john wang
         */
        setMode2(RunMode.RUN_TO_POSITION);
        if (dir == TurnType.STRAFE || dir == TurnType.CURVE) {
            setTargetPosition2(tick,
                               new HwDevice[]{driveRightBack, driveLeftFront});
            setTargetPosition2(-tick,
                               new HwDevice[]{driveRightFront, driveLeftBack});
        } else if (dir ==TurnType.TURN_SWERVE_FWD_TURN){
            setTargetPosition2(tick,
                    new HwDevice[]{driveRightBack, driveLeftBack});
            setTargetPosition2(-tick,
                    new HwDevice[]{driveRightFront, driveLeftFront});
        } else {
            setTargetPosition2(-tick);

        }

        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;
        boolean isSwitchDown = switchSlideDown.isTouch();
        Stalled stalled = new Stalled();
        while (!op.isStopRequested() && op.opModeIsActive() &&
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
                if (Math.abs(average) < Math.abs(tick / 3))
                {
                    // ramp up for 1/3 of distance
                    // minimum power is START_POWER
                    // base is 1600
                    basePower = (average/1600f) + Math.signum(tick) * POWER_START;
                }
                else
                {
                    // ramp down for 2/3 of distance
                    // minimum power is START_POWER
                    // base is 3200
                    basePower = (delta/3200f) + Math.signum(delta) * POWER_STOP;
                }
                basePower = Range.clip(basePower, -maxPower, maxPower);

                /*if (dir == TurnType.STRAFE) {
                    long timeStamp1 = System.currentTimeMillis();
                    if (timeStamp1 - logTimeStamp > 25) {
                        RobotLog.i("Avg:" + average + " Delta:" + delta + " Power:" + basePower);
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
                if (isAdjust) {
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
            RobotLog.e("RevAmped Thread interrupted");
        }
        else if (op.isStopRequested()) {
            RobotLog.e("RevAmped Stop requested");
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
                RobotLog.i("Actions " + action + " stopped");
            }
            op.idle();
        }

        int inches = tickToInch(getEncoder(dir));
        RobotLog.i("Moved " + inches + " inches");
        RobotLog.i("Move took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
        //op.telemetry.addData("travel", inches);
        op.idle();
    }
    /**
     * move wheels according to motor encoder ticks
     * @param dir orientation of movement
     * @param distanceInch the distance to move in motor encoder tick
     * @param maxPower maximum motor power
     * @param timeoutMillis maximum execution time for this function in milliseconds
     * @param isAccelerate whether to ramp up and down. Use false for very short distance
     * @param isBreak whether to break
     */
    public void moveToEncoderCrater(TurnType dir,
                                    int distanceInch,
                                    float maxPower,
                                    int timeoutMillis,
                                    boolean isAccelerate,
                                    boolean isAdjust,
                                    boolean isBreak,
                                    boolean lowerLatch,
                                    boolean pop)
            throws InterruptedException
    {
        int tick = inchToTick(distanceInch);
        final float MOVE_TIMEOUT_FACTOR = 2f;
        final int MOVE_STALL_TIME = 500;

        maxPower = Math.abs(maxPower);
        maxPower = Range.clip(maxPower, 0f, 1f);

        if (timeoutMillis < 0) {
            timeoutMillis = Math.abs((int) (tick * Drive.AM20_MILLISECOND_PER_TICK * MOVE_TIMEOUT_FACTOR)) + 2000;
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
        } else if (dir ==TurnType.TURN_SWERVE_FWD_TURN){
            setTargetPosition2(tick,
                    new HwDevice[]{driveRightBack, driveLeftBack});
            setTargetPosition2(-tick,
                    new HwDevice[]{driveRightFront, driveLeftFront});
        } else if (dir==TurnType.CURVE) {
            setTargetPosition2(-tick,
                    new HwDevice[]{driveRightBack, driveLeftBack, driveRightFront});
            setTargetPosition2(tick,
                    new HwDevice[]{driveLeftFront});
        }

        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;
        boolean isSwitchDown = switchSlideDown.isTouch();
        Stalled stalled = new Stalled();
        int startAverage = ((int)Math.signum(tick))*getEncoder(dir);
        int startDelta = tick - startAverage;
        int i = 0;
        if (lowerLatch) {
            isSwitchDown = switchSlideDown.isTouch();
            if (!isSwitchDown) motorLatch.setPower(-1);
        } else if (pop) {
            motorPopper.setPower(1);
            motorIntake.setPower(1);
        }
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
            if (dir==TurnType.CURVE) {
                if (Math.abs(delta) < Math.abs((startDelta / 1.25))) {
                    setTargetPosition2(tick,
                            new HwDevice[]{driveRightBack, driveLeftFront});
                    setTargetPosition2(-tick,
                            new HwDevice[]{driveRightFront, driveLeftBack});
                    dir = TurnType.STRAFE;
                    setTurn(dir);
                    maxPower = 0.55f;
                }
            }
            float basePower;
            if (!isAccelerate)
            {
                basePower = Math.signum(delta) * maxPower;
            }
            else
            {
                if (Math.abs(average) < Math.abs(tick / 1.5))
                {
                    // ramp up for 1/3 of distance
                    // minimum power is START_POWER
                    // base is 1600
                    basePower = Math.signum(delta) * maxPower;

                    //basePower = (average/1600f) + Math.signum(tick) * POWER_START;
                }
                else
                {
                    // ramp down for 2/3 of distance
                    // minimum power is START_POWER
                    // base is 3200
                    basePower = (delta/3200f) + Math.signum(delta) * POWER_STOP;
                }
                basePower = Range.clip(basePower, -maxPower, maxPower);

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
                if (isAdjust) {
                    adjustPower(dir,
                            basePower);
                }
                else {
                    setPower(basePower,
                            basePower,
                            dir);
                }
            } // else


            //op.telemetry.addData("driveLeftFront", driveLeftFront.getCurrentPosition());
            //op.telemetry.addData("driveRightBack", driveRightBack.getCurrentPosition());
            //op.telemetry.addData("driveRightFront", driveRightFront.getCurrentPosition());
            //op.telemetry.addData("driveLeftBack", driveLeftBack.getCurrentPosition());

            //op.telemetry.update();
            op.idle();
            timeStamp = System.currentTimeMillis();

            op.telemetry.addLine("here");
            op.telemetry.update();
        } // while

        if (isBreak) {
            stop2();
        }

        if (Thread.currentThread().isInterrupted()) {
            RobotLog.e("RevAmped Thread interrupted");
        }
        else if (op.isStopRequested()) {
            RobotLog.e("RevAmped Stop requested");
        }

        // is RUN_TO_POSITION
        setMode2(RunMode.RUN_USING_ENCODER);

        // stop actions
        if (lowerLatch) {
            motorLatch.setPower(0);
        } else if (pop) {
            motorIntake.setPower(0);
            motorPopper.setPower(0);
        }

        int inches = tickToInch(getEncoder(dir));
        RobotLog.i("Moved " + inches + " inches");
        RobotLog.i("Move took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
        //op.telemetry.addData("travel", inches);
        op.idle();
    }

    /**
     * move wheels according to motor encoder ticks
     * @param dir1 orientation of movement in first part
     * @param dir2 orientation of movement in second part
     * @param deg amount to turn in the first part
     * @param distanceInch the distance to move in motor encoder tick
     * @param maxPower maximum motor power
     * @param timeoutMillis maximum execution time for this function in milliseconds
     * @param isAccelerate whether to ramp up and down. Use false for very short distance
     * @param isBreak whether to break
     */
    public void moveToEncoderDepot(TurnType dir1,
                              TurnType dir2,
                              float deg,
                              int distanceInch,
                              float maxPower,
                              int timeoutMillis,
                              int timeoutMillis1,
                              boolean isAccelerate,
                              boolean isAdjust,
                              boolean isBreak)
            throws InterruptedException
    {
            // how much to ramp up
            final float Kp_up = (dir1 == (TurnType.TURN_REGULAR) || dir1 == TurnType.TURN_SWERVE_FWD_TURN) ? 0.015f : 0.03f; // 2 per degree
            // how much to ramp down
            final float Kp_down = (dir1 == (TurnType.TURN_REGULAR) || dir1 == TurnType.TURN_SWERVE_FWD_TURN) ? 0.0075f : 0.015f; // 1 per degree

            deg = deg % 360;

            if (timeoutMillis < 0) {
                // 30 milliseconds per degree
                timeoutMillis = (int)Math.abs(deg * 30) + 2000;
            }

            maxPower = Math.abs(maxPower);
            maxPower = Range.clip(maxPower, 0f, 1f);

            stop2();

            // turn wheels
            setTurnWait(dir1);

            setMode2(RunMode.RUN_USING_ENCODER);

            long startTimestamp = System.currentTimeMillis();
            long timeStamp = startTimestamp;

            float heading;
            gyroSensor.resetHeading();
            while (!op.isStopRequested() && op.opModeIsActive() &&
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
                //BNO055 left turn is positive
                setPower(-power,
                        power,
                        dir1);

                //op.telemetry.addData("heading", Integer.toString(heading));
                //op.telemetry.addData("delta", Integer.toString(delta));
                //op.telemetry.addData("power", Float.toString(power));
                //op.telemetry.addData("counter", Integer.toString(++counter));

                op.idle();
                timeStamp = System.currentTimeMillis();
            } // while
        int tick = inchToTick(distanceInch);
        final float MOVE_TIMEOUT_FACTOR = 2f;
        final int MOVE_STALL_TIME = 500;

        maxPower = Math.abs(maxPower);
        maxPower = Range.clip(maxPower, 0f, 1f);

        if (timeoutMillis1 < 0) {
            timeoutMillis1 = Math.abs((int) (tick * Drive.AM20_MILLISECOND_PER_TICK * MOVE_TIMEOUT_FACTOR)) + 2000;
        }
        setPower(0);
        // turn wheels

        resetPosition();
        // is RUN_TO_POSITION
        /*
        might be right back left back and right front left front due to weird stuffs.
        observed 11/11/2018 and will test later
        john wang
         */
        setMode2(RunMode.RUN_TO_POSITION);
        if (dir2 == TurnType.STRAFE || dir2 == TurnType.CURVE) {
            setTargetPosition2(tick,
                    new HwDevice[]{driveRightBack, driveLeftFront});
            setTargetPosition2(-tick,
                    new HwDevice[]{driveRightFront, driveLeftBack});
        } else if (dir2 ==TurnType.TURN_SWERVE_FWD_TURN){
            setTargetPosition2(tick,
                    new HwDevice[]{driveRightBack, driveLeftBack});
            setTargetPosition2(-tick,
                    new HwDevice[]{driveRightFront, driveLeftFront});
        } else {
            setTargetPosition2(-tick);

        }
        setTurn(dir2);
        startTimestamp = System.currentTimeMillis();
        timeStamp = startTimestamp;
        boolean isSwitchDown = switchSlideDown.isTouch();
        Stalled stalled = new Stalled();
        while (op.opModeIsActive() &&
                // is RUN_TO_POSITION
                isBusy() &&
                (timeStamp - startTimestamp < timeoutMillis1)) {

            int average = ((int)Math.signum(tick))*getEncoder(dir2);
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
                if (Math.abs(average) < Math.abs(tick / 3))
                {
                    // ramp up for 1/3 of distance
                    // minimum power is START_POWER
                    // base is 1600
                    basePower = (average/1600f) + Math.signum(tick) * POWER_START;
                }
                else
                {
                    // ramp down for 2/3 of distance
                    // minimum power is START_POWER
                    // base is 3200
                    basePower = (delta/3200f) + Math.signum(delta) * POWER_STOP;
                }
                basePower = Range.clip(basePower, -maxPower, maxPower);

                /*if (dir == TurnType.STRAFE) {
                    long timeStamp1 = System.currentTimeMillis();
                    if (timeStamp1 - logTimeStamp > 25) {
                        RobotLog.i("Avg:" + average + " Delta:" + delta + " Power:" + basePower);
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
                if (isAdjust) {
                    adjustPower(dir2,
                            basePower);
                }
                else {
                    setPower(basePower,
                            basePower,
                            dir2);
                }
            } // else

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
            RobotLog.e("RevAmped Thread interrupted");
        }
        else if (op.isStopRequested()) {
            RobotLog.e("RevAmped Stop requested");
        }

        // is RUN_TO_POSITION
        setMode2(RunMode.RUN_USING_ENCODER);

        // stop actions
        int inches = tickToInch(getEncoder(dir2));
        RobotLog.i("Moved " + inches + " inches");
        RobotLog.i("Move took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
        //op.telemetry.addData("travel", inches);
        op.idle();
    }
    /**
     * move wheels according to motor encoder ticks
     * @param dir1 orientation of movement in first part
     * @param dir2 orientation of movement in second part
     * @param deg amount to turn in the first part
     * @param distanceInch the distance to move in motor encoder tick
     * @param maxPower maximum motor power
     * @param timeoutMillis maximum execution time for this function in milliseconds
     * @param isAccelerate whether to ramp up and down. Use false for very short distance
     * @param isBreak whether to break
     */
    public void moveToEncoderDepot1(TurnType dir1,
                                   TurnType dir2,
                                   float deg,
                                   int distanceInch,
                                   float maxPower,
                                   int timeoutMillis,
                                   int timeoutMillis1,
                                   boolean isAccelerate,
                                   boolean isAdjust,
                                   boolean isBreak)
            throws InterruptedException
    {

        int tick = inchToTick(distanceInch);
        final float MOVE_TIMEOUT_FACTOR = 2f;
        final int MOVE_STALL_TIME = 500;

        maxPower = Math.abs(maxPower);
        maxPower = Range.clip(maxPower, 0f, 1f);

        if (timeoutMillis1 < 0) {
            timeoutMillis1 = Math.abs((int) (tick * Drive.AM20_MILLISECOND_PER_TICK * MOVE_TIMEOUT_FACTOR)) + 2000;
        }
        setPower(0);
        // turn wheels

        resetPosition();
        // is RUN_TO_POSITION
        /*
        might be right back left back and right front left front due to weird stuffs.
        observed 11/11/2018 and will test later
        john wang
         */
        setMode2(RunMode.RUN_TO_POSITION);
        if (dir2 == TurnType.STRAFE || dir2 == TurnType.CURVE) {
            setTargetPosition2(tick,
                    new HwDevice[]{driveRightBack, driveLeftFront});
            setTargetPosition2(-tick,
                    new HwDevice[]{driveRightFront, driveLeftBack});
        } else if (dir2 ==TurnType.TURN_SWERVE_FWD_TURN){
            setTargetPosition2(tick,
                    new HwDevice[]{driveRightBack, driveLeftBack});
            setTargetPosition2(-tick,
                    new HwDevice[]{driveRightFront, driveLeftFront});
        } else {
            setTargetPosition2(-tick);

        }
        setTurn(dir2);
        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;
        boolean isSwitchDown = switchSlideDown.isTouch();
        Stalled stalled = new Stalled();
        while (op.opModeIsActive() &&
                // is RUN_TO_POSITION
                isBusy() &&
                (timeStamp - startTimestamp < timeoutMillis1)) {

            int average = ((int)Math.signum(tick))*getEncoder(dir2);
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
                if (Math.abs(average) < Math.abs(tick / 3))
                {
                    // ramp up for 1/3 of distance
                    // minimum power is START_POWER
                    // base is 1600
                    basePower = (average/1600f) + Math.signum(tick) * POWER_START;
                }
                else
                {
                    // ramp down for 2/3 of distance
                    // minimum power is START_POWER
                    // base is 3200
                    basePower = (delta/3200f) + Math.signum(delta) * POWER_STOP;
                }
                basePower = Range.clip(basePower, -maxPower, maxPower);

                /*if (dir == TurnType.STRAFE) {
                    long timeStamp1 = System.currentTimeMillis();
                    if (timeStamp1 - logTimeStamp > 25) {
                        RobotLog.i("Avg:" + average + " Delta:" + delta + " Power:" + basePower);
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
                if (isAdjust) {
                    adjustPower(dir2,
                            basePower);
                }
                else {
                    setPower(basePower,
                            basePower,
                            dir2);
                }
            } // else

            //op.telemetry.addData("driveLeftFront", driveLeftFront.getCurrentPosition());
            //op.telemetry.addData("driveRightBack", driveRightBack.getCurrentPosition());
            //op.telemetry.addData("driveRightFront", driveRightFront.getCurrentPosition());
            //op.telemetry.addData("driveLeftBack", driveLeftBack.getCurrentPosition());

            //op.telemetry.update();
            op.idle();
            timeStamp = System.currentTimeMillis();
        } // while
        // how much to ramp up
        final float Kp_up = (dir1 == (TurnType.TURN_REGULAR) || dir1 == TurnType.TURN_SWERVE_FWD_TURN) ? 0.015f : 0.03f; // 2 per degree
        // how much to ramp down
        final float Kp_down = (dir1 == (TurnType.TURN_REGULAR) || dir1 == TurnType.TURN_SWERVE_FWD_TURN) ? 0.0075f : 0.015f; // 1 per degree

        deg = deg % 360;

        if (timeoutMillis < 0) {
            // 30 milliseconds per degree
            timeoutMillis = (int)Math.abs(deg * 30) + 2000;
        }

        maxPower = Math.abs(maxPower);
        maxPower = Range.clip(maxPower, 0f, 1f);

        stop2();

        // turn wheels
        setTurnWait(dir1);

        setMode2(RunMode.RUN_USING_ENCODER);

        startTimestamp = System.currentTimeMillis();
        timeStamp = startTimestamp;

        float heading;
        gyroSensor.resetHeading();
        while (!op.isStopRequested() && op.opModeIsActive() &&
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
            //BNO055 left turn is positive
            setPower(-power,
                    power,
                    dir1);

            //op.telemetry.addData("heading", Integer.toString(heading));
            //op.telemetry.addData("delta", Integer.toString(delta));
            //op.telemetry.addData("power", Float.toString(power));
            //op.telemetry.addData("counter", Integer.toString(++counter));

            op.idle();
            timeStamp = System.currentTimeMillis();
        } // while
        if (isBreak) {
            stop2();
        }

        if (Thread.currentThread().isInterrupted()) {
            RobotLog.e("RevAmped Thread interrupted");
        }
        else if (op.isStopRequested()) {
            RobotLog.e("RevAmped Stop requested");
        }

        // is RUN_TO_POSITION
        setMode2(RunMode.RUN_USING_ENCODER);

        // stop actions
        int inches = tickToInch(getEncoder(dir2));
        RobotLog.i("Moved " + inches + " inches");
        RobotLog.i("Move took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
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

        WaitLinear lp = new WaitLinear(op);
        lp.waitMillis(timeoutMillis,
                      new WaitLinear.WakeUp() {
                          public boolean isWakeUp() {
                              // in case skipping
                              if (dir==TurnType.TURN_SWERVE_FWD_TURN) {
                                  setPower(p, p, TurnType.TURN_SWERVE_FWD_TURN);
                              } else if (dir==TurnType.TURN_REGULAR){
                                  setPower(p, p, TurnType.TURN_REGULAR);
                              } else {
                                  setPower(p);
                              }
                              return false;
                          }
                      });

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
     * @param isAdjust whether to course adjust or not
     * @param isBreak whether to break
     * @throws InterruptedException
     */
    public void moveToLine (final TurnType dir,
                            COLOR_SENSOR colorPort,
                            float power,
                            int timeoutMillis,
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

            if (isAdjust) {
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
            RobotLog.e("Thread interrupted");
        }
        else if (op.isStopRequested()) {
            RobotLog.e("Stop requested");
        }

        RobotLog.i("Lux: " + lux);
        RobotLog.i("Move took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
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
                //op.telemetry.addData("distance", distance);

                if (distance < distanceLimit) {
                    break;
                }
            }
            else if (dir == TurnType.FORWARD) {
                if (power > 0) {
                    distance = sonarFront.getDistance();
                    //op.telemetry.addData("distance", distance);
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
            RobotLog.e("Thread interrupted");
        }
        else if (op.isStopRequested()) {
            RobotLog.e("Stop requested");
        }

        RobotLog.i("Sonar: " + distance);
        RobotLog.i("Move took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
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
        while (!op.isStopRequested() && op.opModeIsActive() &&
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
            //BNO055 left turn is positive
            setPower(-power,
                     power,
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
            RobotLog.e("OverCharged Thread interrupted");
        }
        else if (op.isStopRequested()) {
            RobotLog.e("OverCharged Stop requested");
        }

        heading = gyroSensor.getHeading();
        RobotLog.i("Turned " + heading + " degree");
        RobotLog.i("Turn took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
        //op.telemetry.addData("heading", heading);

        gyroSensor.adjustHeading(deg);
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
    public void turn2 (TurnType dir,
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
        gyroSensor.resetHeading();
        while (!op.isStopRequested() && op.opModeIsActive() &&
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
            //BNO055 left turn is positive
            setPower(-power,
                    power,
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
            RobotLog.e("RevAmped Thread interrupted");
        }
        else if (op.isStopRequested()) {
            RobotLog.e("RevAmped Stop requested");
        }

        heading = gyroSensor.getHeading();
        RobotLog.i("Turned " + heading + " degree");
        RobotLog.i("Turn took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
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
        float adjust = Math.signum(p) * gyroSensor.adjustDirection();
        // with RUN_TO_POSITION sign of power is ignored
        // we have to clip so that (1 + adjust) and (1 - adjust) have the same sign
        adjust = Range.clip(adjust, -1, 1);
        // BNO055 positive for left turn
        float pwrl = p * (1 - adjust);
        float pwrr = p * (1 + adjust);

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
                RobotLog.i("Power left:" + pwrl + " right:" + pwrr);
                logTimeStamp = timeStamp;
            }
        }*/
    }
}
