package com.revAmped.linear.components;

import com.revAmped.components.Drive;
import com.revAmped.components.HwGyro;
import com.revAmped.components.HwMotor;
import com.revAmped.components.TankDrive;
import com.revAmped.components.TurnType;
import com.revAmped.util.HwLog;
import com.revAmped.util.Stalled;
import com.revAmped.linear.util.WaitLinear;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
/**
 * This class is for Autonomous
 */
public class TankDriveLinear
    extends TankDrive
{
    public final HwGyro gyroSensor;

    private HwMotorsLinear motors;

    private LinearOpMode op;

    /** constant for ramping up robot */
    private final static float POWER_STOP = 0.15f;
    private final static float POWER_START = 0.20f;


    public TankDriveLinear (HwMotor driveLeftFront,
                            HwMotor driveLeftBack,
                            HwMotor driveRightFront,
                            HwMotor driveRightBack,
                            HwGyro gyroSensor)
    {
        super(driveLeftFront,
              driveLeftBack,
              driveRightFront,
              driveRightBack);

        this.gyroSensor = gyroSensor;
        this.motors = new HwMotorsLinear(this.motorsArray);
    }

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
     * set motor target position until it is confirmed
     * @param position motor target position
     */
    public void setTargetPosition2(int position)
        throws InterruptedException
    {
        motors.setTargetPosition(position);
    }

    public void moveToEncoderInch(int distanceInch,
                                  float maxPower,
                                  int timeoutMillis,
                                  boolean isAccelerate,
                                  boolean isBreak)
        throws InterruptedException
    {
        int tick = inchToTick(distanceInch);
        moveToEncoder(tick,
                      maxPower,
                      timeoutMillis,
                      isAccelerate,
                      isBreak,
                      null);
    }

    public void moveToEncoderInch(int distanceInch,
                                  float maxPower,
                                  int timeoutMillis,
                                  boolean isAccelerate,
                                  boolean isBreak,
                                  MoveAction... actions)
        throws InterruptedException
    {
        int tick = inchToTick(distanceInch);
        moveToEncoder(tick,
                      maxPower,
                      timeoutMillis,
                      isAccelerate,
                      isBreak,
                      actions);
    }

    public void moveToEncoder(int tick,
                              float maxPower,
                              int timeoutMillis,
                              boolean isAccelerate,
                              boolean isBreak,
                              MoveAction[] actions)
        throws InterruptedException
    {
        final float MOVE_TIMEOUT_FACTOR = 2f;
        final int MOVE_STALL_TIME = 1500;

        maxPower = Math.abs(maxPower);
        maxPower = Range.clip(maxPower, 0f, 1f);

        if (timeoutMillis < 0) {
            timeoutMillis = Math.abs((int) (tick * Drive.AM40_MILLISECOND_PER_TICK * MOVE_TIMEOUT_FACTOR)) + 2000;
        }

        resetPosition();
        // is RUN_TO_POSITION
        setMode2(RunMode.RUN_TO_POSITION);
        setTargetPosition2(tick);

        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;

        Stalled stalled = new Stalled();
        while (op.opModeIsActive() &&
            // is RUN_TO_POSITION
            isBusy() &&
            (timeStamp - startTimestamp < timeoutMillis)) {

            int average = getEncoder();
            int delta = tick - average;
            if (Math.abs(delta) < 30)
            {
                // done
                break;
            }

            float basePower;
            if (!isAccelerate)
            {
                basePower = Math.signum(delta) * (maxPower);
            }
            else
            {
                if (Math.abs(average) < Math.abs(tick / 3))
                {
                    // ramp up for 1/3 of distance
                    // minimum power is START_POWER
                    basePower = (average/1600f) + Math.signum(tick) * (POWER_START);
                }
                else
                {
                    // ramp down for 2/3 of distance
                    // minimum power is START_POWER
                    basePower = (delta/3200f) + Math.signum(delta) * (POWER_STOP);
                }
                basePower = Range.clip(basePower, -maxPower, maxPower);

            } // if

            op.telemetry.addData("power", Float.toString(basePower));
            op.telemetry.addData("travel", Integer.toString(tickToInch(average)));
            op.telemetry.addData("delta", Integer.toString(tickToInch(delta)));

            if (stalled.isStalled(average, timeStamp)) {
                WaitLinear w = new WaitLinear(op);
                w.waitMillis(MOVE_STALL_TIME);
                timeoutMillis += MOVE_STALL_TIME;
                stalled.reset();
                stop2();
            }
            else {
                setPower(basePower);
            }

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

            op.idle();
            timeStamp = System.currentTimeMillis();
        } // while

        if (isBreak) {
            stop2();
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

        int inches = tickToInch(getEncoder());
        HwLog.i("Moved " + inches + " inches");
        HwLog.i("Move took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
        op.telemetry.addData("travel", inches);
        op.idle();
    }

    public void moveToTime (float power,
                            int timeoutMillis)
        throws InterruptedException
    {
        final float p = Range.clip(power, -1f, 1f);

        stop2();

        setMode2(RunMode.RUN_WITHOUT_ENCODER);

        WaitLinear lp = new WaitLinear(op);
        lp.waitMillis(timeoutMillis,
                      new WaitLinear.WakeUp() {
                          public boolean isWakeUp() {
                              // in case skipping
                              setPower(p);
                              return false;
                          }
                      });

        stop2();
        setMode2(RunMode.RUN_USING_ENCODER);
    }

    public void turn (int deg,
                      float maxPower,
                      int timeoutMillis,
                      boolean isAccelerate,
                      TurnType turnType)
        throws InterruptedException
    {
        // how much to ramp up
        final float Kp_up = turnType == TurnType.TURN_REGULAR ? 0.02f : 0.04f; // 2 per degree
        // how much to ramp down
        final float Kp_down = turnType == TurnType.TURN_REGULAR ? 0.01f : 0.02f; // 1 per degree

        deg = deg % 360;

        if (timeoutMillis < 0) {
            // 30 milliseconds per degree
            timeoutMillis = Math.abs(deg * 30) + 2000;
        }

        maxPower = Math.abs(maxPower);
        maxPower = Range.clip(maxPower, 0f, 1f);

        stop2();

        setMode2(RunMode.RUN_USING_ENCODER);

        gyroSensor.resetHeading();
        op.idle();

        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;

        int counter = 0;
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
                power = Math.signum(delta) * (maxPower);
            }
            else {
                if (Math.abs(heading) < Math.abs(deg / 3f)) {
                    // ramp up
                    power = Math.signum(deg) * (Kp_up * Math.abs(heading) + POWER_START);
                    //power = Math.signum(deg) * (maxPower);
                } else {
                    // ramp down
                    power = Kp_down * delta + Math.signum(delta) * POWER_STOP;
                }
            }

            power = Range.clip(power, -maxPower, maxPower);
            setPower(power,
                     power,
                     turnType);

            op.telemetry.addData("heading", Float.toString(heading));
            op.telemetry.addData("delta", Float.toString(delta));
            op.telemetry.addData("power", Float.toString(power));
            op.telemetry.addData("counter", Integer.toString(++counter));

            op.idle();
            timeStamp = System.currentTimeMillis();
        } // while

        stop2();

        heading = gyroSensor.getHeading();
        HwLog.i("Turned " + heading + " degree");
        HwLog.i("Turn took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
        op.telemetry.addData("heading", heading);
        op.idle();
    }


    public void turn (int deg,
                      float maxPower,
                      int timeoutMillis,
                      boolean isAccelerate)
        throws InterruptedException {
        turn(deg,
             maxPower,
             timeoutMillis,
             isAccelerate,
             TurnType.TURN_REGULAR);
    }
}
