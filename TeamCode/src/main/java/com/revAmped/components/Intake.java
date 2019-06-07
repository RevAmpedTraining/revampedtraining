package com.revAmped.components;

import static com.revAmped.components.Intake.FixState.START;
import static com.revAmped.components.Intake.FixState.FIN;
import static com.revAmped.components.Intake.FixState.FOUT;

/**
 * Created by Parthiv Nair on 1/2/2018.
 */

public class Intake {

    ///Left motor of the intake system
    public final HwMotor left;
    ///Right motor of the intake system
    public final HwMotor right;
    ///Power used to intake
    public final float power = 0.8f;
    ///Power used to reject
    public final float outpower = 0.55f;
    ///Overcharged robot
    public RobotOvercharged2 robot;
    ///Maintain the global fix state
    public Intake.FixState fixState = START;
    private long fixTimeStamp;

    /**
     * Initialize the intake system
     */
    public Intake(HwMotor hwMotorL, HwMotor hwMotorR, RobotOvercharged2 rob)
    {
        this.left = hwMotorL;
        this.right = hwMotorR;
        this.robot = rob;
    }

    /**
     * Intake type
     */
    public enum IntakeType {
        IN,
        OUT,
        FIX,
        STOP;
    }

    ///Maintain the state we are in during the Fix
    public enum FixState {
        START,
        FIN,
        FOUT,
        STOP;
    }

    /**
     * Collect the glyphs
     */
    public void collect()
    {
        setIntake(power, IntakeType.IN);
    }

    /**
     * Reject the glyphs
     */
    public void reject()
    {
        setIntake(power, IntakeType.OUT);
    }

    /**
     * Fix the intake by rejecting and then collecting
     */
    public void fix()
    {
        switch (fixState) {
            case START:
                fixTimeStamp = System.currentTimeMillis();
                fixState = FOUT;
                break;
            case FOUT:
                left.setPower(-outpower);
                right.setPower(-outpower);
                if (System.currentTimeMillis() - fixTimeStamp > 350) {
                    fixState = FIN;
                    fixTimeStamp = System.currentTimeMillis();
                }
                break;
            case FIN:
                left.setPower(power);
                right.setPower(power);
                if (System.currentTimeMillis() - fixTimeStamp > 350) {
                    fixState = START;
                }
                break;
        }
    }

    /**
     * stop the intake
     */
    public void stop()
    {
        left.setPower(0f);
        right.setPower(0f);
        robot.ledRed.off();
        robot.ledGreen.off();
    }

    /**
     * turn the collection system at a specified power and direction
     * @param pwr motor power
     * @param intakeType the type of intake to use see IntakeType (IN, OUT or FIX)
     */
    private void setIntake(float pwr,
                           IntakeType intakeType) {
        switch (intakeType) {
            case IN:
                left.setPower(pwr);
                right.setPower(pwr);
                robot.ledRed.off();
                robot.ledGreen.on();
                break;
            case OUT:
                left.setPower(-outpower);
                right.setPower(-outpower);
                robot.ledRed.on();
                robot.ledGreen.off();
                break;
            case FIX:
                left.setPower(pwr);
                right.setPower(-pwr);
                robot.ledRed.off();
                robot.ledGreen.off();
                break;
            default:
                break;
        }
    }
}
