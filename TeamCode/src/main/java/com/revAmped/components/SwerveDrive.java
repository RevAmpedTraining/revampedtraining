package com.revAmped.components;

import com.revAmped.config.SwerveDriveConstants;
import com.revAmped.util.GradualBrake;

/**
 * Class implements the SwerveDrive functionality
 */

public class SwerveDrive
        extends Drive {

    private HwServo servoLeftFront;
    private HwServo servoLeftBack;
    private HwServo servoRightFront;
    private HwServo servoRightBack;

    public final GradualBrake frontGradualBrake;
    public final GradualBrake backGradualBrake;


    protected TurnType wheelOrientation = TurnType.FORWARD;

    public final static int TIME_TURN_WHEEL_SERVO = 800;

    public SwerveDrive(HwMotor driveLeftFront,
                       HwMotor driveLeftBack,
                       HwMotor driveRightFront,
                       HwMotor driveRightBack,
                       HwServo servoLeftFront,
                       HwServo servoLeftBack,
                       HwServo servoRightFront,
                       HwServo servoRightBack)
    {
        super(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack);

        this.frontGradualBrake = new GradualBrake(driveLeftFront,
                                                  driveRightFront);
        this.backGradualBrake = new GradualBrake(driveLeftBack,
                                                 driveRightBack);
        this.servoLeftFront = servoLeftFront;
        this.servoLeftBack = servoLeftBack;
        this.servoRightFront = servoRightFront;
        this.servoRightBack = servoRightBack;
    }

	/*** Turn the robot based on the specified type
	*@param turnType is TurnType
	*/
    public void setTurn(TurnType turnType) {
        if (turnType == wheelOrientation) {
            return;
        }

        wheelOrientation = turnType;
        switch (turnType) {
            case FORWARD:
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_START);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_START);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_START);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_START);
                break;
            case TURN_REGULAR:
                servoLeftBack.setPosition((SwerveDriveConstants.SERVO_LEFTBACK_START + SwerveDriveConstants.SERVO_LEFTBACK_END)/2f);
                servoLeftFront.setPosition((SwerveDriveConstants.SERVO_LEFTFRONT_START + SwerveDriveConstants.SERVO_LEFTFRONT_END)/2f);

                servoRightBack.setPosition((SwerveDriveConstants.SERVO_RIGHTBACK_START + SwerveDriveConstants.SERVO_RIGHTBACK_END)/2f);
                servoRightFront.setPosition((SwerveDriveConstants.SERVO_RIGHTFRONT_START + SwerveDriveConstants.SERVO_RIGHTFRONT_END)/2f);
                break;
            case STRAFE:
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_END);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_END);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_END);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_END);
                break;
            case TURN_LEFT_PIVOT:
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABRIGHT);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABLEFT);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABRIGHT);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABLEFT);
                break;
            case TURN_SWERVE_FWD_TURN:
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABRIGHT+10/255f);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABLEFT+10/255f);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABRIGHT+10/255f);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABLEFT+10/255f);
                break;
            case TURN_SWERVE_FWD_TURN2:
                /*servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_START);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABLEFT-25/255f);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_START);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABLEFT+25f/255f);*/
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_END);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_END);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_STRAFE_CURVE);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_STRAFE_CURVE);
                break;
            case TURN_SWERVE_SIDE_TURN:
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_START);
                servoLeftFront.setPosition((SwerveDriveConstants.SERVO_LEFTFRONT_START + SwerveDriveConstants.SERVO_LEFTFRONT_END)/2f);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_END);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_END);
                break;
            case TURN_SWERVE_SIDE_TURN2:
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_END);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_END);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_START);
                servoRightFront.setPosition((SwerveDriveConstants.SERVO_RIGHTFRONT_START + SwerveDriveConstants.SERVO_RIGHTFRONT_END)/2f);
                break;
            case STRAFE_LEFT_DIAG:
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABLEFT+24/255f);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABLEFT+24/255f);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABLEFT+12/255f);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABLEFT+10/255f);
                break;
            case STRAFE_RIGHT_DIAG:
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABRIGHT-10/255f);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABRIGHT-10/255f);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABRIGHT-20/255f);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABRIGHT-20/255f);
                break;
            case SLIGHT_RIGHT_DIAG:
                servoLeftBack.setPosition((SwerveDriveConstants.SERVO_LEFTBACK_CRABRIGHT
                        +SwerveDriveConstants.SERVO_LEFTBACK_START)/2f);
                servoLeftFront.setPosition((SwerveDriveConstants.SERVO_LEFTFRONT_CRABRIGHT
                        +SwerveDriveConstants.SERVO_LEFTFRONT_START)/2f);

                servoRightBack.setPosition((SwerveDriveConstants.SERVO_RIGHTBACK_CRABRIGHT
                        +SwerveDriveConstants.SERVO_RIGHTBACK_START)/2f);
                servoRightFront.setPosition((SwerveDriveConstants.SERVO_RIGHTFRONT_CRABRIGHT
                        +SwerveDriveConstants.SERVO_RIGHTFRONT_START)/2f);
                break;
            case CURVE:
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_END);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_END);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_STRAFE_CURVE);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_STRAFE_CURVE);
                break;
            case FORWARD_RIGHT:
                //20 will work for double sampling, maybe not center part, change back to 16 4/15/2019
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_START+16/255f);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_START+16/255f);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_START+16/255f);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_START+16/255f);
                break;
            case FORWARD_RIGHT_DS:
                //20 will work for double sampling, maybe not center part, change back to 16 4/15/2019
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_START+21/255f);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_START+21/255f);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_START+21/255f);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_START+21/255f);
                break;
            case FORWARD_LEFT:
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_END);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_END);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_END);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_END);
                break;
            case FORWARD_TM:
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_START);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_START);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_START);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_START);
                break;
            case DIAG_LEFT:
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_START-40/255f);
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_START-40/255f);
                //added 20/255 because want to strafe less diagonally to not hit lander legs 4/9/2019
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_START-50/255f);
                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_START-50/255f);
                break;
            case DIAG_RIGHT:
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABRIGHT);
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABRIGHT);

                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABRIGHT);
                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABRIGHT);
                break;
            case LEFT_CURVE:
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_STRAFE_CURVE);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_STRAFE_CURVE);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_END);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_END);
                break;
            case LEFT_CURVE_1:
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_STRAFE_CURVE-25);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_START);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_END);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_END);
                break;
            default:
                break;
        }
    }

    /*** Turn the robot based on the specified type
     *@param turnType is TurnType
     */
    public void setServo(TurnType turnType) {

        switch (turnType) {
            case FORWARD:
                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_START);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_START);
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_START);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_START);
                break;
            case TURN_REGULAR:
                servoRightBack.setPosition((SwerveDriveConstants.SERVO_RIGHTBACK_START +
                        SwerveDriveConstants.SERVO_RIGHTBACK_END) / 2);
                servoRightFront.setPosition((SwerveDriveConstants.SERVO_RIGHTFRONT_START +
                        SwerveDriveConstants.SERVO_RIGHTFRONT_END) / 2);
                servoLeftBack.setPosition((SwerveDriveConstants.SERVO_LEFTBACK_START +
                        SwerveDriveConstants.SERVO_LEFTBACK_END) / 2);
                servoLeftFront.setPosition((SwerveDriveConstants.SERVO_LEFTFRONT_START +
                        SwerveDriveConstants.SERVO_LEFTFRONT_END) / 2);

                break;
            case STRAFE:
                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_END);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_END);
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_END);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_END);
                break;
            default:
                break;
        }
    }

    public void setPower(float pwrl, float pwrr) {
        driveRightFront.setPower(pwrl);
        driveLeftFront.setPower(pwrl);
        driveRightBack.setPower(-pwrr);
        driveLeftBack.setPower(-pwrr);
    }

    public void setPowerCurve(float pwrb, float pwrf) {
        driveRightFront.setPower(pwrf);
        driveLeftFront.setPower(pwrf);
        driveRightBack.setPower(-pwrb);
        driveLeftBack.setPower(-pwrb);
    }

    /**
     * turn the robot at a specified power
     * @param pwrl motor power left
     * @param pwrr motor power right
     * @param turnType turning method to use
     */
    public void setPower(float pwrl,
                         float pwrr,
                         TurnType turnType) {
        switch (turnType) {
            case FORWARD:
                driveLeftFront.setPower(pwrl);
                driveLeftBack.setPower(pwrl);
                driveRightFront.setPower(pwrr);
                driveRightBack.setPower(pwrr);
                break;
            case TURN_REGULAR:
                driveLeftFront.setPower(-pwrl);
                driveLeftBack.setPower(-pwrl);
                driveRightFront.setPower(-pwrr);
                driveRightBack.setPower(-pwrr);
                break;
            case STRAFE:
                // right is plus
                driveRightFront.setPower(pwrr);
                driveLeftFront.setPower(pwrl);
                driveRightBack.setPower(-pwrr);
                driveLeftBack.setPower(-pwrl);
                break;
            case TURN_SWERVE_FWD_TURN:
                driveRightFront.setPower(pwrr);
                driveLeftFront.setPower(-pwrl);
                driveRightBack.setPower(pwrr);
                driveLeftBack.setPower(-pwrl);
                break;
            case TURN_SWERVE_FWD_TURN2:
                moveFwdTurn(0, pwrr);
                break;
            case TURN_SWERVE_SIDE_TURN:
                moveSideTurn (pwrl, 0);
                break;
            case TURN_SWERVE_SIDE_TURN2:
                moveSideTurn (0, pwrr);
                break;
            case STRAFE_RIGHT_DIAG:
                driveLeftFront.setPower(pwrl);
                driveLeftBack.setPower(pwrl);
                driveRightFront.setPower(pwrr);
                driveRightBack.setPower(pwrr);
                break;
            case STRAFE_LEFT_DIAG:
                driveLeftFront.setPower(pwrl);
                driveLeftBack.setPower(pwrl);
                driveRightFront.setPower(pwrr);
                driveRightBack.setPower(pwrr);
                break;
            case SLIGHT_RIGHT_DIAG:
                driveLeftFront.setPower(pwrl);
                driveLeftBack.setPower(pwrl);
                driveRightFront.setPower(pwrr);
                driveRightBack.setPower(pwrr);
                break;
            case CURVE:
                driveRightFront.setPower(-pwrr);
                driveLeftFront.setPower(pwrl);
                driveRightBack.setPower(-pwrr);
                driveLeftBack.setPower(-pwrl);
                break;
            case TURN_LEFT_PIVOT:
                driveLeftFront.setPower(pwrl);
                driveLeftBack.setPower(-pwrl);
                driveRightFront.setPower(-pwrr);
                driveRightBack.setPower(pwrr);
                break;
            case FORWARD_RIGHT:
                driveLeftFront.setPower(pwrl);
                driveLeftBack.setPower(pwrl);
                driveRightFront.setPower(pwrr);
                driveRightBack.setPower(pwrr);
                break;
            case FORWARD_RIGHT_DS:
                driveLeftFront.setPower(pwrl);
                driveLeftBack.setPower(pwrl);
                driveRightFront.setPower(pwrr);
                driveRightBack.setPower(pwrr);
                break;
            case FORWARD_LEFT:
                driveLeftFront.setPower(pwrl);
                driveLeftBack.setPower(pwrl);
                driveRightFront.setPower(pwrr);
                driveRightBack.setPower(pwrr);
                break;
            case FORWARD_TM:
                driveLeftFront.setPower(pwrl);
                driveLeftBack.setPower(pwrl);
                driveRightFront.setPower(pwrr);
                driveRightBack.setPower(pwrr);
                break;
            case DIAG_LEFT:
                driveLeftFront.setPower(pwrl);
                driveLeftBack.setPower(pwrl);
                driveRightFront.setPower(pwrr);
                driveRightBack.setPower(pwrr);
                break;
            case LEFT_CURVE:
                driveRightFront.setPower(-pwrr);
                driveLeftFront.setPower(pwrl);
                driveRightBack.setPower(-pwrr);
                driveLeftBack.setPower(-pwrl);
                break;
            case LEFT_CURVE_1:
                driveRightFront.setPower(-pwrr);
                driveLeftFront.setPower(pwrl);
                driveRightBack.setPower(-pwrr);
                driveLeftBack.setPower(-pwrl);
                break;
            case DIAG_RIGHT:
                driveLeftFront.setPower(pwrl);
                driveLeftBack.setPower(pwrl);
                driveRightFront.setPower(pwrr);
                driveRightBack.setPower(pwrr);
                break;
            default:
                break;
        }
    }

    /**
     * moves in forward turn
     * @param pwrl power of the left side
     * @param pwrr power of the right side
     */
    private void moveFwdTurn (float pwrl, float pwrr)
    {
        if (pwrl == 0) {
            driveLeftFront.setPower(-pwrr);
            driveRightFront.setPower(pwrr);
        }
        else {
            driveLeftFront.setPower(pwrl);
            driveRightFront.setPower(-pwrl);
        }
        driveLeftBack.setPower(pwrl);
        driveRightBack.setPower(pwrr);
    }

    /**
     * moves in side turn
     * @param pwr2 tx2
     * @param pwr1 tx1
     */
    private void moveSideTurn (float pwr2, float pwr1)
    {
        if (pwr2 == 0) {
            driveLeftFront.setPower(-pwr1);
            driveRightFront.setPower(pwr1);
        }
        else {
            driveLeftFront.setPower(pwr2);
            driveRightFront.setPower(-pwr2);
        }
        driveLeftBack.setPower(pwr2);
        driveRightBack.setPower(pwr1);
    }

    /**
     * get the second largest encoder value of drives
     * use bubble sort
     * @param dir wheel orientation to run in
     * @return the second largest encoder value of drives
     */
    public int getEncoder(TurnType dir) {
        final int NUM_WHEELS = 4;
        // 2nd biggest
        final int RETURN_INDEX = 1;

        int enc[] = new int[NUM_WHEELS];
        enc[0] = Math.abs(driveLeftFront.getCurrentPosition());
        enc[1] = Math.abs(driveRightFront.getCurrentPosition());
        enc[2] = Math.abs(driveRightBack.getCurrentPosition());
        enc[3] = Math.abs(driveLeftBack.getCurrentPosition());

        // bubble sort
        for(int i = 0; i <= RETURN_INDEX; i++) {
            for (int j = i + 1; j < NUM_WHEELS; j++) {
                if (Math.abs(enc[i]) < Math.abs(enc[j])) {
                    // swap
                    int c = enc[i];
                    enc[i] = enc[j];
                    enc[j] = c;
                }
            }
        }
        return enc[RETURN_INDEX];
    }
}
