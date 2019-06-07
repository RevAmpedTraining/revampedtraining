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
            case TURN_SWERVE_FWD_TURN:
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_START);
                servoLeftFront.setPosition((SwerveDriveConstants.SERVO_LEFTFRONT_START + SwerveDriveConstants.SERVO_LEFTFRONT_END)/2f);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_START);
                servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_END);
                break;
            case TURN_SWERVE_FWD_TURN2:
                servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_START);
                servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_END);

                servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_START);
                servoRightFront.setPosition((SwerveDriveConstants.SERVO_RIGHTFRONT_START + SwerveDriveConstants.SERVO_RIGHTFRONT_END)/2f);
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
            default:
                break;
        }
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
                driveLeftFront.setPower(pwrl);
                driveLeftBack.setPower(pwrl);
                driveRightFront.setPower(pwrr);
                driveRightBack.setPower(pwrr);
                break;
            case STRAFE:
                // right is plus
                driveRightFront.setPower(-pwrl);
                driveLeftFront.setPower(pwrl);
                driveRightBack.setPower(pwrr);
                driveLeftBack.setPower(-pwrr);
                break;
            case TURN_SWERVE_FWD_TURN:
                moveFwdTurn(pwrl, 0);
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
