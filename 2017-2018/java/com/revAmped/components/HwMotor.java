package com.revAmped.components;

import com.qualcomm.robotcore.hardware.DcMotor;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by zwang on 2/7/2016.
 */
public class HwMotor
    extends HwDevice {

    private DcMotor motor;
    private int encoderBase = 0;

    /**
     * initialize the motor
     * @param hardwareMap HardwareMap to get motor from
     * @param id name of motor
     * @param direction direction of motor
     */
    public HwMotor (HardwareMap hardwareMap,
                    String id,
                    DcMotor.Direction direction)
            throws IllegalArgumentException
    {
        this(hardwareMap,
             id,
             direction,
             RUN_USING_ENCODER);
    }

    /**
     * initialize the motor
     * @param hardwareMap HardwareMap to get motor from
     * @param id name of motor
     * @param direction direction of motor
     * @param runMode runMode of motor
     */
    public HwMotor (HardwareMap hardwareMap,
                    String id,
                    DcMotor.Direction direction,
                    DcMotor.RunMode runMode)
        throws IllegalArgumentException
    {
        super(id);
        this.motor = hardwareMap.dcMotor.get(id);
        this.motor.setDirection(direction);
        this.motor.setMode(runMode);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public DcMotor getMotor () {
       return motor;
   }

    /**
     * set motor power
     * @param power power to set the motor to
     */
    public void setPower(float power) {
        motor.setPower(power);
    }

    /**
     * @return motor power
     */
    public float getPower() {
        return (float) motor.getPower();
    }

    /**
     * set motor power to float
     */
    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    /**
     * @return if power float
     */
    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    /**
     * @return if motor is busy
     */
    public boolean isBusy() {
        return motor.isBusy();
    }

    /**
     * @return motor encoder value
     */
    public int getCurrentPosition() {
        return motor.getCurrentPosition() - encoderBase;
    }

    /**
     * reset the motor encoder value
     */
    public void resetPosition() {
        resetPosition(0);
    }

    /**
     * update the motor encoder base to reset the encoder
     */
    public void resetPosition(int position) {
        encoderBase = motor.getCurrentPosition() - position;
    }

    /**
     * set the motor runMode
     * @param runMode motor mode
     */
    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    /**
     * get the motor run mode
     * @return run mode of motor
     */
    public DcMotor.RunMode getMode() {
        return motor.getMode();
    }

    /**
     * set the motor target position
     * @param position target position
     */
    public void setTargetPosition(int position) {
        motor.setTargetPosition(encoderBase + position);
    }

    /**
     * get the current encoder target position
     * @return current encoder target position
     */
    public int getTargetPosition() {
        return motor.getTargetPosition() - encoderBase;
    }
}
