package com.revAmped.test;

import com.revAmped.opmode.Task;
import com.qualcomm.robotcore.hardware.DcMotor;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * Delegater class for DcMotor
 */
public class MotorHelper implements HardwareDevice {
    private DcMotor m_motor;
    private Task m_task;

    public MotorHelper(DcMotor motor, Task task) {
        m_motor = motor;
        m_task = task;
    }

    public boolean isResetEncoder() {
        return m_motor.getMode() == RunMode.STOP_AND_RESET_ENCODER;
    }

    public void resetEncoder() throws InterruptedException {
        boolean is0 = false;
        boolean isReset = false;

        m_motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        do {
            m_task.idle();

            // they may not be all true at the same time
            if (!is0) {
                is0 = Math.abs(getCurrentPosition()) < 10;
            }
            if (!isReset && m_motor.getMode() == RunMode.STOP_AND_RESET_ENCODER) {
                isReset = true;
            }
        }
        while ((!isReset || !is0) &&
            m_task.threadIsNotInterrupted());
    }

    public void setMotorRunMode(RunMode mode) throws InterruptedException {
        m_motor.setMode(mode);
        do {
            m_task.idle();
        } while (m_motor.getMode() != mode &&
            m_task.threadIsNotInterrupted());
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    public String getDeviceName() {
        return m_motor.getDeviceName();
    }

    public String getConnectionInfo() {
        return m_motor.getConnectionInfo();
    }

    public int getVersion() {
        return m_motor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        m_motor.resetDeviceConfigurationForOpMode();
    }

    public void close() {
        m_motor.close();
    }

    public DcMotorController getController() {
        return m_motor.getController();
    }

    public void setDirection(DcMotor.Direction direction) {
        m_motor.setDirection(direction);
    }

    public DcMotor.Direction getDirection() {
        return m_motor.getDirection();
    }

    public int getPortNumber() {
        return m_motor.getPortNumber();
    }

    public void setPower(double power) {
        m_motor.setPower(power);
    }

    public double getPower() {
        return m_motor.getPower();
    }

    public boolean isBusy() {
        return m_motor.isBusy();
    }

    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        m_motor.setZeroPowerBehavior(behavior);
    }

    public ZeroPowerBehavior getZeroPowerBehavior() {
        return m_motor.getZeroPowerBehavior();
    }

    public void setTargetPosition(int position) {
        m_motor.setTargetPosition(position);
    }

    public int getTargetPosition() {
        return m_motor.getTargetPosition();
    }

    public int getCurrentPosition() {
        return m_motor.getCurrentPosition();
    }

    public void setMode(RunMode mode) {
        m_motor.setMode(mode);
    }

    public RunMode getMode() {
        return m_motor.getMode();
    }
}
