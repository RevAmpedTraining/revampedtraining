package com.revAmped.sensors;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import static com.qualcomm.robotcore.hardware.I2cDeviceSynch.ReadMode.ONLY_ONCE;

/**
 * {@link HwI2cDevice} allows one to retrieve an underlying {@link I2cDeviceSynch} from
 * an a client of the latter. Such objects usually represent an semantic layer for an I2c sensor
 * or actuator.
 */
public abstract class HwI2cDevice {

    protected final OpMode opmodeContext;
    protected final I2cDeviceSynch deviceClient;
    protected String loggingTag = "HwI2C";
    protected volatile I2cAddr i2cAddr;

    protected HwI2cDevice(OpMode opmodeContext,
                          I2cDevice i2cDevice,
                          I2cAddr i2cAddr) {
        this(opmodeContext,
             new I2cDeviceSynchImpl(i2cDevice,
                                    i2cAddr,
                                    false),
             i2cAddr);
    }

    protected HwI2cDevice(OpMode opmodeContext,
                          I2cDeviceSynch deviceClient,
                          I2cAddr i2cAddr) {
        this.opmodeContext = opmodeContext;
        this.deviceClient = deviceClient;
        this.i2cAddr = i2cAddr;
    }

    /**
     * Initialize the sensor. Note that the execution of
     * this method can take a fairly long while, possibly several tens of milliseconds.
     */
    protected abstract void initialize ();

    public I2cAddr getI2cAddr () {
        return this.i2cAddr;
    }

    public void setI2cAddr (I2cAddr i2cAddr) {
        this.deviceClient.setI2cAddr(i2cAddr);
        this.i2cAddr = i2cAddr;
    }

    /**
     * make sure deviceClient has the correct I2C address and logging tag
     */
    public void prepareDevice () {
        this.deviceClient.setI2cAddr(this.i2cAddr);
        this.deviceClient.setLoggingTag(this.loggingTag);
    }

    /**
     * Returns the device client currently used by this object
     * @return the device client currently used by this object
     */
    public I2cDeviceSynch getI2cDeviceSynch() {
        return this.deviceClient;
    }


    /**
     * Shut down the sensor. This doesn't do anything in the hardware device itself, but rather
     * shuts down any resources (threads, etc) that we use to communicate with it.
     */
    public void close() {
        this.deviceClient.close();
    }

    /**
     * set logging tag
     */
    public void setLoggingTag(String loggingTag) {
        if (loggingTag != null && loggingTag.length() > 0) {
            this.loggingTag = loggingTag;
        }
    }

    /**
     * Set the register value until it is verified
     * @param register register to set
     * @param value value to set
     * @param timeoutMillis timeout in milliseconds
     * @throws InterruptedException if the thread is interrupted
     * @throws HwI2cException if action fails
     */
    protected void setRegister(byte register,
                               byte value,
                               long timeoutMillis)
        throws InterruptedException, HwI2cException {

        byte regVal;
        I2cDeviceSynch.ReadWindow currWindow =  this.deviceClient.getReadWindow();
        this.deviceClient.setReadWindow(new I2cDeviceSynch.ReadWindow(register,
                                                                      1,
                                                                      ONLY_ONCE));

        long startTime = System.currentTimeMillis();
        do {
            if (System.currentTimeMillis() - startTime < timeoutMillis)
            {
                String mesg = String.format("Timeout in setting register 0x%h: value %d: get %d", register, value);
                Log.e(loggingTag, mesg);
                throw new HwI2cException(mesg);
            }
            this.deviceClient.write8(register, value);
            Thread.sleep(50);
            regVal = this.deviceClient.read8(register);
        } while (regVal != value);

        this.deviceClient.setReadWindow(currWindow);
    }

    /**
     * Add the register value until it is verified
     * @param register register to set
     * @param value value to set
     * @param timeoutMillis timeout in milliseconds
     * @throws InterruptedException if the thread is interrupted
     * @throws HwI2cException if action fails
     */
    protected void addRegister(byte register,
                               byte value,
                               long timeoutMillis)
        throws InterruptedException, HwI2cException {

        byte lastVal;
        I2cDeviceSynch.ReadWindow currWindow =  this.deviceClient.getReadWindow();
        this.deviceClient.setReadWindow(new I2cDeviceSynch.ReadWindow(register,
                                                                      1,
                                                                      ONLY_ONCE));

        long startTime = System.currentTimeMillis();
        byte bVal = this.deviceClient.read8(register);
        do {
            if (System.currentTimeMillis() - startTime < timeoutMillis)
            {
                String mesg = String.format("Timeout in setting register 0x%h: value %d: get %d", register, value);
                Log.e(loggingTag, mesg);
                throw new HwI2cException(mesg);
            }
            bVal = (byte) (bVal | value);
            this.deviceClient.write8(register, bVal);
            lastVal = bVal;
            Thread.sleep(50);
            bVal = this.deviceClient.read8(register);
        } while (lastVal != bVal);

        this.deviceClient.setReadWindow(currWindow);
    }

    /**
     * Verify register value
     * @param register register to verify
     * @param value correct register value
     * @param timeoutMillis timeout in milliseconds
     * @throws InterruptedException if the thread is interrupted
     * @throws HwI2cException if action fails
     */
    protected void verifyRegister(byte register,
                                  byte value,
                                  long timeoutMillis)
        throws InterruptedException, HwI2cException {

        I2cDeviceSynch.ReadWindow currWindow =  this.deviceClient.getReadWindow();
        this.deviceClient.setReadWindow(new I2cDeviceSynch.ReadWindow(register,
                                                                      1,
                                                                      ONLY_ONCE));

        long startTime = System.currentTimeMillis();
        byte regVal = this.deviceClient.read8(register);
        while (regVal != value) {
            if (System.currentTimeMillis() - startTime > timeoutMillis) {
                String mesg = String.format("Timeout in verifying register 0x%h: expect %d: get %d", register, value, regVal);
                Log.e(loggingTag, mesg);
                throw new HwI2cException(mesg);
            }
            Thread.sleep(50);
            regVal = this.deviceClient.read8(register);
        }

        this.deviceClient.setReadWindow(currWindow);
    }
}
