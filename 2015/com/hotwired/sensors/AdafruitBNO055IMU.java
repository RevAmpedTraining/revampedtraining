package com.revAmped.sensors;

import android.util.Log;

import com.revAmped.components.HwGyro;
import com.revAmped.util.ByteDumper;
import com.revAmped.util.ByteLoader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import java.io.IOException;

import static com.qualcomm.robotcore.hardware.I2cDeviceSynch.ReadMode.ONLY_ONCE;
import static com.qualcomm.robotcore.hardware.I2cDeviceSynch.ReadMode.REPEAT;

/**
 * Implementation of the Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 sensor.
 * For details please refer to the following data sheet.
 *
 * @see <a href="http://www.adafruit.com/products/2472">http://www.adafruit.com/products/2472</a>
 * @see <a href="http://www.bosch-sensortec.com/en/homepage/products_3/9_axis_sensors_5/ecompass_2/bno055_3/bno055_4">
 * http://www.bosch-sensortec.com/en/homepage/products_3/9_axis_sensors_5/ecompass_2/bno055_3/bno055_4</a>
 */
public class AdafruitBNO055IMU
    extends HwI2cDevice
    implements AdafruitBNO055, HwGyro {

    //------------------------------------------------------------------------------------------
    // State
    //------------------------------------------------------------------------------------------

    private SENSOR_MODE currentMode = null;

    private volatile double baseHeading = 0.0;

    private volatile boolean isCalibrating = false;

    // The msAwaitSelfTest value is lore. We choose here to use the same value for awaiting chip id,
    // on the (not completely unreasonable) theory that similar things are happening in the chip in both
    // cases. A survey of other libraries is as follows:
    //  1000ms:     https://github.com/OpenROV/openrov-software-arduino/blob/master/OpenROV/BNO055.cpp
    //              https://github.com/alexstyl/Adafruit-BNO055-SparkCore-port/blob/master/Adafruit_BNO055.cpp
    private static final int msWAIT_LONG = 2000;

    /**
     * One of two primary register windows we use for reading from the BNO055.
     * <p/>
     * Given the maximum allowable size of a register window, the set of registers on
     * a BNO055 can be usefully divided into two windows, which we here call LOWER_WINDOW
     * and UPPER_WINDOW.
     * <p/>
     * When we find the need to change register windows depending on what data is being requested
     * from the sensor, we try to use these two windows so as to reduce the number of register
     * window switching that might be required as other data is read in the future.
     */
    private static final I2cDeviceSynch.ReadWindow LOWER_WINDOW = newWindow(REGISTER.CHIP_ID, REGISTER.EULER_H_LSB);

    /**
     * A second of two primary register windows we use for reading from the BNO055.
     * We'd like to include the temperature register, too, but that would make a 27-byte window, and
     * those don't (currently) work in the CDIM.
     *
     * @see #LOWER_WINDOW
     */
    private static final I2cDeviceSynch.ReadWindow UPPER_WINDOW = newWindow(REGISTER.EULER_H_LSB, REGISTER.TEMP);

    /**
     * File for storing official calibration data. Temporary calibration data file has to be
     * promoted to official calibration data to be loaded.
     */
    private static final String CALIBRATION_FILE = "BNO055Calib";

    /**
     * File for storing temporary calibration data. Temporary calibration data file has to be
     * promoted to official calibration data to be loaded.
     */
    private static final String CALIBRATION_FILE_TEMP = "BNO055Calibtmp";

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    /**
     * Instantiate an AdaFruitBNO055IMU on the indicated device whose I2C address is the one indicated.
     */
    public AdafruitBNO055IMU(OpMode opmodeContext,
                             I2cDevice i2cDevice) {
        this(opmodeContext,
             i2cDevice,
             new I2cAddr(bI2C_ADDR_DEFAULT));
    }

    /**
     * Instantiate an AdaFruitBNO055IMU on the indicated device whose I2C address is the one indicated.
     */
    public AdafruitBNO055IMU(OpMode opmodeContext,
                             I2cDevice i2cDevice,
                             I2cAddr i2cAddr) {
        super(opmodeContext,
              i2cDevice,
              i2cAddr);

        initialize();
    }

    /**
     * We retry the initialization a few times: it's been reported to fail, intermittently,
     * but, so far as we can tell, entirely non-deterministically. Ideally, we'd like that to
     * never happen, but in light of our (current) inability to figure out how to prevent that,
     * we simply retry the initialization if it seems to fail.
     */
    @Override
    protected void initialize() {

        // We don't have the device auto-close since *we* handle the shutdown logic
        this.deviceClient.setReadWindow(LOWER_WINDOW);
        this.deviceClient.engage();

        setLoggingTag("BNO055");

        // Propagate relevant parameters to our device client
        this.deviceClient.setLogging(true);
        this.deviceClient.setLoggingTag(this.loggingTag);

        int status = 0;

        for (int attempt = 0; attempt < 4; attempt++) {
            try {
                initializeOnce();

                // At this point, the chip should in fact report correctly that it's in the mode requested.
                // See Section '4.3.58 SYS_STATUS' of the BNO055 specification
                status = getSystemStatus();
                if (status == bSYS_STATUS_FUSION) {
                    return;
                }

                Log.e(this.loggingTag,
                      String.format("retrying IMU initialization: unexpected system status %d; expected %d", status, bSYS_STATUS_FUSION));

                Thread.sleep(200);
            } catch (InterruptedException e) {
                return;
            }
        }

        throw new HwI2cException("unexpected system status %d; expected %d", status, bSYS_STATUS_FUSION);
    }

    /**
     * Do one attempt at initializing the device to be running in the indicated operation mode
     */
    private void initializeOnce()
        throws InterruptedException {
        // Lore: "send a throw-away command [...] just to make sure the BNO is in a good state
        // and ready to accept commands (this seems to be necessary after a hard power down)."
        write8(REGISTER.PAGE_ID, 0);

        // Make sure we have the right device
        verifyRegister(REGISTER.CHIP_ID,
                       bCHIP_ID_VALUE,
                       msWAIT_LONG);

        // Get us into config mode, for sure
        write8(REGISTER.OPR_MODE,
               SENSOR_MODE.CONFIG.getValue());

        // Reset the system, and wait for the chip id register to switch back from its reset state
        // to the it's chip id state. This can take a very long time, some 650ms (Table 0-2, p13)
        // perhaps. While in the reset state the chip id (and other registers) reads as 0xFF.
        write8(REGISTER.SYS_TRIGGER,
               SYS_TRIGGER.RST_SYS.getValue());

        // Make sure we have the right device
        verifyRegister(REGISTER.CHIP_ID,
                       bCHIP_ID_VALUE,
                       msWAIT_LONG);
        //Thread.sleep(50);

        // Set to normal power mode
        write8(REGISTER.PWR_MODE,
               bPOWER_MODE_NORMAL);
        Thread.sleep(50);

        // Make sure we're looking at register page zero, as the other registers
        // we need to set here are on that page.
        write8(REGISTER.PAGE_ID,
               (byte) 0);

        // Set the output units. Section 3.6.1, Table 3-11, p31
        int unitsel = PITCH_MODE.WINDOWS.getValue() |      // pitch angle convention
            TEMP_UNIT.CELSIUS.getValue() |                 // temperature
            EULER_ANGLE_UNIT.DEGREES.getValue() |          // euler angle units
            GYRO_ANGLE_UNIT.DEGREES.getValue() |           // gyro units, per second
            ACCEL_UNIT.METERS_PERSEC_PERSEC.getValue();    // accelerometer units
        write8(REGISTER.UNIT_SEL,
               (byte) unitsel);

        // Use the external crystal
        // See Section 5.5 (p100) of the BNO055 specification.
        write8(REGISTER.SYS_TRIGGER,
               SYS_TRIGGER.CLK_SEL.getValue());
        Thread.sleep(50);

        // sensor configuration cannot be changed in FUSION mode. see section 3.5
        // Switch to page 1 so we can write some more registers
        write8(REGISTER.PAGE_ID,
               (byte) 1);

        // Configure selected page 1 registers
        write8(REGISTER.ACC_CONFIG,
               (byte) (ACCEL_POWER_MODE.NORMAL.getValue() | ACCEL_BANDWIDTH.HZ62_5.getValue() | ACCEL_RANGE.G4.getValue()));
        write8(REGISTER.MAG_CONFIG,
               (byte) (MAG_POWER_MODE.NORMAL.getValue() | MAG_OPMODE.HIGH.getValue() | MAG_RATE.HZ20.getValue()));
        write8(REGISTER.GYR_CONFIG_0,
               (byte) (GYRO_BANDWIDTH.HZ32.getValue() | GYRO_RANGE.DPS2000.getValue()));
        write8(REGISTER.GYR_CONFIG_1,
               GYRO_POWER_MODE.NORMAL.getValue());

        // Switch back
        write8(REGISTER.PAGE_ID,
               (byte) 0);

        // Run a self test. This appears to be a necessary step in order for the
        // sensor to be able to actually be used. That is, we've observed that absent this,
        // the sensors do not return correct data. We wish that were documented somewhere.
        write8(REGISTER.SYS_TRIGGER,
               SYS_TRIGGER.CLK_SEL.getValue() | SYS_TRIGGER.SELFTEST.getValue());           // SYS_TRIGGER=0x3F

        // Per Section 3.9.2 Built In Self Test, when we manually kick of a self test,
        // the accelerometer, gyro, and magnetometer are tested, but the microcontroller is not.
        // So: we only wait for successful results from those three.
        verifyRegister(REGISTER.SELFTEST_RESULT,
                       bBI_SELF_TEST_PASSED,
                       msWAIT_LONG);

        loadCalibrationData();

        // Finally, enter the requested operating mode (see section 3.3)
        write8(REGISTER.OPR_MODE,
               SENSOR_MODE.NDOF.getValue());
        currentMode = SENSOR_MODE.NDOF;
    }

    @Override
    public EulerAngle getAngularOrientation() {

        // Ensure that the 6 bytes for this vector are visible in the register window.
        // Ensure we can see the registers we need
        this.deviceClient.setReadWindow(UPPER_WINDOW);

        // Section 3.6.5.5 of BNO055 specification
        I2cDeviceSynch.TimestampedData ts = deviceClient.readTimeStamped(REGISTER.EULER_H_LSB.getValue(),
                                                                         EULER_DATA_LENGTH);
        return new EulerAngle(ts,
                              AGULAR_DEGREE_SCALE);
    }

    @Override
    public Quaternion getQuaternionOrientation() {
        // Ensure we can see the registers we need
        this.deviceClient.setReadWindow(UPPER_WINDOW);

        // Section 3.6.5.5 of BNO055 specification
        I2cDeviceSynch.TimestampedData ts = deviceClient.readTimeStamped(REGISTER.QUATERNION_DATA_W_LSB.getValue(),
                                                                         QUATERNION_DATA_LENGTH);
        return new Quaternion(ts,
                              QUATERNION_SCALE);
    }

    /**
     * get gyro turn heading
     * @return gyro heading
     */
    @Override
    public int getHeading()
    {
        EulerAngle angles = getAngularOrientation();
        int heading = (int)(angles.yaw - baseHeading + 360) % 360;
        if (heading > 180) {
            heading = heading - 360;
        }
        return heading;
    }

    /**
     * reset gyro headings
     */
    @Override
    public void resetHeading() {

        EulerAngle angles = getAngularOrientation();
        this.baseHeading = angles.yaw;
    }

    @Override
    public boolean isCalibrating() {
        return this.isCalibrating;
    }

    @Override
    public synchronized void calibrate()
        throws InterruptedException {
        this.isCalibrating = true;
        // take 10 readings to help internal calibration
        for (int i = 0; i < 20; i++) {
            EulerAngle angles = getAngularOrientation();
            Thread.sleep(1);
        }
        resetHeading();
        this.isCalibrating = false;
    }

    @Override
    public CalibrationStatus getCalibrationStatus() {
        I2cDeviceSynch.ReadWindow currWindow =  this.deviceClient.getReadWindow();
        // Ensure we can see the registers we need
        this.deviceClient.setReadWindow(new I2cDeviceSynch.ReadWindow(REGISTER.CALIB_STAT.getValue(),
                                                                      1,
                                                                      ONLY_ONCE));

        byte b = this.read8(REGISTER.CALIB_STAT);

        this.deviceClient.setReadWindow(currWindow);

        return new BNO055CalibrationStatus(b);
    }

    @Override
    public byte getSystemStatus() {
        return read8(REGISTER.SYS_STAT);
    }

    @Override
    public byte getSystemError() {
        return read8(REGISTER.SYS_ERR);
    }

    @Override
    public synchronized void dumpCalibrationData()
        throws InterruptedException {
        InterruptedException ie = null;
        ByteDumper dumper;

        try {
            dumper = new ByteDumper(CALIBRATION_FILE_TEMP);
        } catch (IOException e) {
            Log.e(this.loggingTag,
                  "Failed to create calibration file " + CALIBRATION_FILE_TEMP);
            return;
        }

        // From Section 3.11.4 of the datasheet:
        //
        // "The calibration profile includes sensor offsets and sensor radius. Host system can
        // read the offsets and radius only after a full calibration is achieved and the operation
        // mode is switched to CONFIG_MODE. Refer to sensor offsets and sensor radius registers."

        SENSOR_MODE prevMode = this.currentMode;
        if (prevMode != SENSOR_MODE.CONFIG) {
            try {
                setSensorMode(SENSOR_MODE.CONFIG);
            } catch (InterruptedException e) {
                ie = e;
            }
        }

        // Read the calibration data
        byte[] data = this.read(REGISTER.ACCEL_OFFSET_X_LSB, CALIBRATION_DATA_LENGTH);

        try {
            dumper.add(data);
            Log.i(this.loggingTag,
                  "Dumped calibration to " + CALIBRATION_FILE);
        } catch (IOException e) {
            Log.e(this.loggingTag,
                  "Failed to dump calibration to " + CALIBRATION_FILE);
        } finally {
            if (dumper != null) {
                dumper.close();
            }
            // Restore the previous mode and return
            if (prevMode != SENSOR_MODE.CONFIG) {
                try {
                    setSensorMode(prevMode);
                } catch (InterruptedException e) {
                    if (ie == null) {
                        ie = e;
                    }
                }
            }
        }

        if (ie != null) {
            throw ie;
        }
    }

    /**
     * Write calibration data previously retrieved.  Only called in initialization.
     * <p/>
     * Section 3.11.4:
     * <p/>
     * It is important that the correct offsets and corresponding sensor radius are used.
     * Incorrect offsets may result in unreliable orientation data even at calibration
     * accuracy level 3. To set the calibration profile the following steps need to be taken
     * <p/>
     * 1. Select the operation mode to CONFIG_MODE
     * 2. Write the corresponding sensor offsets and radius data
     * 3. Change operation mode to fusion mode
     *
     * @throws InterruptedException thread is interrupted
     * @see #dumpCalibrationData()
     */
    private void loadCalibrationData()
        throws InterruptedException {
        ByteLoader loader;

        try {
            loader = new ByteLoader(CALIBRATION_FILE);
        } catch (IOException e) {
            Log.e(this.loggingTag,
                  "Failed to open calibration file " + CALIBRATION_FILE);
            return;
        }

        byte[] data = new byte[CALIBRATION_DATA_LENGTH];
        try {
            loader.load(data);
            Log.i(this.loggingTag,
                  "Loaded calibration from " + CALIBRATION_FILE);
        } catch (IOException e) {
            Log.e(this.loggingTag,
                  "Failed to load calibration from " + CALIBRATION_FILE);
        } finally {
            if (loader != null) {
                loader.close();
            }
        }

        // Write the calibration data
        this.write(REGISTER.ACCEL_OFFSET_X_LSB, data);
    }

    @Override
    public byte read8(REGISTER register) {
        // ensureReadWindow(new I2cDeviceSynch.ReadWindow(reg.getValue(), 1, REPEAT)); // no longer needed: a READ_ONCE window will automatically be used, if needed
        return deviceClient.read8(register.getValue());
    }

    @Override
    public byte[] read(REGISTER register, int cb) {
        // ensureReadWindow(new I2cDeviceSynch.ReadWindow(reg.getValue(), cb, REPEAT)); // no longer needed: a READ_ONCE window will automatically be used, if needed
        return deviceClient.read(register.getValue(), cb);
    }

    @Override
    public void write8(REGISTER register, int bVal) {
        this.deviceClient.write8(register.getValue(), bVal);
    }

    @Override
    public void write(REGISTER register, byte[] data) {
        this.deviceClient.write(register.getValue(), data);
    }

    /**
     * Verify register value
     *
     * @param register      register to verify
     * @param value         correct register value
     * @param timeoutMillis timeout in milliseconds
     * @throws InterruptedException if the thread is interrupted
     * @throws HwI2cException       if action fails
     */
    @Override
    public void verifyRegister(REGISTER register,
                               byte value,
                               long timeoutMillis)
        throws InterruptedException, HwI2cException {
        verifyRegister(register.getValue(),
                       value,
                       timeoutMillis);
    }

    private static I2cDeviceSynch.ReadWindow newWindow(REGISTER regFirst, REGISTER regMax) {
        return new I2cDeviceSynch.ReadWindow(regFirst.getValue(), regMax.getValue() - regFirst.getValue(), REPEAT);
    }

    private void ensureReadWindow(I2cDeviceSynch.ReadWindow needed)
    // We optimize small windows into larger ones if we can
    {
        I2cDeviceSynch.ReadWindow windowToSet = LOWER_WINDOW.containsWithSameMode(needed)
            ? LOWER_WINDOW
            : UPPER_WINDOW.containsWithSameMode(needed)
            ? UPPER_WINDOW
            : needed;           // just use what's needed if it's not within our two main windows
        this.deviceClient.ensureReadWindow(needed, windowToSet);
    }

    /**
     * The default operation mode after power-on is CONFIGMODE. When the user changes to another
     * operation mode, the sensors which are required in that particular sensor mode are powered,
     * while the sensors whose signals are not required are set to suspend mode.
     */
    private void setSensorMode(SENSOR_MODE mode)
        throws InterruptedException {
        // Remember the mode, 'cause that's easy
        this.currentMode = mode;

        // Actually change the operation/sensor mode
        this.write8(REGISTER.OPR_MODE,
                    mode.getValue() & 0x0F);                           // OPR_MODE=0x3D

        // Delay per Table 3-6 of BNO055 Data sheet (p21)
        Thread.sleep(30);
    }

    /**
     * calibration status. see see Section 4.3.54 CALIB_STAT
     */
    static class BNO055CalibrationStatus
        implements CalibrationStatus {

        private byte status;

        private BNO055CalibrationStatus(byte status) {
            this.status = status;
        }

        @Override
        public byte getSystemCalibrationStaus() {
            return (byte) ((status>>6) & bCALIB_STAT_MASK);
        }

        @Override
        public byte getGyroCalibrationStaus() {
            return (byte) ((status>>4) & bCALIB_STAT_MASK);
        }

        @Override
        public byte getAccelerometerCalibrationStaus() {
            return (byte) ((status>>2) & bCALIB_STAT_MASK);
        }

        @Override
        public byte getMagnetometerCalibrationStaus() {
            return (byte) ((status/*>>0*/) & bCALIB_STAT_MASK);
        }

        @Override
        public boolean isSystemCalibrated() {
            return getSystemCalibrationStaus() == bCALIB_STAT_FULLY;
        }

        @Override
        public boolean isGyroCalibrated() {
            return getGyroCalibrationStaus() == bCALIB_STAT_FULLY;
        }

        @Override
        public boolean isAccelerometerCalibrated() {
            return getAccelerometerCalibrationStaus() == bCALIB_STAT_FULLY;
        }

        @Override
        public boolean isMagnetometerCalibrated() {
            return getMagnetometerCalibrationStaus() == bCALIB_STAT_FULLY;
        }

        @Override
        public boolean isAllCalibrated() {
            return isSystemCalibrated() &&
                isGyroCalibrated() &&
                isAccelerometerCalibrated() &&
                isMagnetometerCalibrated();
        }

        @Override
        public String toString () {
            return String.format("S: %d, G: %d, A: %d, M: %d",
                                 getSystemCalibrationStaus(),
                                 getGyroCalibrationStaus(),
                                 getAccelerometerCalibrationStaus(),
                                 getMagnetometerCalibrationStaus());
        }
    }
}
