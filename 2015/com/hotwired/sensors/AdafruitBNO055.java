package com.revAmped.sensors;

import com.revAmped.components.HwGyro;

/**
 * Interface API to the Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 sensor.
 * You can create an implementation of this interface for a given sensor using
 * @see <a href="http://www.adafruit.com/products/2472">http://www.adafruit.com/products/2472</a>
 * @see <a href="http://www.bosch-sensortec.com/en/homepage/products_3/9_axis_sensors_5/ecompass_2/bno055_3/bno055_4">
 *     http://www.bosch-sensortec.com/en/homepage/products_3/9_axis_sensors_5/ecompass_2/bno055_3/bno055_4</a>
 */
public interface AdafruitBNO055 {

    //------------------------------------------------------------------------------------------
    // Constants
    //------------------------------------------------------------------------------------------

    /** The value for CHIP_ID. see Table 4-2 */
    byte bCHIP_ID_VALUE = (byte)0xa0;

    /** The value for default I2C_ADDR. see Table 4-7 */
    byte bI2C_ADDR_DEFAULT = (byte) 0x28*2;

    /** The value for normal POWER_MODE. see Section 3.2.1 */
    byte bPOWER_MODE_NORMAL = (byte) 0x0;

    /** The value for build in self test result ST_RESULT. see Section 4.3.55 ST_RESULT */
    byte bBI_SELF_TEST_PASSED = (byte)0x0F;

    /** The value for system status SYS_STATUS in FUSION. see Section 4.3.58 SYS_STATUS */
    byte bSYS_STATUS_FUSION = (byte)0x05;

    /** The size of the calibration data, starting at ACCEL_OFFSET_X_LSB, is 22 bytes. */
    int CALIBRATION_DATA_LENGTH = 22;

    /** The size of the Euler data, starting at EULER_H_LSB, is 6 bytes. */
    int EULER_DATA_LENGTH = 6;

    /** The size of the Quaternion data, starting at QUATERNION_DATA_W_LSB, is 8 bytes. */
    int QUATERNION_DATA_LENGTH = 8;

    /** The agular scale in degree. see Table 3-29 */
    int AGULAR_DEGREE_SCALE = 16;

    /** The quaternion scale in degree. see Table 3-31 */
    int QUATERNION_SCALE = (1<<14);

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    /** Returns the absolute orientation of the sensor as a set of Euler angles.
     * @see #getQuaternionOrientation()
     * @return  the absolute orientation of the sensor
     */
    EulerAngle          getAngularOrientation();

    /** Returns the absolute orientation of the sensor as a quaternion.
     *
     * @return  the absolute orientation of the sensor
     */
    Quaternion          getQuaternionOrientation();

    //----------------------------------------------------------------------------------------------
    // Status inquiry
    //----------------------------------------------------------------------------------------------

    /**
     * Returns the current status of the system.
     * @return the current status of the system
     *
     * See section 4.3.58 of the BNO055 specification.
     * @see #getSystemError()
     *
    <table summary="System Status Codes">
    <tr><td>Result</td><td>Meaning</td></tr>
    <tr><td>0</td><td>idle</td></tr>
    <tr><td>1</td><td>system error</td></tr>
    <tr><td>2</td><td>initializing peripherals</td></tr>
    <tr><td>3</td><td>system initialization</td></tr>
    <tr><td>4</td><td>executing self-test</td></tr>
    <tr><td>5</td><td>sensor fusion algorithm running</td></tr>
    <tr><td>6</td><td>system running without fusion algorithms</td></tr>
    </table>
     */
    byte getSystemStatus();

    /** If {@link #getSystemStatus()} is 'system error' (1), returns particulars
     * regarding that error.
     *
     * See section 4.3.58 of the BNO055 specification.
     * @return the current error status
     * @see #getSystemStatus()
     *
    <table summary="System Error Codes">
    <tr><td>Result</td><td>Meaning</td></tr>
    <tr><td>0</td><td>no error</td></tr>
    <tr><td>1</td><td>peripheral initialization error</td></tr>
    <tr><td>2</td><td>system initialization error</td></tr>
    <tr><td>3</td><td>self test result failed</td></tr>
    <tr><td>4</td><td>register map bVal out of range</td></tr>
    <tr><td>5</td><td>register map address out of range</td></tr>
    <tr><td>6</td><td>register map write error</td></tr>
    <tr><td>7</td><td>BNO low power mode not available for selected operation mode</td></tr>
    <tr><td>8</td><td>acceleromoeter power mode not available</td></tr>
    <tr><td>9</td><td>fusion algorithm configuration error</td></tr>
    <tr><td>A</td><td>sensor configuraton error</td></tr>
    </table> */
    byte getSystemError();

    /**
     * Get calibration status
     * @return calibration status
     */
    CalibrationStatus getCalibrationStatus ();

    /**
     * Read calibration data from the IMU which later can be restored with loadCalibrationData().
     * This might be persistently stored, and reapplied at a later power-on.
     * <p/>
     * For greatest utility, full calibration should be achieved before reading
     * the calibration data
     * <p/>
     * From Section 3.11.4 of the datasheet:
     * <p/>
     * "The calibration profile includes sensor offsets and sensor radius. Host system can
     * read the offsets and radius only after a full calibration is achieved and the operation
     * mode is switched to CONFIG_MODE. Refer to sensor offsets and sensor radius registers."
     *
     * @throws InterruptedException thread is interrupted
     */
    void dumpCalibrationData() throws InterruptedException;

    //----------------------------------------------------------------------------------------------
    // Low level reading and writing
    //----------------------------------------------------------------------------------------------

    /**
     * Low level: read the byte starting at the indicated register
     * @param register  the location from which to read the data
     * @return          the data that was read
     */
    byte  read8(REGISTER register);
    /**
     * Low level: read data starting at the indicated register
     * @param register  the location from which to read the data
     * @param cb        the number of bytes to read
     * @return          the data that was read
     */
    byte[] read(REGISTER register, int cb);

    /**
     * Low level: write a byte to the indicated register
     * @param register  the location at which to write the data
     * @param bVal      the data to write
     */
    void write8(REGISTER register, int bVal);
    /**
     * Low level: write data starting at the indicated register
     * @param register  the location at which to write the data
     * @param data      the data to write
     */
    void write (REGISTER register, byte[] data);

    /**
     * Verify register bVal
     *
     * @param register      register to verify
     * @param value         correct register bVal
     * @param timeoutMillis timeout in milliseconds
     * @throws InterruptedException if the thread is interrupted
     * @throws HwI2cException       if action fails
     */
    void verifyRegister(REGISTER register,
                        byte value,
                        long timeoutMillis)
        throws InterruptedException, HwI2cException;

    //----------------------------------------------------------------------------------------------
    // Enumerations to make all of the above work
    //----------------------------------------------------------------------------------------------

    // Set the output units. Section 3.6,1, Table 3-11, p31
    enum TEMP_UNIT        { CELSIUS(0), FARENHEIT(1);                        private byte bVal; TEMP_UNIT(int i)        { bVal =(byte)(i<<4); } public byte getValue() { return this.bVal; }}
    enum EULER_ANGLE_UNIT { DEGREES(0), RADIANS(1);                          private byte bVal; EULER_ANGLE_UNIT(int i) { bVal =(byte)(i<<2); } public byte getValue() { return this.bVal; }}
    enum GYRO_ANGLE_UNIT  { DEGREES(0), RADIANS(1);                          private byte bVal; GYRO_ANGLE_UNIT(int i)  { bVal =(byte)(i<<1); } public byte getValue() { return this.bVal; }}
    enum ACCEL_UNIT       { METERS_PERSEC_PERSEC(0), MILLIGALS(1);           private byte bVal; ACCEL_UNIT(int i)       { bVal =(byte)(i<<0); } public byte getValue() { return this.bVal; }}
    enum PITCH_MODE       { WINDOWS(0), ANDROID(1);                          private byte bVal; PITCH_MODE(int i)       { bVal =(byte)(i<<7); } public byte getValue() { return this.bVal; }}

    enum SYS_TRIGGER      { CLK_SEL(7), RST_INT(6), RST_SYS(5), SELFTEST(0); private byte bVal; SYS_TRIGGER(int i)      { bVal =(byte)(1<<i); } public byte getValue() { return this.bVal; }}

    // sensor configuration cannot be changed in FUSION mode. see section 3.5
    enum GYRO_RANGE       { DPS2000(0), DPS1000(1), DPS500(2), DPS250(3), DPS125(4);                               private byte bVal; GYRO_RANGE(int i)       { bVal =(byte)(i<<0);} public byte getValue() { return this.bVal; }}
    enum GYRO_BANDWIDTH   { HZ523(0), HZ230(1), HZ116(2), HZ47(3), HZ23(4), HZ12(5), HZ64(6), HZ32(7);             private byte bVal; GYRO_BANDWIDTH(int i)   { bVal =(byte)(i<<3);} public byte getValue() { return this.bVal; }}
    enum GYRO_POWER_MODE  { NORMAL(0), FAST(1), DEEP(2), SUSPEND(3), ADVANCED(4) ;                                 private byte bVal; GYRO_POWER_MODE(int i)  { bVal =(byte)(i<<0);} public byte getValue() { return this.bVal; }}
    enum ACCEL_RANGE      { G2(0), G4(1), G8(2), G16(3);                                                           private byte bVal; ACCEL_RANGE(int i)      { bVal =(byte)(i<<0);} public byte getValue() { return this.bVal; }}
    enum ACCEL_BANDWIDTH  { HZ7_81(0), HZ15_63(1), HZ31_25(2), HZ62_5(3), HZ125(4), HZ250(5), HZ500(6), HZ1000(7); private byte bVal; ACCEL_BANDWIDTH(int i)  { bVal =(byte)(i<<2);} public byte getValue() { return this.bVal; }}
    enum ACCEL_POWER_MODE { NORMAL(0), SUSPEND(1), LOW1(2), STANDBY(3), LOW2(4), DEEP(5);                          private byte bVal; ACCEL_POWER_MODE(int i) { bVal =(byte)(i<<5);} public byte getValue() { return this.bVal; }}

    enum MAG_RATE         { HZ2(0), HZ6(1), HZ8(2), HZ10(3), HZ15(4), HZ20(5), HZ25(6), HZ30(7);                   private byte bVal; MAG_RATE(int i)         { bVal =(byte)(i<<0);} public byte getValue() { return this.bVal; }}
    enum MAG_OPMODE       { LOW(0), REGULAR(1), ENHANCED(2), HIGH(3);                                              private byte bVal; MAG_OPMODE(int i)       { bVal =(byte)(i<<3);} public byte getValue() { return this.bVal; }}
    enum MAG_POWER_MODE   { NORMAL(0), SLEEP(1), SUSPEND(2), FORCE(3);                                             private byte bVal; MAG_POWER_MODE(int i)   { bVal =(byte)(i<<5);} public byte getValue() { return this.bVal; }}

    /**
     * Sensor modes are described in Table 3-5 (p21) of the BNO055 specification,
     * where they are termed "operation modes".
     */
    enum SENSOR_MODE
    {
        CONFIG(0X00),       ACCONLY(0X01),          MAGONLY(0X02),
        GYRONLY(0X03),      ACCMAG(0X04),           ACCGYRO(0X05),
        MAGGYRO(0X06),      AMG(0X07),              IMU(0X08),
        COMPASS(0X09),      M4G(0X0A),              NDOF_FMC_OFF(0X0B),
        NDOF(0X0C);
        //------------------------------------------------------------------------------------------
        private byte bVal;
        SENSOR_MODE(int i) { this.bVal = (byte) i; }
        public byte getValue() { return this.bVal; }

        /** Is this SENSOR_MODE one of the fusion modes in which the BNO055 operates? */
        public boolean isFusionMode()
        {
            // See Table 3-5, p21, of the BNO055 specification
            switch (this)
            {
                case IMU:
                case COMPASS:
                case M4G:
                case NDOF_FMC_OFF:
                case NDOF:
                    return true;
                default:
                    return false;
            }
        }
    }

    /**
     * REGISTER provides symbolic names for each of the BNO055 device registers
     */
    enum REGISTER
    {
        /** Controls which of the two register pages are visible */
        PAGE_ID(0X07),

        /** Page 0 */
        CHIP_ID(0x00),
        ACCEL_REV_ID(0x01),
        MAG_REV_ID(0x02),
        GYRO_REV_ID(0x03),
        SW_REV_ID_LSB(0x04),
        SW_REV_ID_MSB(0x05),
        BL_REV_ID(0X06),

        /** Acceleration data register */
        ACCEL_DATA_X_LSB(0X08),
        ACCEL_DATA_X_MSB(0X09),
        ACCEL_DATA_Y_LSB(0X0A),
        ACCEL_DATA_Y_MSB(0X0B),
        ACCEL_DATA_Z_LSB(0X0C),
        ACCEL_DATA_Z_MSB(0X0D),

        /** Magnetometer data register */
        MAG_DATA_X_LSB(0X0E),
        MAG_DATA_X_MSB(0X0F),
        MAG_DATA_Y_LSB(0X10),
        MAG_DATA_Y_MSB(0X11),
        MAG_DATA_Z_LSB(0X12),
        MAG_DATA_Z_MSB(0X13),

        /** Gyro data registers */
        GYRO_DATA_X_LSB(0X14),
        GYRO_DATA_X_MSB(0X15),
        GYRO_DATA_Y_LSB(0X16),
        GYRO_DATA_Y_MSB(0X17),
        GYRO_DATA_Z_LSB(0X18),
        GYRO_DATA_Z_MSB(0X19),

        /** Euler data registers */
        EULER_H_LSB(0X1A),
        EULER_H_MSB(0X1B),
        EULER_R_LSB(0X1C),
        EULER_R_MSB(0X1D),
        EULER_P_LSB(0X1E),
        EULER_P_MSB(0X1F),

        /** Quaternion data registers */
        QUATERNION_DATA_W_LSB(0X20),
        QUATERNION_DATA_W_MSB(0X21),
        QUATERNION_DATA_X_LSB(0X22),
        QUATERNION_DATA_X_MSB(0X23),
        QUATERNION_DATA_Y_LSB(0X24),
        QUATERNION_DATA_Y_MSB(0X25),
        QUATERNION_DATA_Z_LSB(0X26),
        QUATERNION_DATA_Z_MSB(0X27),

        /** Linear acceleration data registers */
        LINEAR_ACCEL_DATA_X_LSB(0X28),
        LINEAR_ACCEL_DATA_X_MSB(0X29),
        LINEAR_ACCEL_DATA_Y_LSB(0X2A),
        LINEAR_ACCEL_DATA_Y_MSB(0X2B),
        LINEAR_ACCEL_DATA_Z_LSB(0X2C),
        LINEAR_ACCEL_DATA_Z_MSB(0X2D),

        /** Gravity data registers */
        GRAVITY_DATA_X_LSB(0X2E),
        GRAVITY_DATA_X_MSB(0X2F),
        GRAVITY_DATA_Y_LSB(0X30),
        GRAVITY_DATA_Y_MSB(0X31),
        GRAVITY_DATA_Z_LSB(0X32),
        GRAVITY_DATA_Z_MSB(0X33),

        /** Temperature data register */
        TEMP(0X34),

        /** Status registers */
        CALIB_STAT(0X35),
        SELFTEST_RESULT(0X36),
        INTR_STAT(0X37),

        SYS_CLK_STAT(0X38),
        SYS_STAT(0X39),
        SYS_ERR(0X3A),

        /** Unit selection register */
        UNIT_SEL(0X3B),
        DATA_SELECT(0X3C),

        /** Mode registers */
        OPR_MODE(0X3D),
        PWR_MODE(0X3E),

        SYS_TRIGGER(0X3F),
        TEMP_SOURCE(0X40),

        /** Axis remap registers */
        AXIS_MAP_CONFIG(0X41),
        AXIS_MAP_SIGN(0X42),

        /**Accelerometer Offset registers */
        ACCEL_OFFSET_X_LSB(0X55),
        ACCEL_OFFSET_X_MSB(0X56),
        ACCEL_OFFSET_Y_LSB(0X57),
        ACCEL_OFFSET_Y_MSB(0X58),
        ACCEL_OFFSET_Z_LSB(0X59),
        ACCEL_OFFSET_Z_MSB(0X5A),

        /** Magnetometer Offset registers */
        MAG_OFFSET_X_LSB(0X5B),
        MAG_OFFSET_X_MSB(0X5C),
        MAG_OFFSET_Y_LSB(0X5D),
        MAG_OFFSET_Y_MSB(0X5E),
        MAG_OFFSET_Z_LSB(0X5F),
        MAG_OFFSET_Z_MSB(0X60),

        /** Gyroscope Offset register s*/
        GYRO_OFFSET_X_LSB(0X61),
        GYRO_OFFSET_X_MSB(0X62),
        GYRO_OFFSET_Y_LSB(0X63),
        GYRO_OFFSET_Y_MSB(0X64),
        GYRO_OFFSET_Z_LSB(0X65),
        GYRO_OFFSET_Z_MSB(0X66),

        /** Radius registers */
        ACCEL_RADIUS_LSB(0X67),
        ACCEL_RADIUS_MSB(0X68),
        MAG_RADIUS_LSB(0X69),
        MAG_RADIUS_MSB(0X6A),

        /** Selected Page 1 registers */
        ACC_CONFIG(0x08),
        MAG_CONFIG(0x09),
        GYR_CONFIG_0(0x0A),
        GYR_CONFIG_1(0x0B),
        ACC_SLEEP_CONFIG(0x0C),
        GYR_SLEEP_CONFIG(0x0D);

        //------------------------------------------------------------------------------------------
        private byte bVal;
        private REGISTER(int i)
        {
            this.bVal = (byte)i;
        }
        public byte getValue() { return this.bVal; }
    }

    /**
     * calibration status. see see Section 4.3.54 CALIB_STAT
     */
    interface CalibrationStatus {

        /** The value for fully calibrated status CALIB_STAT. see Section 4.3.54 CALIB_STAT */
        byte bCALIB_STAT_FULLY = (byte)0x03;

        /** The 2 bits mask for each field of the calibrated status CALIB_STAT. see Section 4.3.54 CALIB_STAT */
        byte bCALIB_STAT_MASK = (byte)0x03;

        //----------------------------------------------------------------------------------------------
        // Calibration section
        //----------------------------------------------------------------------------------------------

        byte getSystemCalibrationStaus();
        byte getGyroCalibrationStaus();
        byte getAccelerometerCalibrationStaus();
        byte getMagnetometerCalibrationStaus();

        boolean isSystemCalibrated();
        boolean isGyroCalibrated();
        boolean isAccelerometerCalibrated();
        boolean isMagnetometerCalibrated();
        boolean isAllCalibrated();
    }
}
