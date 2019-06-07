package com.revAmped.sensors;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import static com.qualcomm.robotcore.hardware.I2cDeviceSynch.ReadMode.ONLY_ONCE;

/**
 * I2CXL-MaxSonar-EZ series
 *
 * @see <a href="http://www.maxbotix.com/articles/095.htm">http://www.maxbotix.com/articles/095.htm</a>
 */
public class MaxSonarI2CXLRanger
    implements MaxSonarI2CXL
{
    private I2cDeviceSynch deviceClient;

    private String loggingTag = "MaxSonar";

    private volatile long pingTime = FIRST_TIME;

    private volatile int distance = -1;

    private final static long FIRST_TIME = 0;

    private final static I2cDeviceSynch.ReadWindow WINDOW = new I2cDeviceSynch.ReadWindow(0x0,
                                                                                          bDATA_LENGTH,
                                                                                          ONLY_ONCE);

    public MaxSonarI2CXLRanger(I2cDevice i2cDevice) {
        this(i2cDevice,
             new I2cAddr(bDEFAULT_ADDRESS));
    }

    public MaxSonarI2CXLRanger(I2cDevice i2cDevice,
                               I2cAddr i2cAddr) {
        this.deviceClient = new I2cDeviceSynchImpl(i2cDevice,
                                                   i2cAddr,
                                                   false);

        initialize();
    }

    private void initialize() {
        // We don't have the device auto-close since *we* handle the shutdown logic
        this.deviceClient.setReadWindow(WINDOW);
        this.deviceClient.engage();
    }

    /**
     * Shut down the sensor. This doesn't do anything in the hardware device itself, but rather
     * shuts down any resources (threads, etc) that we use to communicate with it.
     */
    public void close() {
        this.deviceClient.close();
    }

    public I2cAddr getI2cAddr () {
        return this.deviceClient.getI2cAddress();
    }

    public void setI2cAddr (I2cAddr i2cAddr) {
        this.deviceClient.setI2cAddress(i2cAddr);
    }

    /**
     * allow 100ms between pings.
     * see I2C-MaxSonar-EZ Commands
     * It is best to allow 100ms between readings to allow for proper acoustic dissipation.
     */
    @Override
    public void startRanging()
    {
        if (System.currentTimeMillis() - pingTime >= RANGING_WAIT_MS ||
            pingTime == FIRST_TIME) {
            this.deviceClient.write8(bSTART_RANGING, 0x0);
            pingTime = System.currentTimeMillis();
        }
    }

    @Override
    public int getDistance()
    {
        if (pingTime != FIRST_TIME &&
            System.currentTimeMillis() - pingTime < READING_WAIT_MS) {
            return distance;
        }

        // Ensure we can see the registers we need
        this.deviceClient.setReadWindow(WINDOW);
        byte[] bytes = deviceClient.read(0x0,
                                         bDATA_LENGTH);
        ByteBuffer buffer = ByteBuffer.wrap(bytes).order(ByteOrder.BIG_ENDIAN);
        distance = buffer.getShort();
        return distance;
    }

    @Override
    public void changeAddress(I2cAddr newI2cAddr) {
        int newAddr = newI2cAddr.get8Bit();
        // see I2C-MaxSonar-EZ Commands
        if (newAddr == 0 ||
            newAddr == 80 ||
            newAddr == 164 ||
            newAddr == 170 ||
            newAddr == this.getI2cAddr().get8Bit()) {
            return;
        }

        this.deviceClient.write(170,
                                new byte[]{(byte)165, (byte)newAddr});

        this.setI2cAddr(newI2cAddr);
    }
}
