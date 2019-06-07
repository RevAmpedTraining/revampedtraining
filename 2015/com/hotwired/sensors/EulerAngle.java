package com.revAmped.sensors;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Instances of EulerAngles represent a direction in three-dimensional space by way of rotations.
 * Units are as specified in sensor initiation. Angles are in rotation order (heading, then roll,
 * then pitch) and are right-handed about their respective axes.
 */
public class EulerAngle {

    /** the rotation about the Z axis */
    public final double yaw;

    /** the rotation about the Y axis */
    public final double roll;

    /** the rotation about the X axix */
    public final double pitch;

    /** the time on the System.nanoTime() clock at which the data was acquired. If no
     * timestamp is associated with this particular set of data, this value is zero */
    public final long nanoTime;

    public EulerAngle(double x, double y, double z, long nanoTime) { // yaw, roll, pitch
        this.yaw = adjustAngle(x);
        this.roll = adjustAngle(y);
        this.pitch = adjustAngle(z);
        this.nanoTime = nanoTime;
    }

    public EulerAngle(EulerAngle ea) {
        this(ea.yaw, ea.roll, ea.pitch, ea.nanoTime);
    }

    public EulerAngle(I2cDeviceSynch.TimestampedData ts, int scale)
    {
        ByteBuffer buffer = ByteBuffer.wrap(ts.data).order(ByteOrder.LITTLE_ENDIAN);

        final double inverseScale = 1.0 / (double)scale;
        this.yaw     = adjustAngle(buffer.getShort() * inverseScale);
        this.roll    = adjustAngle(buffer.getShort() * inverseScale);
        this.pitch   = adjustAngle(buffer.getShort() * inverseScale);
        this.nanoTime = ts.nanoTime;
    }

    private double adjustAngle (double angle) {
        if (angle > 180) {
            return angle - 360;
        }
        else {
            return angle;
        }
    }

    public String toString() {
        return String.format("Y: %d, R: %d, P: %d", (int)yaw, (int)roll, (int)pitch);
    }
}
