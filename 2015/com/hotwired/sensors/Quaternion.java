package com.revAmped.sensors;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * A Quaternion can indicate an orientation in three-space without the trouble of
 * possible gimbal-lock.
 *
 * @see <a href="https://en.wikipedia.org/wiki/Quaternion">https://en.wikipedia.org/wiki/Quaternion</a>
 * @see <a href="https://en.wikipedia.org/wiki/Gimballock">https://en.wikipedia.org/wiki/Gimbal_lock</a>
 * @see <a href="https://www.youtube.com/watch?v=zc8b2Jo7mno">https://www.youtube.com/watch?v=zc8b2Jo7mno</a>
 * @see <a href="https://www.youtube.com/watch?v=mHVwd8gYLnI">https://www.youtube.com/watch?v=mHVwd8gYLnI</a>
 */
public class Quaternion {

    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    public final double w;
    public final double x;
    public final double y;
    public final double z;

    /** the time on the System.nanoTime() clock at which the data was acquired. If no
     * timestamp is associated with this particular set of data, this value is zero */
    public final long nanoTime;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public Quaternion(double w, double x, double y, double z, long nanoTime)
    {
        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;
        this.nanoTime = nanoTime;
    }

    public Quaternion(I2cDeviceSynch.TimestampedData ts, long scale)
    {
        ByteBuffer buffer = ByteBuffer.wrap(ts.data).order(ByteOrder.LITTLE_ENDIAN);

        final double inverseScale = 1.0 / (double)scale;
        this.w = buffer.getShort() * inverseScale;
        this.x = buffer.getShort() * inverseScale;
        this.y = buffer.getShort() * inverseScale;
        this.z = buffer.getShort() * inverseScale;
        this.nanoTime = ts.nanoTime;
    }

    //----------------------------------------------------------------------------------------------
    // Operations
    //----------------------------------------------------------------------------------------------

    public double magnitude()
    {
        return Math.sqrt(w*w + x*x + y*y + z*z);
    }

    public Quaternion normalized()
    {
        final double mag = 1.0 / this.magnitude();
        return new Quaternion(
            w * mag,
            x * mag,
            y * mag,
            z * mag,
            this.nanoTime);
    }

    public Quaternion congugate()
    {
        return new Quaternion(w, -x, -y, -z, this.nanoTime);
    }

    /**
     * Returns euler angles that represent the quaternion.  Angles are
     * returned in rotation order and right-handed about the specified
     * axes:
     * <p/>
     * yaw is applied 1st about z
     * roll is applied 2nd about y
     * pitch is applied 3rd about x
     * <p/>
     */
    public EulerAngle toEuler ()
    {
        double sqw = w*w;
        double sqx = x*x;
        double sqy = y*y;
        double sqz = z*z;

        return new EulerAngle(
             Math.toDegrees(Math.atan2(2.0 * (x * y + z * w), (sqx - sqy - sqz + sqw))),
             Math.toDegrees(Math.asin(-2.0 * (x * z - y * w) / (sqx + sqy + sqz + sqw))),
             Math.toDegrees(Math.atan2(2.0 * (y * z + x * w), (-sqx - sqy + sqz + sqw))),
             this.nanoTime);
    }

    public Quaternion mult(Quaternion q)
    {
        final double A, B, C, D, E, F, G, H;
        A = (q.w + q.x)*(w + x);
        B = (q.z - q.y)*(y - z);
        C = (q.w - q.x)*(y + z);
        D = (q.y + q.z)*(w - x);
        E = (q.x + q.z)*(x + y);
        F = (q.x - q.z)*(x - y);
        G = (q.w + q.y)*(w - z);
        H = (q.w - q.y)*(w + z);

        return new Quaternion(
            B + (-E - F + G + H) / 2,
            A - (E + F + G + H) / 2,
            C + (E - F + G - H) / 2,
            D + (E - F - G + H) / 2,
            System.nanoTime()
        );
    }

    public Quaternion plus(Quaternion q)
    {
        return new Quaternion(w + q.w, x + q.x, y + q.y, z + q.z, System.nanoTime());
    }

}
