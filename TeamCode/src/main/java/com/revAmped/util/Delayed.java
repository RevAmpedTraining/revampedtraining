package com.revAmped.util;

/**
 * Created by zwang on 9/2/2015.
 */
public class Delayed
{
    private long m_lastTimeStamp = 0;

    public void reset ()
    {
        m_lastTimeStamp = 0;
    }

    /**
     * whether delay enough time1
     * @param timeStamp current time stamp
     * @return true if the delay is enough
     */
    public boolean isDelayed (long timeStamp)
    {
        final int DELAY_MILLI = 50;
        if (m_lastTimeStamp == 0)
        {
            m_lastTimeStamp = timeStamp;
            return false;
        }

        // delay
        return (Math.abs(timeStamp - m_lastTimeStamp) >= DELAY_MILLI);
    }
}
