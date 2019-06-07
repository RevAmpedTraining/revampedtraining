package com.revAmped.util;

/**
 * gradually increase the power to prevent tip over.
 */

public class TipOverProtection {
    private boolean isForward;

    private long moveStartTime = System.currentTimeMillis();
    private long protectStartTime = System.currentTimeMillis();
    private boolean isProtect = false;
    private boolean isPowerAdjusted = false;
    private float pwr = 0;
    private float pwl = 0;

    private final static float PROTECT_PERIOD = 1200f;
    private final static float POWER_THRESHOLD = 0.35f;
    private final static float POWER_RATIO = 0.5f;

    public TipOverProtection (boolean isForward) {
        this.isForward = isForward;
    }

    public void protect (float pwl, float pwr) {
        isPowerAdjusted = false;
        long timestamp = System.currentTimeMillis();
        //prevent tipping
        if ((isForward && pwl > POWER_THRESHOLD && pwr > POWER_THRESHOLD) ||
                (!isForward && pwl < -POWER_THRESHOLD && pwr < -POWER_THRESHOLD)){
            // move
            protectStartTime = timestamp;
            // move for sometime to start protection
            isProtect = (timestamp - moveStartTime > 400);
        }
        else {
            // opposite direction
            moveStartTime = timestamp;
            long protectDuration = timestamp - protectStartTime;
            if (isProtect && protectDuration < PROTECT_PERIOD) {
                if ((isForward && pwl < -POWER_THRESHOLD && pwr < -POWER_THRESHOLD) ||
                        (!isForward && pwl > POWER_THRESHOLD && pwr > POWER_THRESHOLD)){
                    this.pwl = pwl* POWER_RATIO *protectDuration/ PROTECT_PERIOD;
                    this.pwr = pwr* POWER_RATIO *protectDuration/ PROTECT_PERIOD;
                    isPowerAdjusted = true;
                }
            }
            else {
                isProtect = false;
            }
        }
    }

    public boolean isPowerAdjusted () {
        return isPowerAdjusted;
    }

    public float getLeftPower () {
        return pwl;
    }

    public float getRightPower () {
        return pwr;
    }
}
