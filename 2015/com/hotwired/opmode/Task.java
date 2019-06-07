package com.revAmped.opmode;

/**
 * Created by zwang on 8/29/2015.
 */
public interface Task {

    public boolean threadIsNotInterrupted();

    public void idle() throws InterruptedException;

    public void sleep(long milliseconds) throws InterruptedException;

    public boolean opModeIsActive();

    public void stop();

    public double getRuntime();

    public void resetStartTime();
}
