package com.revAmped.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by zwang on 8/25/2015.
 */
public abstract class LinearTask extends LinearOpMode implements Task {

    /**
     * whether the current thread is not interrupted
     *
     * @return true if the current thread is not interrupted
     */
    @Override
    public boolean threadIsNotInterrupted() {
        return !Thread.currentThread().isInterrupted();
    }
}
