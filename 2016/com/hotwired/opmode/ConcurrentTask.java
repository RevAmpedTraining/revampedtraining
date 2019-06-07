package com.revAmped.opmode;

import android.util.Log;

import com.revAmped.util.HwLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.TimeUnit;

/**
 * Created by zwang on 8/29/2015.
 */
public abstract class ConcurrentTask implements Task, Runnable {
    private LinearOpMode m_opMode;
    private volatile boolean m_isActive = false;
    private long m_startTime = 0L;
    private Thread m_thread = null;

    public ConcurrentTask(LinearOpMode opMode) {
        m_opMode = opMode;
        m_thread = new Thread(this);
        m_thread.start();
        resetStartTime();
    }

    public abstract void runOpMode() throws InterruptedException;

    public LinearOpMode getLinearOpMode() {
        return m_opMode;
    }

    public void join() throws InterruptedException {
        m_thread.join();
    }

    public void join(long milliseconds) throws InterruptedException {
        m_thread.join(milliseconds);
    }

    @Override
    public boolean threadIsNotInterrupted() {
        return !m_thread.isInterrupted();
    }

    @Override
    public void sleep(long milliseconds) throws InterruptedException {
        Thread.sleep(milliseconds);
    }

    @Override
    public void idle() throws InterruptedException {
        Thread.yield();
    }

    @Override
    public void stop() {
        if (m_isActive) {
            m_thread.interrupt();
            m_isActive = false;
        }
    }

    @Override
    public boolean opModeIsActive() {
        return m_isActive && m_opMode.opModeIsActive();
    }

    @Override
    public double getRuntime() {
        double nanoPerSecond = TimeUnit.SECONDS.toNanos(1L);
        return (System.nanoTime() - m_startTime) / nanoPerSecond;
    }

    @Override
    public void resetStartTime() {
        m_startTime = System.nanoTime();
    }

    @Override
    public void run() {
        m_isActive = true;

        try {
            runOpMode();
        } catch (InterruptedException ie) {
            if (HwLog.isLoggable(Log.INFO)) {
                HwLog.i("ConcurrentTask " + m_thread.getId() + " received an Interrupted Exception; shutting down");
            }
        } catch (RuntimeException re) {
            if (HwLog.isLoggable(Log.INFO)) {
                HwLog.i("ConcurrentTask " + m_thread.getId() + " received an Runtime Exception; shutting down", re);
            }
            //throw re;
        } finally {
            m_isActive = false;
        }
    }
}
