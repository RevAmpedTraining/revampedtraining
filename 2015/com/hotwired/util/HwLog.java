package com.revAmped.util;

import android.util.Log;

/**
 * Created by zwang on 8/29/2015.
 */
public class HwLog {
    public static final String TAG = "HotWired";
    public static final String ERROR_PREPEND = "### ERROR: ";

    public static void v(String message) {
        Log.v(TAG, message);
    }

    public static void d(String message) {
        Log.d(TAG, message);
    }

    public static void i(String message) {
        Log.i(TAG, message);
    }

    public static void i(String message, Exception t) {
        Log.i(TAG, message);
        logIStacktrace(t);
    }

    public static void w(String message) {
        Log.w(TAG, message);
    }

    public static void w(String message, Exception t) {
        Log.w(TAG, message);
        logWStacktrace(t);
    }

    public static void error(String message) {
        e(message);
    }

    public static void error(String format, Object... args) {
        e(String.format(format, args));
    }

    public static void e(String message) {
        Log.e(TAG, ERROR_PREPEND + message);
    }

    public static void e(String message, Exception t) {
        Log.e(TAG, ERROR_PREPEND + message);
        logEStacktrace(t);
    }

    public static boolean isLoggable(int level) {
        return Log.isLoggable(TAG, level);
    }

    public static void logIStacktrace(Exception e) {
        i(e.toString());
        for (StackTraceElement localStackTraceElement : e.getStackTrace()) {
            i(localStackTraceElement.toString());
        }
    }

    public static void logWStacktrace(Exception e) {
        w(e.toString());
        for (StackTraceElement localStackTraceElement : e.getStackTrace()) {
            w(localStackTraceElement.toString());
        }
    }

    public static void logEStacktrace(Exception e) {
        e(e.toString());
        for (StackTraceElement localStackTraceElement : e.getStackTrace()) {
            e(localStackTraceElement.toString());
        }
    }
}
