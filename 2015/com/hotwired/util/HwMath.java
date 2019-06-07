package com.revAmped.util;

/**
 * Created by zwang on 9/2/2015.
 */
public class HwMath {
    public static int sign(int s, int v) {
        if (s >= 0 ) {
            return v;
        }
        else {
            return -v;
        }
    }

    public static float sign(float s, float v) {
        if (s >= 0.0 ) {
            return v;
        }
        else {
            return -v;
        }
    }

    public static float sign(int s, float v) {
        if (s >= 0 ) {
            return v;
        }
        else {
            return -v;
        }
    }

    public static double sign(double s, double v) {
        if (s >= 0.0 ) {
            return v;
        }
        else {
            return -v;
        }
    }
}
