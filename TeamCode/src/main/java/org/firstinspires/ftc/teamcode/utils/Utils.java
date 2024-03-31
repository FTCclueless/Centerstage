package org.firstinspires.ftc.teamcode.utils;

public class Utils {
    public Utils() {}

    public static double minMaxClip(double value, double min, double max) {
        return Math.min(Math.max(min, value), max);
    }

    public static int minMaxClipInt(double value, double min, double max) {
        return (int) Math.min(Math.max(min, value), max);
    }

    public static double headingClip(double value) {
        while(value >= Math.PI) {
            value -= 2*Math.PI;
        }
        while(value <= -Math.PI) {
            value += 2*Math.PI;
        }
        return value;
    }

    public static boolean withinThreshold(double value, double minThreshold, double maxThreshold) {
        return value > minThreshold && value < maxThreshold;
    }

    public static double kalmanFilter (double value1, double value2, double value2Weight) {
        return (value1 * (1.0-value2Weight)) + (value2 * value2Weight);
    }
}
