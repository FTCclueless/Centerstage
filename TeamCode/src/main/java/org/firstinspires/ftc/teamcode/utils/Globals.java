package org.firstinspires.ftc.teamcode.utils;

public class Globals {
    // general
    public static long LOOP_START = System.nanoTime();
    public static double LOOP_TIME = 0.0;





    // drivetrain
    public static boolean DRIVETRAIN_ENABLED = true;
    public static double TRACK_WIDTH = 7.351173256 + 7.00091155; // 13.3750297033534346;
    public static double MIN_MOTOR_POWER_TO_OVERCOME_FRICTION = 0.1;
    public static double MAX_DRIVETRAIN_SPEED = 75;




    public static Pose2d ROBOT_POSITION =new Pose2d(0,0,0);
    public static Pose2d ROBOT_VELOCITY = new Pose2d(0,0,0);

    // loop time methods
    public static void START_LOOP() {
        LOOP_START = System.nanoTime();
    }

    public static double GET_LOOP_TIME() {
        LOOP_TIME = (System.nanoTime() - LOOP_START) / 1.0e9; // converts from nano secs to secs
        return LOOP_TIME;
    }
}
