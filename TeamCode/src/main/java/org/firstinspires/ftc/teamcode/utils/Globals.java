package org.firstinspires.ftc.teamcode.utils;

;

public class Globals {
    // general
    public static long LOOP_START = System.nanoTime();
    public static double LOOP_TIME = 0.0;
    public static RunMode RUNMODE = RunMode.TESTER;

    // drivetrain
    public static boolean DRIVETRAIN_ENABLED = true;
    public static double TRACK_WIDTH = 14.352084806;
    public static double MIN_MOTOR_POWER_TO_OVERCOME_FRICTION = 0.1;
    public static double MAX_HEADING_SPEED = 11.969357287892013;
    public static double MAX_X_SPEED = 62.55301371449667;
    public static double MAX_Y_SPEED = 38.736508988947904;
    public static double START_HEADING_OFFSET = 0.0;

    public static Pose2d ROBOT_POSITION =new Pose2d(0,0,0);
    public static Pose2d ROBOT_VELOCITY = new Pose2d(0,0,0);
    public static Pose2d AUTO_ENDING_POSE = new Pose2d(0,0,0);

    // deposit
    public static int NUM_PIXELS = 0;

    // loop time methods
    public static void START_LOOP() {
        LOOP_START = System.nanoTime();
    }

    public static double GET_LOOP_TIME() {
        LOOP_TIME = (System.nanoTime() - LOOP_START) / 1.0e9; // converts from nano secs to secs
        return LOOP_TIME;
    }
}
