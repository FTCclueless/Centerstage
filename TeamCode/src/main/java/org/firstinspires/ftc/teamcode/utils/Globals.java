package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

;

@Config
public class Globals {
    // general
    public static long LOOP_START = System.nanoTime();
    public static double LOOP_TIME = 0.0;
    public static RunMode RUNMODE = RunMode.TESTER;
    public static boolean isRed = false;
    public static long autoStartTime = -1;
    public static boolean gotBloodyAnnihilated = false; // STOP DELETEING THIS FOR GODS SAKE
    // drivetrain
    public static boolean DRIVETRAIN_ENABLED = true;
    public static double TRACK_WIDTH = 9.75;
    public static double MIN_MOTOR_POWER_TO_OVERCOME_FRICTION = 0.1;
    public static double MAX_HEADING_SPEED = 9.046897651154636;
    public static double MAX_X_SPEED = 86.14575569046524;
    public static double MAX_Y_SPEED = 55.54759669214725;

    public static Pose2d ROBOT_POSITION = new Pose2d(0,0,0);
    public static Pose2d ROBOT_VELOCITY = new Pose2d(0,0,0);
    public static Pose2d AUTO_ENDING_POSE = new Pose2d(0,0, Math.toRadians(180));

    // deposit
    public static int NUM_PIXELS = 0;
    public static double slidesV4Thresh = 5.0;

    // auto
    public static boolean mergeUltrasonics = false;

    // loop time methods
    public static void START_LOOP() {
        LOOP_START = System.nanoTime();
    }

    public static double GET_LOOP_TIME() {
        LOOP_TIME = (System.nanoTime() - LOOP_START) / 1.0e9; // converts from nano secs to secs
        return LOOP_TIME;
    }
}
