package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utils.Pose2d;

/**
 * Auto Path Constants <br>
 * <br>
 * All the points that an auto would need. <br>
 * Makes an easy and tunable auto. <br>
 * These constants are ALL for red sided auto
 */

// TODO: REMEMBER RED SIDED AUTO IS REFERENCE POINT FOR ALL POINTS

@Config
public class AutoPathConstants {
    // Internally defined robot starting position
    public static Pose2d startUp =                   new Pose2d(12, -58, -Math.PI / 2);
    public static Pose2d startDown =                 new Pose2d(-36, -58, -Math.PI / 2);

    // Just to the left of the red side board
    public static Pose2d parkingLocation =           new Pose2d(60, -12, 0);

    // ground preload position offsets
    public static Pose2d groundPreloadLeftOffset =   new Pose2d(-4, 24, 0);
    public static Pose2d groundPreloadCenterOffset = new Pose2d(0, 24, 0);
    public static Pose2d groundPreloadRightOffset =  new Pose2d(5, 24, 0);

    // Initial deposit. Should be in fr ont of the red side board
    public static Pose2d boardPreload =              new Pose2d(48, -36, Math.PI);

    // First intake if we are on the bottom side of field. Should be the rightmost intake stack (from current origin)
    public static Pose2d initialIntake =             new Pose2d(-58, -36, Math.PI);

    public static Pose2d depositLocation =           new Pose2d(50, -12, 0);
}
