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

@Config
public class AutoPathConstants {
    // Internally defined robot starting position
    public static Pose2d startUp =                   new Pose2d(12, -62, Math.PI / 2);
    public static Pose2d startDown =                 new Pose2d(-36, 62, Math.PI / 2);

    // Just to the left of the red side board
    public static Pose2d parkingLocation =           new Pose2d(60, -12, 0);

    // Offsets from the parking location to the position of the preload
    // These are offsets because there are different starting locations
    public static Pose2d groundPreloadTopOffset =    new Pose2d(0, 14, Math.PI / 2);
    public static Pose2d groundPreloadCenterOffset = new Pose2d(0, 14, Math.PI / 2);
    public static Pose2d groundPreloadBottomOffset =  new Pose2d(0, 14, Math.PI / 2);

    // Initial deposit. Should be in front of the red side board
    public static Pose2d boardPreload =              new Pose2d(48, -36, Math.PI);

    // First intake if we are on the bottom side of field. Should be the rightmost intake stack (from current origin)
    public static Pose2d initialIntake =             new Pose2d(-60, -36, Math.PI);

    public static Pose2d depositLocation =           new Pose2d(60, -12, 0);
}
