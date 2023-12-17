package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Vector3;

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
    public static Pose2d startUp =                     new Pose2d(12, -65, -Math.PI / 2);
    public static Pose2d startDown =                   new Pose2d(-36, -65, -Math.PI / 2);

    // Just to the left of the red side board
    public static Pose2d intermediateParkingLocation = new Pose2d(42, -60, 0);
    public static Pose2d parkingLocation =             new Pose2d(58, -60, 0);

    // ground preload position offsets
    // TODO RENAME THEY R NOT RIGHT
    public static Pose2d groundPreloadLeftOffset =     new Pose2d(4, 26.5, 0);

    public static Pose2d groundPreloadCenterOffset =   new Pose2d(6, 29.5, 0);

    public static Pose2d groundPreloadRightOffset =    new Pose2d(18, 26.5, 0);

    public static Pose2d blueGroundPreloadLeftOffset =     new Pose2d(18, 26.5, 0);

    public static Pose2d blueGroundPreloadRightOffset =    new Pose2d(-4, 26.5, 0);

    public static double groundPreloadStrafeOffset =   6.75;
    // Initial deposit. Should be in front of the red side board
    public static Pose2d boardPreload =                new Pose2d(46, -36, Math.PI);
    public static Vector3 boardPreloadLeftDeposit =    new Vector3(0.48, 4.5, -5.5);
    public static Vector3 boardPreloadCenterDeposit =  new Vector3(0.4, 0, -6);
    public static Vector3 boardPreloadRightDeposit =  new Vector3(0.65, -3.5, 2);


    public static Vector3 blueBoardPreloadLeftDeposit =    new Vector3(0.48, 4.5, -5.5);
    public static Vector3 blueBoardPreloadCenterDeposit =  new Vector3(0.33, 0, -6);
    public static Vector3 blueBoardPreloadRightDeposit =  new Vector3(0.65, -3.5, 2);

    // First intake if we are on the bottom side of field. Should be the rightmost intake stack (from current origin)
    public static Pose2d initialIntake =               new Pose2d(-58, -36, Math.PI);

    public static Pose2d sideDepositLocation =         new Pose2d(48, -12, 0);

    public static Pose2d targetBoard =                 new Pose2d(61,-36,0);

}
