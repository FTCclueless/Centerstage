package org.firstinspires.ftc.teamcode.vision.apriltags;

import org.firstinspires.ftc.teamcode.utils.Pose2d;

public class Pose2dWithTime {
    public Pose2d pose;
    public long time;

    public Pose2dWithTime(Pose2d pose, long time) {
        this.pose = pose;
        this.time = time;
    }
}
