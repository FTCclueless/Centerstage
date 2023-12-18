package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class DoublePreloadAutoRed extends DoublePreloadAuto {
    // Constructor must be called at some point bc runopmode is not static

    public DoublePreloadAutoRed() {
        super();
        up = true;
        red = true;
    }
}
