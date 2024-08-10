package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

public class OneHundredMSIMULocalizer extends Localizer {
    private long lastUpdate;

    public OneHundredMSIMULocalizer(HardwareMap hardwareMap, Sensors sensors, Drivetrain drivetrain, String color, String expectedColor) {
        super(hardwareMap, sensors, drivetrain, color, expectedColor);
        lastUpdate = System.currentTimeMillis();
    }

    @Override
    public void update() {
        super.update();
        if (System.currentTimeMillis() - lastUpdate > 100){
            heading = currentPose.heading = sensors.getOtosHeading();
            lastUpdate = System.currentTimeMillis();
        }
    }
}
