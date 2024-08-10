package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

public class IMUMergeLocalizer extends Localizer {
    public IMUMergeLocalizer(HardwareMap hardwareMap, Sensors sensors, Drivetrain drivetrain, String color, String expectedColor) {
        super(hardwareMap, sensors, drivetrain, color, expectedColor);
    }

    @Override
    public void update() {
        long currentTime = System.nanoTime();
        double loopTime = (double)(currentTime-lastTime)/1.0E9;
        lastTime = currentTime;

        // Odometry
        double deltaLeft = encoders[0].getDelta();
        double deltaRight = encoders[1].getDelta();
        double deltaBack = encoders[2].getDelta();
        double leftY = encoders[0].y;
        double rightY = encoders[1].y;
        double backX = encoders[2].x;

        //This is the heading from the otos imu
        double deltaIMUHeading = (sensors.getOtosHeading() - sensors.getLastOtosHeading());
        //This is the heading because the heading is proportional to the difference between the left and right wheel.
        double deltaOdoHeading = (deltaRight - deltaLeft)/(leftY-rightY);
        //This is the heading given by taking 20% from the odo and 80% from the otos
        double deltaHeading = deltaIMUHeading*0.8 + deltaOdoHeading*0.2;
        //This gives us deltaY because the back minus theta*R is the amount moved to the left minus the amount of movement in the back encoder due to change in heading
        relDeltaY = deltaBack - deltaHeading*backX;
        //This is a weighted average for the amount moved forward with the weights being how far away the other one is from the center
        relDeltaX = (deltaRight*leftY - deltaLeft*rightY)/(leftY-rightY);
        distanceTraveled += Math.sqrt(relDeltaX*relDeltaX+relDeltaY*relDeltaY);

        // constant accel
        Pose2d relDelta = new Pose2d(relDeltaX,relDeltaY,deltaHeading);
        constAccelMath.calculate(loopTime,relDelta,currentPose);

        x = currentPose.x;
        y = currentPose.y;

        heading = currentPose.heading = sensors.getOtosHeading();

        relHistory.add(0,relDelta);
        nanoTimes.add(0, currentTime);
        poseHistory.add(0,currentPose.clone());

        updateVelocity();
        updateExpected();
        updateField();
    }
}
