package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

public class IMUMergeLocalizer extends Localizer {
    public IMUMergeLocalizer(HardwareMap hardwareMap, Sensors sensors, Drivetrain drivetrain, String color, String expectedColor) {
        super(hardwareMap, sensors, drivetrain, color, expectedColor);
    }

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

        //This is the heading because the heading is proportional to the difference between the left and right wheel.
        double deltaHeading = ((deltaRight - deltaLeft)/(leftY-rightY)) * 0.20 + (sensors.getOtosHeading() - sensors.getLastOtosHeading()) * 0.8;
        //This gives us deltaY because the back minus theta*R is the amount moved to the left minus the amount of movement in the back encoder due to change in heading
         relDeltaY = deltaBack - deltaHeading*backX;
        //This is a weighted average for the amount moved forward with the weights being how far away the other one is from the center
         relDeltaX = (deltaRight*leftY - deltaLeft*rightY)/(leftY-rightY);

        // constant accel
        Pose2d lastRelativePose = relHistory.get(0);

        double lastLoop = loopTime;

        if (nanoTimes.size() > 1) {
            lastLoop = (nanoTimes.get(0) - nanoTimes.get(1))/1.0E9;
        }

        double arx = (relDeltaX*lastLoop - lastRelativePose.x*loopTime)/(loopTime*lastLoop*lastLoop + loopTime*loopTime*lastLoop);
        double vrx = relDeltaX/loopTime - arx*loopTime;
        //v_x = vrx + arx*t
        double ary = (relDeltaY*lastLoop - lastRelativePose.y*loopTime)/(loopTime*lastLoop*lastLoop + loopTime*loopTime*lastLoop);
        double vry = relDeltaY/loopTime - ary*loopTime;
        //v_y = vry + ary*t
        double arh = (deltaHeading*lastLoop - lastRelativePose.heading*loopTime)/(loopTime*lastLoop*lastLoop + loopTime*loopTime*lastLoop);
        double vrh = deltaHeading/loopTime - arh*loopTime;
        //h = h1 + vry*t + ary*t^2

        AdaptiveQuadrature xQuadrature = new AdaptiveQuadrature(new double[] {vrx,2*arx},new double[] {heading,vrh,arh});
        AdaptiveQuadrature yQuadrature = new AdaptiveQuadrature(new double[] {vry,2*ary},new double[] {heading,vrh,arh});

        x += xQuadrature.evaluateCos(fidelity, 0, loopTime, 0) - yQuadrature.evaluateSin(fidelity, 0, loopTime, 0);
        y += yQuadrature.evaluateCos(fidelity, 0, loopTime, 0) + xQuadrature.evaluateSin(fidelity, 0, loopTime, 0);

        relHistory.add(0,new Pose2d(relDeltaX,relDeltaY,deltaHeading));

        heading += (sensors.getOtosHeading() - sensors.getLastOtosHeading());

        currentPose = new Pose2d(x, y, heading);

        nanoTimes.add(0, currentTime);
        poseHistory.add(0,currentPose.clone());

        updateVelocity();
        updateExpected();
        updateField();
    }
}
