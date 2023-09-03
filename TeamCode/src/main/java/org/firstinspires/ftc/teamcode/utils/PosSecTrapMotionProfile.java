package org.firstinspires.ftc.teamcode.utils;

public class PosSecTrapMotionProfile {
    double maxAccel;
    double maxVel;
    double startPos;
    double targetPos;

    double accelTime = 0;
    double accelDist;
    double cruiseDist;
    double cruiseTime;
    double decelDist;
    double decelTime;
    double traveledDist;
    double distance;

    double halfTime;

    double startVel;

    double sign;

    double startTime;








    public PosSecTrapMotionProfile(double maxAccel, double maxVel) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;




    }

    public void setDistance(double targetPos, double currentPos, double currentVel, double time) {
        startTime = time;
        this.targetPos = targetPos;
        distance = targetPos-currentPos;
        startPos = currentPos;
        startVel = currentVel;
        accelTime = (maxVel-currentVel)/maxAccel;
        decelTime = maxVel / maxAccel;
        decelDist = 0.5 * maxAccel * decelTime * decelTime;
        accelDist = 0.5 * maxAccel* accelTime * accelTime + startVel * accelTime; //probably broken maybe

        sign = Math.signum(distance);

        double tempMaxVel = maxVel;
        if (accelDist > Math.abs(distance/2)) {
            halfTime = Math.sqrt(Math.abs(distance/2) / (0.5*maxAccel)) - currentVel/maxAccel;
            tempMaxVel = halfTime*maxAccel;

            accelTime = halfTime;
            accelDist = 0.5 * maxAccel* accelTime * accelTime + startVel * accelTime; //
        }

        cruiseDist = Math.abs(distance)-accelDist;
        cruiseTime = cruiseDist/tempMaxVel;
        decelTime = accelTime + cruiseTime + decelTime;







    }

    public double getTargetVel(double currentTime) {
        double elapseTime = currentTime-startTime;

        if (Math.abs(elapseTime) < accelTime)  {
            return elapseTime * maxAccel * maxAccel + startVel *elapseTime;
        }

        else if (Math.abs(traveledDist) < cruiseTime) {
            return maxVel * sign;
        }

        else if (Math.abs(traveledDist) < decelDist) {
            return maxVel-maxAccel*((2 * traveledDist/maxAccel) * sign);
        }

        else {
            return 0;
        }


    }
}
