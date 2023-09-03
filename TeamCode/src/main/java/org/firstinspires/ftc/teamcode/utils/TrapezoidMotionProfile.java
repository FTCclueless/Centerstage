package org.firstinspires.ftc.teamcode.utils;

public class TrapezoidMotionProfile {
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
    public double traveledDist;
    double distance;

    double halfTime;

    double startVel;
    double offsetDist;

    double sign;



    double decelThreshold;

    double tempMaxVel;




    public TrapezoidMotionProfile(double maxAccel, double maxVel, double offset) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
        this.offsetDist = offset;




    }

    //when you set target position/distance
    public void setTargetPos(double targetPos, double currentPos, double currentVel) {
        this.targetPos = targetPos;
        distance = targetPos-currentPos;
        sign = Math.signum(distance);
        startPos = currentPos;
        startVel = currentVel;
        accelTime = Math.abs((sign*maxVel-currentVel)/maxAccel);
        TelemetryUtil.packet.put("*** maxVel", maxVel);
        TelemetryUtil.packet.put("*** maxAccel", maxAccel);
        TelemetryUtil.packet.put("*** currentVel", currentVel);
        TelemetryUtil.packet.put("**accelTime", accelTime);

        decelTime = maxVel / maxAccel;
        decelDist = 0.5 * maxAccel * decelTime * decelTime; //positive
        accelDist = Math.abs(0.5 * sign * maxAccel* accelTime * accelTime); //probably broken maybe
        TelemetryUtil.packet.put("accelDist", accelDist);




        if (accelDist > Math.abs(distance/2)) {

            //tempTime = (-startVel + Math.sqrt(startVel*startVel + 4*maxAccel*sign*Math.abs(distance/2))) / 2 /maxAccel*sign; //broken, need to do
            //tempTime = Math.sqrt(sign*maxAccel*distance+(startVel*startVel/2))/maxAccel;
            TelemetryUtil.packet.put("halftime", halfTime);
            halfTime = Math.sqrt(Math.abs(distance/2) / (0.5*maxAccel)) - startVel /maxAccel;
            tempMaxVel = halfTime *maxAccel*sign + startVel * halfTime;
            TelemetryUtil.packet.put("tempMaxVel", tempMaxVel);

            accelTime = halfTime;
            decelTime = Math.abs(tempMaxVel/maxAccel);
            accelDist = 0.5 * maxAccel* accelTime * accelTime + startVel * accelTime; //
            decelDist = maxAccel*decelTime;
        }
        if (Math.abs(decelDist) >= Math.abs(distance) - offsetDist) {
            accelTime = 0;
            accelDist = 0;
        }

        cruiseDist = Math.abs(distance)-decelDist;

        decelThreshold = accelDist + cruiseDist + decelDist;







    }
//every loop to get target velocity
    public double getTargetVel(double currentPos) {
        //traveledDist = Math.abs(currentPos-startPos) + offsetDist; //offsetDist is to prevent power from permanently being 0
        TelemetryUtil.packet.put("traveledDist", traveledDist);

        TelemetryUtil.packet.put("** cruiseDist", cruiseDist);
        TelemetryUtil.packet.put("** decelThreshold", decelThreshold);
        TelemetryUtil.packet.put(" ** decelDist", decelDist);
        TelemetryUtil.packet.put("***prof dist", distance);


        TelemetryUtil.packet.put("**tempMaxVel", tempMaxVel);
        TelemetryUtil.packet.put("** startVel", startVel);
        System.out.println(accelDist);


        if (Math.abs(traveledDist) < accelDist + offsetDist || (sign == -1 && traveledDist > accelDist*sign) || (sign == 1 && traveledDist < accelDist))  {
            TelemetryUtil.packet.put("pain", 10);

            return maxAccel*(-startVel + Math.sqrt(startVel*startVel + 4 * maxAccel * sign * currentPos))/ (2*maxAccel*sign) * sign + startVel;

        }

        else if (Math.abs(traveledDist) < cruiseDist + offsetDist) {
            TelemetryUtil.packet.put("pain", 20);
            return maxVel * sign;
        }

        else if (Math.abs(traveledDist) < decelThreshold + offsetDist) {
            TelemetryUtil.packet.put("pain", 30);
            return maxAccel*(-startVel + Math.sqrt(startVel*startVel + 4 * maxAccel * sign* (distance-traveledDist)))/ (2 * maxAccel * sign) * sign;
        }

        else {
            TelemetryUtil.packet.put("pain", 40);
            return 0;

        }


    }
}
