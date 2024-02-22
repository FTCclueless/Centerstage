package org.firstinspires.ftc.teamcode.subsystems.drive;

import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

import java.util.ArrayList;

class SplinePose2d extends Pose2d {
    public final boolean reversed;
    public final double radius;
    public final double power;

    public SplinePose2d(Pose2d p, boolean reversed, double radius) {
        this(p.x, p.y, p.heading, reversed, radius, 1.0);
    }

    public SplinePose2d(Pose2d p, boolean reversed, double radius, double power) {
        this(p.x, p.y, p.heading, reversed, radius, power);
    }

    public SplinePose2d(double x, double y, double heading, boolean reversed, double radius, double power) {
        super(x, y, heading);
        this.reversed = reversed;
        this.radius = radius;
        this.power = power;
    }
}

public class Spline {
    public ArrayList<SplinePose2d> poses = new ArrayList<>();

    public final double inchesPerNewPointGenerated;
    private boolean reversed = false;


    public Spline(Pose2d p, double inchesPerNewPointGenerated) {
        poses.add(new SplinePose2d(p, false, 100));

        this.inchesPerNewPointGenerated = inchesPerNewPointGenerated;
    }

    public Spline(double x, double y, double heading, double inchesPerNewPointGenerated) {
        this(new Pose2d(x,y,heading), inchesPerNewPointGenerated);
    }

    double[] xCoefficents = new double[4];
    double[] yCoefficents = new double[4];

    public double findR(double time){
        // gets the velocity because the derivative of position = velocity
        double velX = xCoefficents[1] + 2.0*xCoefficents[2]*time + 3.0*xCoefficents[3]*time*time;
        double velY = yCoefficents[1] + 2.0*yCoefficents[2]*time + 3.0*yCoefficents[3]*time*time;

        // gets the acceleration which is second derivative of position
        double accelX = 2.0*xCoefficents[2] + 6.0*xCoefficents[3]*time;
        double accelY = 2.0*yCoefficents[2] + 6.0*yCoefficents[3]*time;
        if ((accelY * velX - accelX * velY) != 0) {
            return Math.min(Math.abs(Math.pow(velX * velX + velY * velY, 1.5) / (accelY * velX - accelX * velY)),100);
        }
        return 100; // straight line
    }

    public Spline addPoint(Pose2d p) {
        return this.addPoint(p, 1.0);
    }

    public Spline addPoint(Pose2d p, double power) {
        p = p.clone();
        if (reversed) {
            p.heading += Math.PI;
        }

        // https://www.desmos.com/calculator/yi3jovk0hp
        Pose2d point = new Pose2d(0,0,0);

        Pose2d lastPoint = poses.get(poses.size()-1); // when you add a new spline the last point becomes the starting point for the new spline

        double arbitraryVelocity = 1.25*Math.sqrt(Math.pow((lastPoint.x - p.x),2) + Math.pow((lastPoint.y - p.y),2));
        xCoefficents[0] = lastPoint.x;
        xCoefficents[1] = arbitraryVelocity * Math.cos(lastPoint.heading);
        xCoefficents[2] = 3*p.x - arbitraryVelocity*Math.cos(p.heading) - 2*xCoefficents[1] - 3*xCoefficents[0];
        xCoefficents[3] = p.x - xCoefficents[0] - xCoefficents[1] - xCoefficents[2];

        yCoefficents[0] = lastPoint.y;
        yCoefficents[1] = arbitraryVelocity * Math.sin(lastPoint.heading);
        yCoefficents[2] = 3*p.y - arbitraryVelocity*Math.sin(p.heading) - 2*yCoefficents[1] - 3*yCoefficents[0];
        yCoefficents[3] = p.y - yCoefficents[0] - yCoefficents[1] - yCoefficents[2];

        point.x = xCoefficents[0];
        point.y = yCoefficents[0];

        double firstR = findR(0);
        if (Double.isNaN(firstR) || Double.isInfinite(firstR)) {
            System.out.println("HOLY JESUS SOMETHING BAD HAPPENED (FIRST TEMPR IS BRICKED)");
        }

        poses.set(0, new SplinePose2d(poses.get(0).x, poses.get(0).y, poses.get(0).heading, poses.get(0).reversed, firstR, power));

        for (double time = 0.0; time < 1.0; time+=0.001) {
            point = new Pose2d(0,0,0);

            point.x = xCoefficents[0] + xCoefficents[1]*time + xCoefficents[2]*time*time + xCoefficents[3]*time*time*time;
            point.y = yCoefficents[0] + yCoefficents[1]*time + yCoefficents[2]*time*time + yCoefficents[3]*time*time*time;

            if(lastPoint.getDistanceFromPoint(point) > inchesPerNewPointGenerated) { // new point every two inches

                // gets the velocity because the derivative of position = velocity
                double velX = xCoefficents[1] + 2.0*xCoefficents[2]*time + 3.0*xCoefficents[3]*time*time;
                double velY = yCoefficents[1] + 2.0*yCoefficents[2]*time + 3.0*yCoefficents[3]*time*time;
                // heading is equal to the inverse tangent of velX and velY because velX and velY have a magnitude and a direction and soh cah toa
                point.heading = Math.atan2(velY,velX);
                point.clipAngle();

                poses.add(new SplinePose2d(point, reversed, findR(time), power));
                System.out.println("pathIndex: " + poses.size() + " radius: " + findR(time));

                lastPoint = point;
            }
        }

        poses.add(new SplinePose2d(p, reversed, findR(1.0), power));

        return this;
    }

    public Spline addPoint(double x, double y, double heading) {
        return this.addPoint(new Pose2d(x, y, heading), 1.0);
    }

    public Spline addPoint(double x, double y, double heading, double power) {
        return this.addPoint(new Pose2d(x, y, heading), power);
    }

    public Pose2d getLastPoint() {
        if (poses.size() > 0) {
            return poses.get(poses.size() - 1);
        }
        return null;
    }

    /*public void setPoint(int index, Pose2d point, boolean reversed, double radius) {
        poses.set(index, new SplinePose2d(point, reversed, radius));
    }

    public void setPoint(int index, SplinePose2d point, double heading) {
        poses.set(index, new SplinePose2d(point.x, point.y, heading, point.reversed, point.radius));
    }*/


    /**
     * Ideally the path should be behind it otherwise it would break
     * So if you do it wrong its your fault!
     * @param reversed
     * @return
     */
    public Spline setReversed(boolean reversed) {
        if (reversed) {
            poses.get(poses.size() - 1).heading += Math.PI;
        }
        this.reversed = reversed;
        return this;
    }

    /*public static Spline reflecth(Spline spline) {
        Spline temp = new Spline(new Pose2d(0,0,0), spline.inchesPerNewPointGenerated);
        temp.poses = new ArrayList<>();
        for (SplinePose2d point : spline.poses) {
            temp.poses.add(new SplinePose2d(point.x,-point.y, AngleUtil.clipAngle(point.heading + Math.PI),point.reversed,point.radius));
        }
        return temp;
    }

    public static Spline reflect(Spline spline) {
        Spline temp = new Spline(new Pose2d(0,0,0), spline.inchesPerNewPointGenerated);
        temp.poses = new ArrayList<>();
        for (SplinePose2d point : spline.poses) {
            temp.poses.add(new SplinePose2d(point.x,-point.y,point.heading,point.reversed,point.radius));
        }
        return temp;
    }*/
}