package org.firstinspires.ftc.teamcode.utils;

public class Vector3 {
    public double x;
    public double y;
    public double z;
    private double magcache;

    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    public Vector3() {
        x=y=z=0;
    }



    public void add(Vector3 a) {
        x += a.x;
        y += a.y;
        z += a.z;
        magcache = 0;
    }
    public double getMag() {
        if (magcache == 0 && ( x != 0 || y != 0 || z != 0)) {
            magcache = Math.sqrt(Math.pow(x,2) + Math.pow(y,2) + Math.pow(z,2));
        }
        return magcache;
    }

    public void mul(double a) {
        x *=a;
        y *= a;
        z *= a;
        magcache *= a;
    }

    public void norm() {
        double mag = getMag();
        x /= mag;
        y /= mag;
        z /= mag;
        magcache = 1;
    }


    public static Vector3 add(Vector3 a, Vector3 b) {
        return new Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
    }
    public static double dot(Vector3 a, Vector3 b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
    public static Vector3 project(Vector3 v, Vector3 u) {
        double component = Vector3.dot(v,u)/u.getMag();
        u.norm();
        u.mul(component);
        return u;
    }




}