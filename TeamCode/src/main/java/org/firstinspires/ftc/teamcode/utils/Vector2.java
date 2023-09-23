package org.firstinspires.ftc.teamcode.utils;

public class Vector2 {
    public double x;
    public double y;
    private double magcache;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2() {
        x = y = 0;
    }

    public static Vector2 add(Vector2 a, Vector2 b) {
        return new Vector2(a.x + b.x, a.y + b.y);
    }

    public double mag() {
        if (magcache == 0 && (x != 0 || y != 0)) {
            magcache = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        }

        return magcache;
    }

    public void mul(double a) {
        x *= a;
        y *= a;
        magcache *= a;
    }


    public static double dot(Vector2 a,Vector2 b) {
        return (a.x*b.x + a.y*b.y);
    }

    public void norm() {
        double mag = mag();

        if (mag == 0) {
            return;
        }

        x /= mag;
        y /= mag;
        magcache = 1;
    }

    public void add(Vector2 a) {
        x += a.x;
        y += a.y;
        magcache = 0;
    }

    public String toString() {
        return String.format("(%f, %f)", x, y);
    }

    public void rotate(double angle) {
        x = x*Math.cos(angle) + y*Math.sin(angle);
        y = x*-Math.sin(angle) + y*Math.cos(angle);
    }
    public static Vector2 rotate(Vector2 vector, double angle) {
        double x = vector.x;
        double y = vector.y;
        x = x*Math.cos(angle) + y*Math.sin(angle);
        y = x*-Math.sin(angle) + y*Math.cos(angle);
        return new Vector2(x, y);
    }
    public void rotateAround(double angle, double x, double y) {
        this.x -= x;
        this.y -= y;
        rotate(angle); //idk if is scuffed or not lmao -Kyle
        this.x +=x;
        this.y +=y;
    }
    public static Vector2 rotateAround(Vector2 vector, double angle, double x, double y) {
        double vx = vector.x-x;
        double vy = vector.y-y;
        Vector2 temp = Vector2.rotate(new Vector2(vx,vy), angle);
        return new Vector2(temp.x+x, temp.y+y);
    }
}