package org.firstinspires.ftc.teamcode.Swerve;

/**
 * Created by Justin on 10/8/2016.
 */
public class Vector {
    double x = 0, y = 0;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getAngle() {
        return Math.atan2(y, x);
    }

    public double getMagnitude() {
        return Math.hypot(x, y);
    }

    public void scale(double scalar) {
        x *= scalar;
        y *= scalar;
    }

    public void add(Vector v) {
        x += v.x;
        y += v.y;
    }

    public void subtract(Vector v) {
        x -= v.x;
        y -= v.y;
    }

    public void makePerpendicular() {
        double temp = x;
        x = y;
        y = -temp;
    }

    public static double dotProduct(Vector v1, Vector v2){
        return v1.x*v2.x+v1.y*v2.y;
    }

}
