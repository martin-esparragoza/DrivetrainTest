package org.firstinspires.ftc.teamcode.libswerve;

public class Vector2 {
    public double x;
    public double y;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Vector2 add(Vector2 a, Vector2 b) {
        return new Vector2(a.x + b.x, a.y + b.y);
    }

    public double mag() {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public void mul(double a) {
        x *= a;
        y *= a;
    }
}
