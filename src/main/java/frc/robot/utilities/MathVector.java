package frc.robot.utilities;

public class MathVector {
    
    public double x;
    public double y;

    public MathVector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    // public Vector(double angle) {
    //     this.x = Math.cos(angle);
    //     this.y = Math.sin(angle);
    // }

    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public MathVector normalize() {
        double magnitude = getMagnitude();
        if (magnitude == 0) {
            return new MathVector(0, 0);
        }
        return new MathVector(x / magnitude, y / magnitude);
    }
    
    public MathVector add(MathVector other) {
        return new MathVector(x + other.x, y + other.y);
    }
    
    public MathVector subtract(MathVector other) {
        return new MathVector(x - other.x, y - other.y);
    }
    
    public double angle() {
        return Math.atan2(y, x);
    }
}