package frc.robot.util.shooter;

public class ShooterParameter {
    public double distance;
    public int rpm;
    public double angle;

    public ShooterParameter() {}

    public ShooterParameter(double distance, int rpm, double angle) {
        this.distance = distance;
        this.rpm = rpm;
        this.angle = angle;
    }

    /**
     * Interpolate between two ShooterParameter points (this one, and p2).
     * The interpolation is done using the line formed between the two parameters, defined by y = ((y_2 - y_1) / (x_2 - x_1))(x) + y_1. x would be the literal
     * distance between point 1 and the actual distance.
     * 
     * p2.distance > this.distance
     * 
     * @param p2 The second known point
     * @param deltaX The x distance between point 1 and the point you are trying to interpolate
     * @return The interpolated point
     */
    public ShooterParameter interpolate(ShooterParameter p2, double deltaX) {
        double rpmSlope = (p2.rpm - this.rpm) / (p2.distance - this.distance);
        int interpolatedRpm = (int) (rpmSlope * deltaX + this.rpm);

        double angleSlope = (p2.angle - this.angle) / (p2.distance - this.distance);
        double interpolatedAngle = (angleSlope * deltaX + this.angle);

        return new ShooterParameter(this.distance + deltaX, interpolatedRpm, interpolatedAngle);
    }
}
