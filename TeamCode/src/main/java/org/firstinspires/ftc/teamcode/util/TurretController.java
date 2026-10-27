package org.firstinspires.ftc.teamcode.util;

public class TurretController {
    private final double kP;
    private final double kVP;
    private final double kV;

    private final double minAngle;
    private final double maxAngle;

    private double previousMeasurement = Double.NaN;
    private long lastTime = 0;
    private double measuredVelocity = 0;

    public TurretController(double kP, double kVP, double kV, double minAngle, double maxAngle) {
        this.kP = kP;
        this.kVP = kVP;
        this.kV = kV;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
    }

    public double calculate(double measuredAngle, double targetAngle, double targetVelocity) {
        long now = System.nanoTime();
        double dt = lastTime == 0 ? 0 : (now - lastTime) / 1e9;
        lastTime = now;

        targetAngle = getTargetAngle(measuredAngle, targetAngle);
        double positionError = targetAngle - measuredAngle;

        // Estimate measured velocity in degrees per second.
        if (dt > 1e-6 && dt < 0.1 && !Double.isNaN(previousMeasurement))
            measuredVelocity = (measuredAngle - previousMeasurement) / dt;

        previousMeasurement = measuredAngle;

        double velocityError = targetVelocity - measuredVelocity;

        double output = kP * positionError + kVP * velocityError + kV * targetVelocity;

        if (measuredAngle <= minAngle && output < 0 || measuredAngle >= maxAngle && output > 0)
            output = 0;

        return Math.max(-1.0, Math.min(1.0, output));
    }

    private double getTargetAngle(double currentAngle, double targetAngle) {
        int minTurns = (int) Math.ceil((minAngle - targetAngle) / 360.0);
        int maxTurns = (int) Math.floor((maxAngle - targetAngle) / 360.0);

        if (minTurns > maxTurns)
            return Math.max(minAngle, Math.min(maxAngle, targetAngle));

        int nearestTurns = (int) Math.round((currentAngle - targetAngle) / 360.0);

        int turns = Math.max(minTurns, Math.min(maxTurns, nearestTurns));

        return targetAngle + 360.0 * turns;
    }

    public void reset() {
        previousMeasurement = Double.NaN;
        lastTime = 0;
        measuredVelocity = 0;
    }
}