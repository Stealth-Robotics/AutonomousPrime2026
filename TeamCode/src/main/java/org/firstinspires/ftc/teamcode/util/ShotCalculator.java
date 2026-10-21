package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.mechanisms.Turret;

import dev.nextftc.control.util.InterpolatingMap;

// TODO: Latency, rpm, feedforward, interpolation, t-tuning
public class ShotCalculator {
    private static final double GRAVITY = 9.80665; // Gravity on earth in m/s²
    private static final double LATENCY = 0.02; // Estimated delay between pose measurements and update time in seconds

    private static final Pose3D ROBOT_TO_TURRET_OFFSET = new Pose3D(0, 0, 0, 0);

    // Interpolation map for RPM values at given distances in inches
    private final InterpolatingMap<Double> distanceToRPM = InterpolatingMap.linear();


    private double turretAngle = 0, hoodAngle = 0, rpm = 0;

    public ShotCalculator() {
        // Units = (inches, RPM)
        distanceToRPM.put(0.0, 1000.0);
        distanceToRPM.put(144.0, 2000.0);
    }

    public void update(Pose3D robotPose, Pose3D robotVelocity, Pose3D targetPose) {
        Pose3D turretPose = robotPose.plus(Turret.ROBOT_TO_TURRET_OFFSET.rotateBy(robotPose.heading())); // Get Rotated robot position

        Pose3D shotDisplacement = targetPose.minus(turretPose);

        double t = 1.0; // Arbitrary time of flight measurement in seconds [NEED TO UPDATE SOON]

        // Vector representing a field relative launch velocity vector
        Pose3D launchVelocity = new Pose3D(
                (targetPose.x() - robotPose.x()) / t,
                (targetPose.y() - robotPose.y()) / t,
                (targetPose.z() - robotPose.z()) / t + GRAVITY * t * 2.0
        );

        Pose3D movingShotVelocity = launchVelocity.minus(robotVelocity);
        double horizontalVelocity = Math.hypot(movingShotVelocity.x(), movingShotVelocity.y());
        double horizontalDistance = Math.hypot(shotDisplacement.x(), shotDisplacement.y());

        // Turret angle is the angle of the horizontal components of the shot vector
        turretAngle = Math.toDegrees(Math.atan2(movingShotVelocity.y(), movingShotVelocity.x()));

        // "90 - ..." results from the perpendicular angle between hood lift and fire angle
        hoodAngle = 90 - Math.toDegrees(Math.atan2(movingShotVelocity.z(), horizontalVelocity));

        // RPM is derived from the tuned rpm interpolation table
        rpm = distanceToRPM.get(horizontalDistance);
    }

    public double getTurretAngle() {
        return turretAngle;
    }
    public double getHoodAngle() {
        return hoodAngle;
    }
    public double getRPM() {
        return rpm;
    }
}
