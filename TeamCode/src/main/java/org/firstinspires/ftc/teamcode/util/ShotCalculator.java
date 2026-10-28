package org.firstinspires.ftc.teamcode.util;

import org.stealthrobotics.library.Alliance;

import java.util.concurrent.TimeUnit;

public class ShotCalculator {
    private static final double GRAVITY = 386.0886; // Gravity on earth in in/s²
    private static final double LATENCY = 0.02; // Estimated delay between pose measurements and update time in seconds

    private static final Pose3D ROBOT_TO_TURRET_OFFSET = new Pose3D(0, -2.1, 7.6);

    // Hive cell target poses
    private static final Pose3D BLUE_RIGHT_CELL = new Pose3D(84,80,60);
    private static final Pose3D BLUE_LEFT_CELL = new Pose3D(84, 61.5, 60);
    private static final Pose3D RED_RIGHT_CELL = new Pose3D(57.8,61.5,60);
    private static final Pose3D RED_LEFT_CELL = new Pose3D(57.8, 80, 60);

    // Interpolation map for RPM values at given distances in inches
    private static final LerpTable distanceToRPM = new LerpTable(
        0, 3300, //Min
        30, 3500,
        52, 3700,
        100, 3700 //Max
    );

    private double pastShotTurretAngle = 0;
    private long pastTimeStamp = System.nanoTime();

    private double turretAngle = 0, turretFeedForward = 0, hoodAngle = 0, rpm = 0;

    public ShotCalculator() {

    }

    public void update(Pose3D robotPose, Pose3D robotVelocity) {
        Pose3D targetPose = Alliance.isRed() ?
            (robotPose.y() > 72 ? RED_LEFT_CELL : RED_RIGHT_CELL) :
            (robotPose.y() < 72 ? BLUE_LEFT_CELL : BLUE_RIGHT_CELL);

        pastShotTurretAngle = turretAngle;

        Pose3D turretPose = robotPose.plus(ROBOT_TO_TURRET_OFFSET.rotateBy(robotPose.heading())); // Get rotated robot position
        turretPose = turretPose.plus(robotVelocity.times(LATENCY));

        Pose3D shotDisplacement = targetPose.minus(turretPose);

        double t = 1.0; // Arbitrary time of flight measurement in seconds [NEED TO UPDATE SOON]

        // Vector representing a field relative launch velocity vector
        Pose3D launchVelocity = new Pose3D(
                (targetPose.x() - robotPose.x()) / t,
                (targetPose.y() - robotPose.y()) / t,
                (targetPose.z() - robotPose.z()) / t + GRAVITY * t * 0.5
        );

        Pose3D movingShotVelocity = launchVelocity.minus(robotVelocity);
        double horizontalVelocity = Math.hypot(movingShotVelocity.x(), movingShotVelocity.y());
        double horizontalDistance = Math.hypot(shotDisplacement.x(), shotDisplacement.y());

        double turretLatencyOffset = robotVelocity.heading() * LATENCY;

        // Ratio between moving shot velocity and (hypothetical) static shot velocity
        double movingShotVelocityRatio = movingShotVelocity.magnitude() / launchVelocity.magnitude();

        // Field relative turret angle is the angle of the horizontal components of the shot vector
        double fieldRelativeTurretAngle = Math.toDegrees(Math.atan2(movingShotVelocity.y(), movingShotVelocity.x()));

        // Robot relative turret angle is acquired by subtracting robot heading and a latency term from field relative turret angle
        turretAngle = fieldRelativeTurretAngle - robotPose.heading() - turretLatencyOffset;

        // Rate of change of turret angle with respect to time
        //TODO: research better methods to do this
        turretFeedForward = (turretAngle - pastShotTurretAngle) / TimeUnit.NANOSECONDS.toSeconds(System.nanoTime() - pastTimeStamp);
        pastTimeStamp = System.nanoTime();

        // "90 - ..." results from the perpendicular angle between hood lift and fire angle
        hoodAngle = 90 - Math.toDegrees(Math.atan2(movingShotVelocity.z(), horizontalVelocity));

        // RPM is derived from the tuned rpm interpolation table, and multiplied by a moving shot velocity factor for velocity compensation
        rpm = movingShotVelocityRatio * distanceToRPM.get(horizontalDistance);
    }

    public double getTurretFeedForward() {
        return turretFeedForward;
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
