package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.utils.BijectiveMap;

import org.stealthrobotics.library.Alliance;

// TODO: rpm, feedforward, interpolation, t-tuning
public class ShotCalculator {
    private final double GRAVITY = 9.80665; // Gravity on earth in m/s²
    private final double LATENCY = 0.02; // Estimated delay between pose measurements and update time in seconds

    private final Pose3D ROBOT_TO_TURRET_OFFSET = new Pose3D(0, 0, 0, 0);

    // Hive cell target poses (TODO: Find actual poses from CAD)
    private final Pose3D BLUE_RIGHT_CELL = new Pose3D(86,80,64);
    private final Pose3D BLUE_LEFT_CELL = new Pose3D(58, 80, 64);
    private final Pose3D RED_RIGHT_CELL = new Pose3D(58,64,64);
    private final Pose3D RED_LEFT_CELL = new Pose3D(86, 64, 64);

    // Interpolation map for RPM values at given distances in inches
    private final BijectiveMap.InterpolatableMap distanceToRPM = new BijectiveMap.InterpolatableMap();

    private double turretAngle = 0, hoodAngle = 0, rpm = 0;

    public ShotCalculator() {
        distanceToRPM.put(0.0, 1000.0);
        distanceToRPM.put(144.0, 2000.0);
    }

    public void update(Pose3D robotPose, Pose3D robotVelocity) {
        Pose3D targetPose = Alliance.isRed() ?
            (robotPose.y() < 72 ? RED_RIGHT_CELL : RED_LEFT_CELL) :
            (robotPose.y() > 72 ? BLUE_RIGHT_CELL : BLUE_LEFT_CELL);

        Pose3D turretPose = robotPose.plus(ROBOT_TO_TURRET_OFFSET.rotateBy(robotPose.heading())); // Get rotated robot position
        turretPose = turretPose.plus(robotVelocity.times(LATENCY));

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

        double turretLatencyOffset = robotVelocity.heading() * LATENCY;

        // Ratio between moving shot velocity and (hypothetical) static shot velocity
        double movingShotVelocityRatio = movingShotVelocity.magnitude() / launchVelocity.magnitude();

        // Field relative turret angle is the angle of the horizontal components of the shot vector
        double fieldRelativeTurretAngle = Math.toDegrees(Math.atan2(movingShotVelocity.y(), movingShotVelocity.x()));

        // Robot relative turret angle is acquired by subtracting robot heading and a latency term from field relative turret angle
        turretAngle = fieldRelativeTurretAngle - robotPose.heading() - turretLatencyOffset;

        // "90 - ..." results from the perpendicular angle between hood lift and fire angle
        hoodAngle = 90 - Math.toDegrees(Math.atan2(movingShotVelocity.z(), horizontalVelocity));

        // RPM is derived from the tuned rpm interpolation table, and multiplied by a moving shot velocity factor for velocity compensation
        rpm = movingShotVelocityRatio * distanceToRPM.getOrDefault(horizontalDistance,0.0);
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
