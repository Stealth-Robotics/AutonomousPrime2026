package org.firstinspires.ftc.teamcode.util;

import org.stealthrobotics.library.Alliance;

import java.util.concurrent.TimeUnit;

public class ShotCalculator {
    private static final double GRAVITY = 386.0886; // Gravity on earth in in/s²
    private static final double LATENCY = 0.02; // Estimated delay between pose measurements and update time in seconds
    private static final double APEX_HEIGHT = 72; // inches

    private static final Pose3D ROBOT_TO_TURRET_OFFSET = new Pose3D(0, -2.1, 7.6);

    // Hive cell target poses
    private static final Pose3D BLUE_RIGHT_CELL = new Pose3D(84,90,60);
    private static final Pose3D BLUE_LEFT_CELL = new Pose3D(84, 51, 60);
    private static final Pose3D RED_RIGHT_CELL = new Pose3D(57.8,51,60);
    private static final Pose3D RED_LEFT_CELL = new Pose3D(57.8, 90, 60);

    // Distance (inches) -> RPM
    private static final LerpTable distanceToRPM = new LerpTable(
        0, 3400,
        30, 3400,
        50, 3400,
        100, 3400
    );

    private double turretAngle = 0, hoodAngle = 0, rpm = 0;

    public void update(Pose3D robotPose, Pose3D robotVelocity) {
        Pose3D targetPose = Alliance.isRed()
            ? (robotPose.y() > 72 ? RED_LEFT_CELL : RED_RIGHT_CELL)
            : (robotPose.y() < 72 ? BLUE_LEFT_CELL : BLUE_RIGHT_CELL);

        Pose3D turretOffset = ROBOT_TO_TURRET_OFFSET.rotateBy(robotPose.heading());

        Pose3D turretVelocity = new Pose3D(
            robotVelocity.x() - robotVelocity.heading() * turretOffset.y(),
            robotVelocity.y() + robotVelocity.heading() * turretOffset.x(),
            robotVelocity.z()
        );

        Pose3D futureRobotPose = robotPose.plus(robotVelocity.times(LATENCY));
        Pose3D futureTurretPose = futureRobotPose.plus(ROBOT_TO_TURRET_OFFSET.rotateBy(futureRobotPose.heading()));

        Pose3D displacement = targetPose.minus(futureTurretPose);
        double distance = Math.hypot(displacement.x(), displacement.y());

        double shotHeight = Math.max(APEX_HEIGHT, Math.max(futureTurretPose.z(), targetPose.z()) + 0.1);

        double t = Math.sqrt(2 * (shotHeight - futureTurretPose.z()) / GRAVITY)
            + Math.sqrt(2 * (shotHeight - targetPose.z()) / GRAVITY);

        double verticalVelocity = displacement.z() / t + GRAVITY * t / 2.0;

        Pose3D movingShotVelocity = new Pose3D(
            displacement.x() / t - turretVelocity.x(),
            displacement.y() / t - turretVelocity.y(),
            verticalVelocity
        );

        Pose3D staticShotVelocity = new Pose3D(
            displacement.x() / t,
            displacement.y() / t,
            verticalVelocity
        );

        double horizontalVelocity = Math.hypot(movingShotVelocity.x(), movingShotVelocity.y());

        turretAngle = Math.toDegrees(Math.atan2(movingShotVelocity.y(), movingShotVelocity.x())) -
            Math.toDegrees(futureRobotPose.heading());

        hoodAngle = 90 - Math.toDegrees(Math.atan2(movingShotVelocity.z(), horizontalVelocity));

        rpm = distanceToRPM.get(distance) *
            movingShotVelocity.magnitude() / Math.max(staticShotVelocity.magnitude(), 1e-6);
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
