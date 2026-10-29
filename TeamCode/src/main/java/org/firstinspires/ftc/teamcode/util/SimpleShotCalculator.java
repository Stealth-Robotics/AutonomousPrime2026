package org.firstinspires.ftc.teamcode.util;

import org.stealthrobotics.library.Alliance;

import java.util.concurrent.TimeUnit;

public class SimpleShotCalculator {
    private static final Pose3D ROBOT_TO_TURRET_OFFSET = new Pose3D(0, -2.1, 7.6);

    // Hive cell target poses
    private static final Pose3D BLUE_RIGHT_CELL = new Pose3D(84,87.5,60);
    private static final Pose3D BLUE_LEFT_CELL = new Pose3D(84, 54, 60);
    private static final Pose3D RED_RIGHT_CELL = new Pose3D(57.8,54,60);
    private static final Pose3D RED_LEFT_CELL = new Pose3D(57.8, 87.5, 60);

    // Distance (inches) -> RPM
    private static final LerpTable distanceToRPM = new LerpTable(
        0, 3300,
        30, 3400,
        52, 3550,
        100, 3550
    );

    private static final double HOOD_ANGLE = 22.5;
    private double turretAngle = 0, rpm = 0;

    public void update(Pose3D robotPose) {
        Pose3D targetPose = Alliance.isRed() ?
            (robotPose.y() > 72 ? RED_LEFT_CELL : RED_RIGHT_CELL) :
            (robotPose.y() < 72 ? BLUE_LEFT_CELL : BLUE_RIGHT_CELL);

        Pose3D turretPose = robotPose.plus(
            ROBOT_TO_TURRET_OFFSET.rotateBy(robotPose.heading())
        );

        Pose3D displacement = targetPose.minus(turretPose);

        rpm = distanceToRPM.get(Math.hypot(displacement.x(), displacement.y()));
        turretAngle = Math.toDegrees(Math.atan2(displacement.y(), displacement.x()) - robotPose.heading());
    }

    public double getTurretAngle() {
        return turretAngle;
    }
    public double getHoodAngle() {
        return HOOD_ANGLE;
    }
    public double getRPM() {
        return rpm;
    }
}