package org.firstinspires.ftc.teamcode.util;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import org.stealthrobotics.library.Alliance;

public class SimpleShotCalculator {
    private static final double LATENCY = 0.01;

    private static final Pose3D ROBOT_TO_TURRET_OFFSET = new Pose3D(0, -2.1, 7.6);

    // Hive cell target poses
    private static final Pose3D BLUE_RIGHT_CELL = new Pose3D(84,90,60);
    private static final Pose3D BLUE_LEFT_CELL = new Pose3D(84, 51, 60);
    private static final Pose3D RED_RIGHT_CELL = new Pose3D(57.8,51,60);
    private static final Pose3D RED_LEFT_CELL = new Pose3D(57.8, 90, 60);

    // Distance (inches) -> RPM
    private static final LerpTable distanceToRPM = new LerpTable(
        40, 3250,
        50, 3350,
        65, 3400
    );

    // Distance (inches) -> Hood angle (degrees)
    private static final LerpTable distanceToHoodAngle = new LerpTable(
        0, 11,
        30, 11,
        50, 13,
        100, 13
    );

    private double turretAngle = 0, hoodAngle = 0, rpm = 0;

    public void update(Pose3D robotPose, Pose3D robotVelocity) {
        Pose3D targetPose = Alliance.isRed() ?
            (robotPose.y() > 72 ? RED_LEFT_CELL : RED_RIGHT_CELL) :
            (robotPose.y() < 72 ? BLUE_LEFT_CELL : BLUE_RIGHT_CELL);

        Pose3D predictedRobotPose = new Pose3D(
            robotPose.x() + robotVelocity.x() * LATENCY,
            robotPose.y() + robotVelocity.y() * LATENCY,
            robotPose.heading()
        );

        Pose3D turretPose = robotPose.plus(ROBOT_TO_TURRET_OFFSET.rotateBy(robotPose.heading()));
        Pose3D displacement = targetPose.minus(turretPose);

        double inchesAwayFromGoal = Math.hypot(displacement.x(), displacement.y());

        rpm = distanceToRPM.get(inchesAwayFromGoal);
        hoodAngle = distanceToHoodAngle.get(inchesAwayFromGoal);
        turretAngle = Math.toDegrees(Math.atan2(displacement.y(), displacement.x()) - robotPose.heading());

        telemetry.addData("Goal Distance (in)", inchesAwayFromGoal);
        telemetry.addData("Hood Angle", hoodAngle);
        telemetry.addData("RPM", rpm);
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