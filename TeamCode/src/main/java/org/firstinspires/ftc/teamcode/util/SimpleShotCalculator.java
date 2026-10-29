package org.firstinspires.ftc.teamcode.util;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

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
        0, 3400,
        30, 3400,
        50, 3400,
        100, 3400
    );

    private static final LerpTable distanceToHoodAngle = new LerpTable(
        0, 13,
        30, 13,
        50, 13,
        100, 13
    );

    private double turretAngle = 0, hoodAngle = 0, rpm = 0;

    public void update(Pose3D robotPose) {
        Pose3D targetPose = Alliance.isRed() ?
            (robotPose.y() > 72 ? RED_LEFT_CELL : RED_RIGHT_CELL) :
            (robotPose.y() < 72 ? BLUE_LEFT_CELL : BLUE_RIGHT_CELL);

        Pose3D turretPose = robotPose.plus(
            ROBOT_TO_TURRET_OFFSET.rotateBy(robotPose.heading())
        );

        Pose3D displacement = targetPose.minus(turretPose);

        double inchesAwayFromGoal = Math.hypot(displacement.x(), displacement.y());

        rpm = distanceToRPM.get(inchesAwayFromGoal);
        hoodAngle = distanceToHoodAngle.get(inchesAwayFromGoal);

        telemetry.addData("Goal Distance (in)", inchesAwayFromGoal);

        turretAngle = Math.toDegrees(Math.atan2(displacement.y(), displacement.x()) - robotPose.heading());
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