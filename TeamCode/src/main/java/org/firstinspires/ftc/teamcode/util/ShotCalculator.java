package org.firstinspires.ftc.teamcode.util;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.MathUtils;
import com.pedropathing.math.Velocity;

import org.stealthrobotics.library.Alliance;

public class ShotCalculator {
    private static final Pose3D ROBOT_TO_TURRET_OFFSET = new Pose3D(0, -2.1, 7.6);

    private static final Pose3D BLUE_RIGHT_CELL = new Pose3D(84,90,60);
    private static final Pose3D BLUE_LEFT_CELL = new Pose3D(84, 51, 60);
    private static final Pose3D RED_RIGHT_CELL = new Pose3D(57.8,51,60);
    private static final Pose3D RED_LEFT_CELL = new Pose3D(57.8, 90, 60);

    private static final double MIN_LUT_DISTANCE_INCHES = 20.1;
    private static final double MAX_LUT_DISTANCE_INCHES = 79.9;

    private static final int CONVERGENCE_ITERATIONS = 5;

    private final InterpLUT rpmLUT = new InterpLUT();
    private final InterpLUT hoodLUT = new InterpLUT();
    private final InterpLUT flightTimeLUT = new InterpLUT();

    private ShotValues latestValues;

    public ShotCalculator() {
        rpmLUT.add(20, 3000);
        rpmLUT.add(42, 3100);
        rpmLUT.add(53, 3300);
        rpmLUT.add(60, 3400);
        rpmLUT.add(80, 3600);

        hoodLUT.add(20, 5);
        hoodLUT.add(36, 8);
        hoodLUT.add(42, 10);
        hoodLUT.add(60, 13);
        hoodLUT.add(80, 15);

        // Experimental data, need to measure actual values
        flightTimeLUT.add(20, 0.4);
        flightTimeLUT.add(36, 0.4);
        flightTimeLUT.add(42, 0.4);
        flightTimeLUT.add(60, 0.45);
        flightTimeLUT.add(80, 0.6);

        rpmLUT.createLUT();
        hoodLUT.createLUT();
        flightTimeLUT.createLUT();
    }

    public void update(Pose3D robotPose, Velocity robotVelocity) {
        Pose3D targetPose = Alliance.isRed() ?
            (robotPose.y() > 72 ? RED_LEFT_CELL : RED_RIGHT_CELL) :
            (robotPose.y() > 72 ? BLUE_RIGHT_CELL : BLUE_LEFT_CELL);

        Pose3D turretOffset = ROBOT_TO_TURRET_OFFSET.rotateBy(robotPose.heading());

        double initialDistance = Math.hypot(
            targetPose.x() - (robotPose.x() + turretOffset.x()),
            targetPose.y() - (robotPose.y() + turretOffset.y())
        );

        double turretX = robotPose.x() + turretOffset.x();
        double turretY = robotPose.y() + turretOffset.y();

        double turretVelocityX = robotVelocity.vx - robotVelocity.omega * turretOffset.y();
        double turretVelocityY = robotVelocity.vy + robotVelocity.omega * turretOffset.x();

        double flightTime = flightTimeLUT.get(
            MathUtils.clamp(initialDistance, MIN_LUT_DISTANCE_INCHES, MAX_LUT_DISTANCE_INCHES)
        );

        double dx = 0;
        double dy = 0;
        double predictedDistance = initialDistance;

        for (int i = 0; i < CONVERGENCE_ITERATIONS; i++) {
            // Compensate for the velocity inherited by the projectile
            dx = targetPose.x() - turretX - turretVelocityX * flightTime;
            dy = targetPose.y() - turretY - turretVelocityY * flightTime;

            predictedDistance = Math.hypot(dx, dy);

            flightTime = flightTimeLUT.get(
                MathUtils.clamp(predictedDistance, MIN_LUT_DISTANCE_INCHES, MAX_LUT_DISTANCE_INCHES)
            );
        }

        double targetBearing = Math.atan2(dy, dx);

        double turretTargetAngle = Math.toDegrees(targetBearing - robotPose.heading());

        double relativeVelocityX = -turretVelocityX;
        double relativeVelocityY = -turretVelocityY;

        double distanceSquared = dx * dx + dy * dy;

        double bearingVelocity = (dx * relativeVelocityY - dy * relativeVelocityX) / distanceSquared;

        double turretTargetVelocity = Math.toDegrees(
            bearingVelocity - robotVelocity.omega
        );

        double lookupDistance = MathUtils.clamp(predictedDistance, MIN_LUT_DISTANCE_INCHES, MAX_LUT_DISTANCE_INCHES);

        latestValues = new ShotValues(
            rpmLUT.get(lookupDistance),
            hoodLUT.get(lookupDistance),
            turretTargetAngle,
            turretTargetVelocity
        );


        telemetry.addData("Target Angle", turretTargetAngle);
        telemetry.addData("Flight Time", flightTime);
        telemetry.addData("Turret Target Velocity", turretTargetVelocity);

        telemetry.addLine("--------------------------------\n\n");
    }

    public ShotValues getLatestShotValues() {
        return latestValues;
    }
}