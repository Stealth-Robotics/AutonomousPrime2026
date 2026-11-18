package org.firstinspires.ftc.teamcode.util;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.MathUtils;

import org.stealthrobotics.library.Alliance;

public class ShotCalculator {
    private static final Pose3D ROBOT_TO_TURRET_OFFSET = new Pose3D(0, -2.1, 7.6);

    private static final Pose3D BLUE_RIGHT_CELL = new Pose3D(84,90,60);
    private static final Pose3D BLUE_LEFT_CELL = new Pose3D(84, 51, 60);
    private static final Pose3D RED_RIGHT_CELL = new Pose3D(57.8,51,60);
    private static final Pose3D RED_LEFT_CELL = new Pose3D(57.8, 90, 60);

    private static final double MIN_LUT_DISTANCE_INCHES = 20.1;
    private static final double MAX_LUT_DISTANCE_INCHES = 79.9;

    private final InterpLUT rpmLUT = new InterpLUT();
    private final InterpLUT hoodLUT = new InterpLUT();

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

        rpmLUT.createLUT();
        hoodLUT.createLUT();
    }

    public void update(Pose3D robotPose, Pose3D robotVelocity) {
        Pose3D targetPose = Alliance.isRed() ?
            (robotPose.y() > 72 ? RED_LEFT_CELL : RED_RIGHT_CELL) :
            (robotPose.y() > 72 ? BLUE_RIGHT_CELL : BLUE_LEFT_CELL);

        Pose3D turretPose = robotPose.plus(ROBOT_TO_TURRET_OFFSET.rotateBy(robotPose.heading()));
        Pose3D displacement = targetPose.minus(turretPose);

        double inchesAwayFromGoal = Math.hypot(displacement.x(), displacement.y());

        latestValues = new ShotValues(
            rpmLUT.get(MathUtils.clamp(inchesAwayFromGoal, MIN_LUT_DISTANCE_INCHES, MAX_LUT_DISTANCE_INCHES)),
            hoodLUT.get(MathUtils.clamp(inchesAwayFromGoal, MIN_LUT_DISTANCE_INCHES, MAX_LUT_DISTANCE_INCHES)),
            Math.toDegrees(Math.atan2(displacement.y(), displacement.x()) - robotPose.heading()),
            0
        );
    }

    public ShotValues getLatestShotValues() {
        return latestValues;
    }
}