package org.firstinspires.ftc.teamcode.systems;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.util.HiveSide;
import org.firstinspires.ftc.teamcode.util.Pose3D;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

public class Robot extends StealthSubsystem {
    public final Drive drive;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;

    private HiveSide targetHiveSide = HiveSide.LEFT;

    private static final Pose BLUE_RESET_POSE = new Pose(0, 0, 0);
    private static final Pose RED_RESET_POSE = new Pose(0, 0, 0);

    public Robot(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
    }

    //Resets our robot pose to a known location
    public void resetPose() {
        if (Alliance.get().equals(Alliance.BLUE))
            drive.setPose(BLUE_RESET_POSE);
        else
            drive.setPose(RED_RESET_POSE);
    }

    //Allows the operator to control which side of the hive we are aiming at
    public void toggleTargetHiveSide() {
        if (targetHiveSide.equals(HiveSide.LEFT))
            targetHiveSide = HiveSide.RIGHT;
        else
            targetHiveSide = HiveSide.LEFT;
    }

    //Get the 3D pose of the hive cell we want to aim at
    private Pose3D getTargetCellPose() {
        if (Alliance.get().equals(Alliance.BLUE))
            return targetHiveSide.equals(HiveSide.LEFT) ? FieldConstants.LEFT_BLUE_CELL : FieldConstants.RIGHT_BLUE_CELL;
        else
            return targetHiveSide.equals(HiveSide.LEFT) ? FieldConstants.LEFT_RED_CELL : FieldConstants.RIGHT_RED_CELL;
    }

    @Override
    public void periodic() {
        turret.updateTracking(drive.getPose(), getTargetCellPose());
    }
}
