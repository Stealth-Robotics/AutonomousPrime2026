package org.firstinspires.ftc.teamcode;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.Command;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.Drive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.Pose3D;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.Set;

import dev.nextftc.hardware.RobotController;

public class Robot extends StealthSubsystem {
    public final Drive drive;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;

    private HiveSide targetHiveSide = HiveSide.LEFT;

    public enum HiveSide {
        LEFT,
        RIGHT
    }

    private static final Pose BLUE_RESET_POSE = new Pose(0, 0, 0);
    private static final Pose RED_RESET_POSE = new Pose(0, 0, 0);

    public Robot(HardwareMap h) {
        drive = new Drive(h);
        intake = new Intake(h);
        shooter = new Shooter(h);
        turret = new Turret(h);
    }

    public Command resetPose() {
        return conditional(
            Alliance::isBlue,
            instant(() -> drive.setPose(BLUE_RESET_POSE)),
            instant(() -> drive.setPose(RED_RESET_POSE))
        );
    }

    public Command toggleTargetHiveSide() {
        return conditional(
            () -> targetHiveSide.equals(HiveSide.LEFT),
            instant(() -> targetHiveSide = HiveSide.RIGHT),
            instant(() -> targetHiveSide = HiveSide.LEFT)
        );
    }

    //Returns the 3D pose of the target hive cell
    private Pose3D getTargetCellPose() {
        if (Alliance.get().equals(Alliance.BLUE))
            return targetHiveSide.equals(HiveSide.LEFT) ? FieldConstants.LEFT_BLUE_CELL : FieldConstants.RIGHT_BLUE_CELL;
        else
            return targetHiveSide.equals(HiveSide.LEFT) ? FieldConstants.LEFT_RED_CELL : FieldConstants.RIGHT_RED_CELL;
    }

    @Override
    public void periodic() {
        turret.updateTracking(drive.getPose(), getTargetCellPose());

        telemetry.addData("Alliance", Alliance.get());
        telemetry.addData("Target Hive Side", targetHiveSide);
    }
}
