package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.*;

import androidx.annotation.NonNull;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.mechanisms.Drive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.Pose3D;
import org.stealthrobotics.library.Alliance;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class Robot implements NextRobot {
    public final Drive drive = new Drive();
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final Turret turret = new Turret();

    private HiveSide targetHiveSide = HiveSide.LEFT;

    public enum HiveSide {
        LEFT,
        RIGHT
    }

    private static final Pose BLUE_RESET_POSE = new Pose(0, 0, 0);
    private static final Pose RED_RESET_POSE = new Pose(0, 0, 0);

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
    }

    @Override @NonNull
    public Set<Mechanism> getMechanisms() {
        return Set.of(drive, intake, shooter, turret);
    }
}
