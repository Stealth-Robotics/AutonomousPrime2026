package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.api.Paths.line;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.PoseSaver;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public abstract class Auto extends StealthOpMode {
    protected Robot robot;
    protected Follower follower;
    protected PoseFactory poseFactory = PoseFactory.degrees();

    private static final Pose FIELD_CENTER = new Pose(70.75, 70.75);

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        follower = robot.drive.getFollower();

        if (Alliance.isBlue())
            poseFactory = poseFactory.rotateAround(FIELD_CENTER,180);

        buildPaths();

        // Starting configuration
        robot.shooter.closeGate().schedule();
        robot.intake.raiseHarvester().schedule();
    }

    @Override
    public void onStart() {

    }

    @Override
    public void end() {
        PoseSaver.savedPose = robot.drive.getPose();
    }

    public void buildPaths() {}

    public Path createLine(Pose p1, Pose p2) {
        return line(p1, p2).linear(p1, p2);
    }

    public Command followPath(Path p) {
        return new InstantCommand(() -> follower.follow(p))
            .andThen(new WaitUntilCommand(() -> !follower.following()));
    }
}
