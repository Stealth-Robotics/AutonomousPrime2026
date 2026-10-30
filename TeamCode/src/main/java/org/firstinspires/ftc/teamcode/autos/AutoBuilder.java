package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.api.Paths.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.PoseSaver;
import org.stealthrobotics.library.Alliance;

public class AutoBuilder {
    public final Robot robot;
    public final PoseFactory poseFactory = PoseFactory.degrees();
    public final Follower follower;

    public AutoBuilder(HardwareMap hardwareMap) {
        robot = new Robot(hardwareMap);
        follower = Constants.create(hardwareMap);

        if (Alliance.isRed())
            poseFactory.rotateAround(0,0,180);
    }

    public void initialize() {
        robot.shooter.closeGate().schedule();
        robot.intake.raiseHarvester().schedule();
    }

    public void end() {
        PoseSaver.savedPose = robot.drive.getPose();
    }

    public Path createLine(Pose p1, Pose p2) {
        return line(p1, p2).linear(p1, p2);
    }

    public Command followPath(Path p) {
        return new RunCommand(() -> follower.follow(p)).interruptOn(follower::following);
    }
}
