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
    private final Robot robot;
    private final Follower follower;


    private final PoseFactory poseFactory = PoseFactory.degrees();
    private Pose startPose;
    private Pose shootPose;
    private Pose flowerPose;


    // public for testing
    public Path startToShoot;

    public AutoBuilder(HardwareMap hardwareMap, Alliance alliance) {
        robot = new Robot(hardwareMap);
        follower = Constants.create(hardwareMap);

        if(alliance == Alliance.BLUE) {
            poseFactory.rotateAround(0,0,180);
        }

        buildPoses();
        buildPaths();
    }

    public void buildPoses() {
        startPose = poseFactory.of(56,8,90);
        shootPose = poseFactory.of(56,27,90);
        flowerPose = poseFactory.of(10.8,47.2,180);
    }

    public void buildPaths() {
        startToShoot = createLine(startPose, shootPose);
    }

    public void initialize() {
        robot.shooter.closeGate().schedule();
    }
    public void end() {
        PoseSaver.savedPose = robot.drive.getPose();
    }
    public static Path createLine(Pose p1, Pose p2) {
        return line(p1, p2).linear(p1, p2);
    }
    public Command followPath(Path p) {
        return new RunCommand(() -> follower.follow(p)).interruptOn(follower::following);
    }
}
