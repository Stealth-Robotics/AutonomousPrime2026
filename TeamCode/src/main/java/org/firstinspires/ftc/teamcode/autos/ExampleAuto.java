package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.api.Paths.*;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.PedroPoseSaver;
import org.stealthrobotics.library.Alliance;

import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

public class ExampleAuto extends NextOpMode {
    private final Robot robot;
    private final Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(24, 24, 0);
    private final Pose parkPose = poseFactory.of(72,48,90);

    private Path path1 = line(startPose, parkPose).linear(startPose, parkPose);

    public ExampleAuto(Robot robot, Alliance alliance) {
        super(robot, BulkReadHook.INSTANCE);

        Scheduler.reset();
        this.robot = robot;
        this.follower = Constants.create(hardwareMap);

        Alliance.set(alliance);

        //Flip poses based on alliance here
    }

    @Override
    public void start() {
        //Run the auto here
    }

    @Override
    public void end() {
        //Save the end of auto pose
        PedroPoseSaver.savedPose = robot.drive.getPose();
    }

    @NextAutonomous(name = "Blue Example Auto", group = "NextFTC Auto", preselectTeleop = "Blue Teleop")
    public static class BlueAuto extends ExampleAuto {
        public BlueAuto(Robot robot) {
            super(robot, Alliance.BLUE);
        }
    }

    @NextAutonomous(name = "Red Example Auto", group = "NextFTC Auto", preselectTeleop = "Red Teleop")
    public static class RedAuto extends ExampleAuto {
        public RedAuto(Robot robot) {
            super(robot, Alliance.RED);
        }
    }
}
