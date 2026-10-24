package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.api.Paths.line;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.PoseSaver;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public abstract class ExampleAuto extends StealthOpMode {
    private final Robot robot;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(24, 24, 0);
    private final Pose parkPose = poseFactory.of(72,48,90);

    private Path path1 = line(startPose, parkPose).linear(startPose, parkPose);

    public ExampleAuto() {
        robot = new Robot(hardwareMap);

        // Flip poses based on alliance here
    }

    @Override
    public void initialize() {
        // Go to the correct starting configuration
        robot.shooter.closeGate().schedule();

        CommandScheduler.getInstance().schedule(
            // Put auto here:
        );
    }

    @Override
    public void end() {
        PoseSaver.savedPose = robot.drive.getPose();
    }

    @Autonomous(name = "Example Auto (Blue)", group = "Blue")
    public static class BlueAuto extends ExampleAuto {
        public BlueAuto() {
            Alliance.set(Alliance.BLUE);
        }
    }

    @Autonomous(name = "Example Auto (Red)", group = "Red")
    public static class RedAuto extends ExampleAuto {
        public RedAuto() {
            Alliance.set(Alliance.RED);
        }
    }
}
