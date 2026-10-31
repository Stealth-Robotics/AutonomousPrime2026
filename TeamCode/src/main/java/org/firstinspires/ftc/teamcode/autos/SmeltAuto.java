package org.firstinspires.ftc.teamcode.autos;

import static org.stealthrobotics.library.Commands.sequence;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public abstract class SmeltAuto extends Auto {
    private Path parkPath;

    @Override
    public void buildPaths() {
        // All poses are defined on the blue alliance and flipped for red autos
        Pose start = poseFactory.of(37.572086328314256, 133.1342555319616, 90);
        Pose park = poseFactory.of(5.8059, 89.0022, 90);

        parkPath = Paths.line(start, park).linear(start, park);

        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand() {
        return sequence(
            new WaitCommand(13000),
            robot.farAutonomousShoot(),
            followPath(parkPath)
        );
    }

    @Autonomous(name = "Smelt Auto (Blue)", group = "Blue")
    public static class BlueAuto extends SmeltAuto { }

    @Autonomous(name = "Smelt Auto (Red)", group = "Red")
    public static class RedAuto extends SmeltAuto { }
}
