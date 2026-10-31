package org.firstinspires.ftc.teamcode.autos;

import static org.stealthrobotics.library.Commands.sequence;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public abstract class ParkAuto extends Auto {
    private Path parkPath;

    @Override
    public void buildPaths() {
        // All poses are defined on the blue alliance and flipped for red autos
        Pose start = poseFactory.of(6.076, 133.3453, 90);
        Pose park = poseFactory.of(7.2099, 118.8378, 90);

        parkPath = Paths.line(start, park).constant(park);

        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand() {
        return sequence(
            followPath(parkPath)
        );
    }

    @Autonomous(name = "Park Auto (Blue)", group = "Blue")
    public static class BlueAuto extends ParkAuto { }

    @Autonomous(name = "Park Auto (Red)", group = "Red")
    public static class RedAuto extends ParkAuto { }
}
