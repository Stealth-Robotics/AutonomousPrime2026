package org.firstinspires.ftc.teamcode.autos;

import static org.stealthrobotics.library.Commands.sequence;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public abstract class PreloadAuto extends Auto {
    private Pose start;

    @Override
    public void buildPaths() {
        // All poses are defined on the blue alliance and flipped for red autos
        start = poseFactory.of(52.6409, 8.1679, -90);

        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand() {
        return sequence(
            new WaitCommand(1000),
            robot.autonomousShoot()
        );
    }

    @Autonomous(name = "Preload Auto (Blue)", group = "Blue")
    public static class BlueAuto extends PreloadAuto { }

    @Autonomous(name = "Preload Auto (Red)", group = "Red")
    public static class RedAuto extends PreloadAuto { }
}
