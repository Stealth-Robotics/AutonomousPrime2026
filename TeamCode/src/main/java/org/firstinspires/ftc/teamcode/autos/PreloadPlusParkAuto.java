package org.firstinspires.ftc.teamcode.autos;

import static org.stealthrobotics.library.Commands.sequence;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public abstract class PreloadPlusParkAuto extends Auto {
    private Pose start;
    private Pose parkPath;
    private Pose parkPathControl1;

    private Path park;

    @Override
    public void buildPaths() {
        // All poses are defined on the blue alliance and flipped for red autos
        start = poseFactory.of(52.6409, 8.1679, -90);
        parkPath = poseFactory.of(8.6374, 96.127, -90);
        parkPathControl1 = poseFactory.of(38.7789, 37.8307, 0);

        park = Paths.curve(start, parkPathControl1, parkPath).constant(parkPath);

        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand() {
        return sequence(
            new WaitCommand(1000),
            robot.autonomousShoot(),
            followPath(park)
        );
    }

    @Autonomous(name = "Preload + Park Auto (Blue)", group = "Blue")
    public static class BlueAuto extends PreloadPlusParkAuto { }

    @Autonomous(name = "Preload + Park Auto (Red)", group = "Red")
    public static class RedAuto extends PreloadPlusParkAuto { }
}
