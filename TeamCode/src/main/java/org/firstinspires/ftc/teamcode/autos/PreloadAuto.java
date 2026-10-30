package org.firstinspires.ftc.teamcode.autos;

import static org.stealthrobotics.library.Commands.sequence;

import com.arcrobotics.ftclib.command.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.stealthrobotics.library.commands.WaitBeforeCommand;

public abstract class PreloadAuto extends Auto {
    @Override
    public void buildPaths() {
        // All poses are defined on the blue alliance and flipped for red autos
        Pose startPose = poseFactory.of(52.76, 7.86, -90);

        follower.setPose(startPose);
    }

    @Override
    public Command getAutoCommand() {
        return sequence(
            robot.shoot()
        );
    }

    @Autonomous(name = "Preload Auto (Blue)", group = "Blue")
    public static class BlueAuto extends PreloadAuto { }

    @Autonomous(name = "Preload Auto (Red)", group = "Red")
    public static class RedAuto extends PreloadAuto { }
}
