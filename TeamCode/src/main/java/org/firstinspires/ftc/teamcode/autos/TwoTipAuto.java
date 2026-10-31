package org.firstinspires.ftc.teamcode.autos;

import static org.stealthrobotics.library.Commands.sequence;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public abstract class TwoTipAuto extends Auto {
    private Path driveToFlower, intakeFlower, backupFromFlower, shootPath;

    @Override
    public void buildPaths() {
        // All poses are defined on the blue alliance and flipped for red autos
        Pose start = poseFactory.of(52.6409, 8.1679, -90);
        Pose flowerPath = poseFactory.of(27, 46.8812, 180);
        Pose flowerPathControl1 = poseFactory.of(37.8964, 46.9088, 0);
        Pose intake = poseFactory.of(14.8, 46.8812, 180);
        Pose backUp = poseFactory.of(27, 46.8812, -180);
        Pose shoot = poseFactory.of(31.927, 132.2673, 90);

        driveToFlower = Paths.curve(start, flowerPathControl1, flowerPath).linear(start, flowerPath);
        intakeFlower = Paths.line(flowerPath, intake).constant(intake);
        backupFromFlower = Paths.line(intake, backUp).reverseTangent();
        shootPath = Paths.line(backUp, shoot).linear(backUp, shoot);

        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand() {
        return sequence(
            new WaitCommand(1000),
            robot.autonomousShoot(),
            followPath(driveToFlower),
            robot.intake.lowerHarvester(),
            robot.intake.intake(),
            new WaitCommand(1000),
            followPath(intakeFlower),
            new WaitCommand(1000),
            followPath(backupFromFlower),
            new WaitCommand(1000),
            followPath(intakeFlower),
            new WaitCommand(1000),
            followPath(backupFromFlower),
            robot.intake.stop(),
            robot.intake.raiseHarvester(),
            followPath(shootPath),
            robot.autonomousShoot()
        );
    }

    @Autonomous(name = "Two Tip Auto (Blue)", group = "Blue")
    public static class BlueAuto extends TwoTipAuto { }

    @Autonomous(name = "Two Tip Auto (Red)", group = "Red")
    public static class RedAuto extends TwoTipAuto { }
}
