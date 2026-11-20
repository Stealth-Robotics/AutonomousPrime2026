package org.firstinspires.ftc.teamcode.autos;

import static org.stealthrobotics.library.Commands.sequence;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public abstract class TwoTipAuto extends Auto {
    private Pose start, driveToGarden, intakeGarden, leaveGarden,
        point4, driveToOtherSide, driveToFlowerStart, driveToFlower;
    private Path driveToGardenPath, intakeGardenPath, leaveGardenPath, path4, driveToOtherSidePath, driveToFlowerPath;

    @Override
    public void buildPaths() {
        start = poseFactory.of(52.5, 8.4, -90);
        driveToGarden = poseFactory.of(6.2387, 15.1752, -90);
        intakeGarden = poseFactory.of(6.1236, 8.0792, -90);
        leaveGarden = poseFactory.of(5.8838, 16.1962, -90);
        point4 = poseFactory.of(61.326, 30.8467, -90);
        driveToOtherSide = poseFactory.of(61.0353, 122.123, -90);
        driveToFlowerStart = poseFactory.of(61.0353, 122.123, -90);
        driveToFlower = poseFactory.of(46.8788, 115.3733, 90);

        driveToGardenPath = Paths.line(start, driveToGarden).constant(driveToGarden);
        intakeGardenPath = Paths.line(driveToGarden, intakeGarden).constant(intakeGarden)
            .with(Constants.foresightConfig.maxPathSpeed.at(0.2));
        leaveGardenPath = Paths.line(intakeGarden, leaveGarden).constant(leaveGarden);
        path4 = Paths.line(leaveGarden, point4).constant(point4);
        driveToOtherSidePath = Paths.line(point4, driveToOtherSide).constant(driveToOtherSide);
        driveToFlowerPath = Paths.line(driveToFlowerStart, driveToFlower).linear(driveToFlowerStart, driveToFlower);

        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand() {
        return sequence(
            robot.intake.raiseHarvester(),
            robot.shooter.closeGate(),
            new WaitCommand(500),
            robot.shoot(),
            followPath(driveToGardenPath),
            robot.intake.intake(),
            followPath(intakeGardenPath, 500),
            new WaitCommand(250),
            followPath(leaveGardenPath),
            new WaitCommand(250),
            followPath(intakeGardenPath, 500),
            new WaitCommand(250),
            followPath(leaveGardenPath),
            robot.intake.stop(),
            followPath(path4),
            followPath(driveToOtherSidePath),
            new WaitCommand(400),
            robot.shoot(),
            followPath(driveToFlowerPath),
            robot.intake.lowerHarvester()
        );
    }

    @Autonomous(name = "Two Tip Auto (Blue)", group = "Blue")
    public static class BlueAuto extends TwoTipAuto { }

    @Autonomous(name = "Two Tip Auto (Red)", group = "Red")
    public static class RedAuto extends TwoTipAuto { }
}
