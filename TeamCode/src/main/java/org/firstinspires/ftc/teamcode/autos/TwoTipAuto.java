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
    private Path driveToGardenPath, intakeGardenPath, leaveGardenPath, path4, driveToOtherSidePath, driveToFlowerPath,
    intakeFlowerPath, backAwayPath, goToShootPath, goToOtherFlowerPath, intakeFlower2Path, driveToShootPath, alignPath,
    parkPath;

    @Override
    public void buildPaths() {
        Pose start = poseFactory.of(52.5, 8.4, -90);
        Pose driveToGarden = poseFactory.of(6.2387, 15.1752, -90);
        Pose intakeGarden = poseFactory.of(6.1236, 8.0792, -90);
        Pose leaveGarden = poseFactory.of(5.8838, 16.1962, -90);
        Pose point4 = poseFactory.of(61.326, 30.8467, -90);
        Pose driveToOtherSide = poseFactory.of(61.0353, 122.123, -90);
        Pose driveToFlowerStart = poseFactory.of(61.0353, 122.123, -90);
        Pose driveToFlower = poseFactory.of(50.7339, 112.5663, 90);
        Pose intakeFlower = poseFactory.of(50.8224, 125.2585, 90);
        Pose backAway = poseFactory.of(61.3791, 111.2058, 90);
        Pose goToShoot = poseFactory.of(61.1765, 124.0658, 90);
        Pose goToOtherFlower = poseFactory.of(27.044, 46.8957, -178.8186);
        Pose goToOtherFlowerControl1 = poseFactory.of(33.437, 102.8243, 0);
        Pose goToOtherFlowerControl2 = poseFactory.of(5.8004, 46.4577, 0);
        Pose intakeFlower2 = poseFactory.of(14.7588, 46.8197, 180);
        Pose driveToShoot = poseFactory.of(35.6489, 109.8059, -90.3636);
        Pose driveToShootControl1 = poseFactory.of(35.2061, 40.0451, 0);
        Pose align = poseFactory.of(45.6687, 126.374, -90);
        Pose park = poseFactory.of(13.3864, 89.5597, -90);
        Pose parkControl1 = poseFactory.of(42.4706, 93.8973, 0);

        driveToGardenPath = Paths.line(start, driveToGarden).linear(start, driveToGarden);
        intakeGardenPath = Paths.line(driveToGarden, intakeGarden).constant(intakeGarden)
            .with(Constants.foresightConfig.maxPathSpeed.at(0.2));
        leaveGardenPath = Paths.line(intakeGarden, leaveGarden).constant(leaveGarden);
        path4 = Paths.line(leaveGarden, point4).constant(point4)
            .with(Constants.foresightConfig.maxPathSpeed.at(0.8));
        driveToOtherSidePath = Paths.line(point4, driveToOtherSide).constant(driveToOtherSide)
            .with(Constants.foresightConfig.maxPathSpeed.at(0.6));
        driveToFlowerPath = Paths.line(driveToFlowerStart, driveToFlower).linear(driveToFlowerStart, driveToFlower);
        intakeFlowerPath = Paths.line(driveToFlower, intakeFlower).constant(intakeFlower);
        backAwayPath = Paths.line(intakeFlower, backAway).reverseTangent();
        goToShootPath = Paths.line(backAway, goToShoot).constant(goToShoot);
        goToOtherFlowerPath = Paths.curve(goToShoot, goToOtherFlowerControl1, goToOtherFlowerControl2, goToOtherFlower).reverseTangent();
        intakeFlower2Path = Paths.line(goToOtherFlower, intakeFlower2).constant(intakeFlower2);
        driveToShootPath = Paths.curve(intakeFlower2, driveToShootControl1, driveToShoot).reverseTangent();
        alignPath = Paths.line(driveToShoot, align).constant(align);
        parkPath = Paths.curve(align, parkControl1, park).constant(park);

        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand() {
        return sequence(
            // Go to starting config
            robot.intake.raiseHarvester(),
            robot.shooter.closeGate(),

            // Shoot preloaded 4 pollen
            new WaitCommand(500),
            robot.shoot(),

            // Garden intaking
            followPath(driveToGardenPath),
            robot.intake.intake(),
            followPath(intakeGardenPath, 500),
            new WaitCommand(250),
            followPath(leaveGardenPath),
            new WaitCommand(250),
            followPath(intakeGardenPath, 500),
            new WaitCommand(250),
            followPath(leaveGardenPath),
            new WaitCommand(250),
            robot.intake.stop(),

            // Drive to other side of hive
            followPath(path4),
            followPath(driveToOtherSidePath),

            // Score 4 pollen
            new WaitCommand(500),
            robot.shoot(),

            // Intake from near flower
            followPath(driveToFlowerPath),
            robot.intake.lowerHarvester(),
            robot.intake.intake(),
            new WaitCommand(500),
            followPath(intakeFlowerPath, 1000),

            // Score 4 pollen
            new WaitCommand(500),
            followPath(backAwayPath),
            robot.intake.raiseHarvester(),
            new WaitCommand(500),
            followPath(goToShootPath),
            new WaitCommand(500),
            robot.shoot(),

            // Go get other flower
            followPath(goToOtherFlowerPath),
            robot.intake.lowerHarvester(),
            robot.intake.intake(),
            new WaitCommand(500),
            followPath(intakeFlower2Path, 1000),
            new WaitCommand(500),

            // Shoot final 4 pollen
            followPath(driveToShootPath),
            robot.intake.raiseHarvester(),
            followPath(alignPath),
            robot.shoot(),

            //Park
            followPath(parkPath)
        );
    }

    @Autonomous(name = "Two Tip Auto (Blue)", group = "Blue")
    public static class BlueAuto extends TwoTipAuto { }

    @Autonomous(name = "Two Tip Auto (Red)", group = "Red")
    public static class RedAuto extends TwoTipAuto { }
}
