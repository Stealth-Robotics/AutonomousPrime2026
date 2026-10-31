package org.firstinspires.ftc.teamcode.autos;

import static org.stealthrobotics.library.Commands.sequence;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public abstract class TwoTipAuto extends Auto {
    private Pose start, flower1Prep, flower1Intake, shoot1, driveToShoot1Handle1;
    private Pose driveToShoot1Handle2, flower2Prep, flower2Intake, shoot2, park;
    private Path driveToFlower1, intakeFromFlower1, driveToShoot1;
    private Path driveToFlower2, intakeFromFlower2, driveToShoot2, driveToPark;

    @Override
    public void buildPaths() {
        // All poses are defined on the blue alliance and flipped for red autos
        // "Handle" -> curve midpoint

        start = poseFactory.of(52.6409, 8.1679, -90);
        flower1Prep = poseFactory.of(20, 46.5927, 180);
        flower1Intake = poseFactory.of(13.5, 46.653, 180);
        driveToShoot1Handle1 = poseFactory.of(30.1426, 65.4183, 0);
        driveToShoot1Handle2 = poseFactory.of(32.0212, 104.9496, 0);
        shoot1 = poseFactory.of(53.0859, 126.1724, 90);
        flower2Prep = poseFactory.of(46.4129, 123.1745, 90);
        flower2Intake = poseFactory.of(46.7082, 128.5, 90);
        shoot2 = poseFactory.of(46.747, 123.7771, 90);
        park = poseFactory.of(16.9435, 121.6391, 0);

        driveToFlower1 = Paths.line(start, flower1Prep).linear(start, flower1Prep);
        intakeFromFlower1 = Paths.line(flower1Prep, flower1Intake).linear(flower1Prep, flower1Intake);
        driveToShoot1 = Paths.through(flower1Intake, driveToShoot1Handle1, driveToShoot1Handle2, shoot1).linear(flower1Intake, shoot1);
        driveToFlower2 = Paths.line(shoot1, flower2Prep).linear(shoot1, flower2Prep);
        intakeFromFlower2 = Paths.line(flower2Prep, flower2Intake).linear(flower2Prep, flower2Intake);
        driveToShoot2 = Paths.line(flower2Intake, shoot2).linear(flower2Intake, shoot2);
        driveToPark = Paths.line(shoot2, park).linear(shoot2, park);

        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand() {
        return sequence(
                robot.shoot(),
                robot.intake.lowerHarvester(),
                followPath(driveToFlower1),
                harvestWith(intakeFromFlower1),
                retractWith(driveToShoot1),
                robot.shoot(),
                robot.intake.lowerHarvester(),
                followPath(driveToFlower2),
                harvestWith(intakeFromFlower2),
                retractWith(driveToShoot2),
                robot.shoot(),
                followPath(driveToPark)
        );
    }

    private Command harvestWith(Path intakePath) {
        return sequence(
                robot.intake.intake(),
                followPath(intakePath),
                new WaitCommand(500), // Wait for balls to enter intake
                robot.intake.stop()
        );
    }

    private Command retractWith(Path intakePath) {
        return followPath(intakePath).alongWith(
                new WaitCommand(500), // Wait until clear from flower
                robot.intake.raiseHarvester()
        );
    }

    @Autonomous(name = "Two Tip Auto (Blue)", group = "Blue")
    public static class BlueAuto extends TwoTipAuto { }

    @Autonomous(name = "Two Tip Auto (Red)", group = "Red")
    public static class RedAuto extends TwoTipAuto { }
}
