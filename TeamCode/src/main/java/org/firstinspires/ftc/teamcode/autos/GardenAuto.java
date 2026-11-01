package org.firstinspires.ftc.teamcode.autos;

import static org.stealthrobotics.library.Commands.sequence;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.api.Paths;
import com.pedropathing.config.Modifier;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public abstract class GardenAuto extends Auto {
    private Path garden, alignWithGarden, intakeGarden, goBack, driveBack, driveThroughHub, park, turn, driveBackToWall;

    @Override
    public void buildPaths() {
        // All poses are defined on the blue alliance and flipped for red autos
        Pose start = poseFactory.of(52.6409, 8.1679, -90);
        Pose gardenPath = poseFactory.of(35.6415, 17.3966, 180);
        Pose alignWithGardenPath = poseFactory.of(28.1706, 5.58, 180);
        Pose intakeGardenPath = poseFactory.of(7.5, 5.58, 180);
        Pose driveBackPathStart = poseFactory.of(7.5, 5.6078, 180);
        Pose driveBackPath = poseFactory.of(57.7936, 30.6891, -90);
        Pose driveThroughHubPath = poseFactory.of(58.1706, 115, -90);
        Pose turnPath = poseFactory.of(58, 116, 90);
        Pose driveBackToWallPath = poseFactory.of(58.1706, 133.6604, 90);
        Pose parkPath = poseFactory.of(12.1015, 95.3709, -18.9309);
        Pose parkPathControl1 = poseFactory.of(39.785, 85.876, 0);

        garden = Paths.line(start, gardenPath).linear(start, gardenPath);
        alignWithGarden = Paths.line(gardenPath, alignWithGardenPath).constant(alignWithGardenPath);
        intakeGarden = Paths.line(alignWithGardenPath, intakeGardenPath).constant(intakeGardenPath)
            .with(Constants.foresightConfig.maxPathSpeed.at(0.5));

        goBack = Paths.line(intakeGardenPath, alignWithGardenPath).constant(alignWithGardenPath);

        driveBack = Paths.line(driveBackPathStart, driveBackPath).linear(driveBackPathStart, driveBackPath);
        driveThroughHub = Paths.line(driveBackPath, driveThroughHubPath).constant(driveThroughHubPath);

        turn = Paths.line(driveThroughHubPath, turnPath).linear(driveThroughHubPath, turnPath);
        driveBackToWall = Paths.line(turnPath, driveBackToWallPath).linear(turnPath, driveBackToWallPath);

        park = Paths.curve(driveThroughHubPath, parkPathControl1, parkPath).reverseTangent();

        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand() {
        return sequence(
            new WaitCommand(1000),
            robot.autonomousShoot(),
            followPath(garden),
            robot.intake.intake(),
            followPath(alignWithGarden),
            followPath(intakeGarden, 2000),
            followPath(goBack),
            followPath(intakeGarden, 2000),
            new WaitCommand(1000),
            followPath(driveBack),
            followPath(driveThroughHub),
            followPath(turn),
            followPath(driveBackToWall, 1000),
            robot.autonomousShoot(),
            followPath(park)
        );
    }

    @Autonomous(name = "Garden Auto (Blue)", group = "Blue")
    public static class BlueAuto extends GardenAuto { }

    @Autonomous(name = "Garden Auto (Red)", group = "Red")
    public static class RedAuto extends GardenAuto { }
}
