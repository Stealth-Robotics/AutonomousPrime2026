package org.firstinspires.ftc.teamcode.autos;

import static org.stealthrobotics.library.Commands.sequence;
import static org.stealthrobotics.library.Commands.waitSeconds;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.PoseSaver;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.commands.WaitBeforeCommand;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public abstract class PreloadAuto extends StealthOpMode {
    private Robot robot;
    private Follower follower;
    private PoseFactory poseFactory = PoseFactory.degrees();

    private Pose startPose;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        follower = robot.drive.getFollower();

        if (Alliance.isBlue())
            poseFactory = poseFactory.rotateAround(70.75,70.75,180);

        startPose = poseFactory.of(52.76, 7.86, -90);
        follower.setPose(startPose);

        robot.shooter.closeGate().schedule();
        robot.intake.raiseHarvester().schedule();
    }

    @Override
    public Command getAutoCommand() {
        return sequence(
//            new WaitBeforeCommand(1000, robot.shoot())
        );
    }

    @Override
    public void end() {
        PoseSaver.savedPose = robot.drive.getPose();
    }

    @Autonomous(name = "Preload Auto (Blue)", group = "Blue")
    public static class BlueAuto extends PreloadAuto { }

    @Autonomous(name = "Preload Auto (Red)", group = "Red")
    public static class RedAuto extends PreloadAuto { }
}
