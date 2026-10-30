package org.firstinspires.ftc.teamcode.autos;

import static org.stealthrobotics.library.Commands.sequence;

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

//        if (Alliance.isRed())
//            poseFactory.rotateAround(0,0,180);

        startPose = poseFactory.of(52.76, 7.86, Math.toRadians(90));

        robot.shooter.closeGate().schedule();
        robot.intake.raiseHarvester().schedule();
    }

    @Override
    public Command getAutoCommand() {
        return sequence(
            new WaitCommand(1000),
            new InstantCommand(() -> follower.setPose(startPose)),
            new WaitCommand(500),
            robot.shoot()
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
