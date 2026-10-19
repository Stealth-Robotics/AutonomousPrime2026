package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.PedroPoseSaver;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.stealthrobotics.library.Alliance;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.triggers.CommandGamepad;

public class Drive implements Mechanism {
    private final Follower pedroFollower;

    public Drive() {
        pedroFollower = Constants.create(RobotController.hardwareMap());
    }

    public Command teleopDrive(CommandGamepad gamepad) {
        return infinite(() -> {
            if (gamepad.rightStickButton().getAsBoolean())
                pedroFollower.setHeading(Alliance.isRed() ? 0 : 180);

            ManualDrive.driveOrHold(
                pedroFollower,
                ManualDrive.fieldCentric(
                        -gamepad.leftStickY().getValue(),
                        gamepad.leftStickX().getValue(),
                        gamepad.rightStickX().getValue(),
                        pedroFollower.pose().heading()
                )
            );
        });
    }

    public void setPose(Pose p) {
        pedroFollower.setPose(p);
    }

    public Pose getPose() {
        return pedroFollower.pose();
    }

    @Override
    public void periodic() {
        pedroFollower.update();

        Telemetry.log("Follower Mode: ", pedroFollower.mode());
    }
}
