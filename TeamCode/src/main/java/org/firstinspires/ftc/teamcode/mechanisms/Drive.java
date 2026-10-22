package org.firstinspires.ftc.teamcode.mechanisms;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

public class Drive extends StealthSubsystem {
    private final Follower follower;

    public Drive(HardwareMap h) {
        follower = Constants.create(h);
    }

    public Command teleopDrive(GamepadEx gamepad) {
        return infinite(() -> {
            if (gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).get())
                follower.setHeading(Alliance.isRed() ? 0 : Math.toRadians(180));

            follower.manual(ManualDrive.fieldCentric(
                -gamepad.getLeftY(),
                gamepad.getLeftX(),
                -gamepad.getRightX(),
                follower.pose().heading()
            ));
        });
    }

    public void setPose(Pose p) {
        follower.setPose(p);
    }

    public Pose getPose() {
        return follower.pose();
    }

    @Override
    public void periodic() {
        follower.update();

        telemetry.addData("Follower Mode: ", follower.mode());
    }
}
