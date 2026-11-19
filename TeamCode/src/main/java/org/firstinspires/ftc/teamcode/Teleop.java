package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Drive;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.stealthrobotics.library.opmodes.StealthOpMode;

import java.util.Map;

public abstract class Teleop extends StealthOpMode {
    private Robot robot;
    private GamepadEx driverGamepad;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        driverGamepad = new GamepadEx(gamepad1);

        // Setup default commands
        robot.drive.setDefaultCommand(robot.drive.teleopDriveDefaultCommand(driverGamepad));
        robot.intake.setDefaultCommand(robot.intake.teleopIntakeDefaultCommand(driverGamepad));

        configureBindings();
    }

    @Override
    public void onStart() {
        // Go to the correct starting configuration
        robot.intake.raiseHarvester().schedule();
        robot.shooter.closeGate().schedule();
    }

    private void configureBindings() {
        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(robot.shoot());
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
            new ConditionalCommand(
                robot.shooter.runShooter(),
                robot.shooter.stopShooter(),
                () -> robot.shooter.getState().equals(Shooter.ShooterState.STOPPED)
            )
        );

        driverGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(robot.intake.toggleHarvester());

        // Relocalize the robot to a known field reset pose (assumes that the robot is facing forward)
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(robot.drive.resetPoseTo(Drive.PoseResetLocation.FAR_HIVE_CENTER));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(robot.drive.resetPoseTo(Drive.PoseResetLocation.NEAR_RIGHT_CORNER));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(robot.drive.resetPoseTo(Drive.PoseResetLocation.NEAR_HIVE_CENTER));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(robot.drive.resetPoseTo(Drive.PoseResetLocation.NEAR_LEFT_CORNER));
    }

    @TeleOp(name = "Teleop (BLUE)", group = "Blue")
    public static class BlueTeleop extends Teleop { }

    @TeleOp(name = "Teleop (RED)", group = "Red")
    public static class RedTeleop extends Teleop { }
}